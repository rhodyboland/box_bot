"""HTTP and ROS navigation integration on an isolated domain with a fake robot."""
import csv
from datetime import datetime, timezone
import http.client
import json
import math
from pathlib import Path
import sys
import threading
import time

import pytest
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.msg import WaypointStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import BatteryState, Image, Imu
from ublox_ubx_msgs.msg import UBXNavPVT

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from boxbot_ui import BoxbotUI
from ui_http import validate_filter, validate_goal, validate_waypoints
from imu_tilt_cloud_filter import ImuTiltCloudFilter
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


@pytest.mark.parametrize('payload', [None, {}, {'latitude': True, 'longitude': 0, 'heading': 0},
                                   {'latitude': 91, 'longitude': 0, 'heading': 0},
                                   {'latitude': 0, 'longitude': math.nan, 'heading': 0}])
def test_invalid_targets(payload):
    with pytest.raises(ValueError):
        validate_goal(payload)


def test_http_goal_conversion_cancellation_and_telemetry():
    rclpy.init(domain_id=178, args=['--ros-args', '-p', 'port:=0', '-p', 'host:=127.0.0.1'])
    ui = BoxbotUI()
    fake = rclpy.create_node('fake_boxbot')
    ground_filter = ImuTiltCloudFilter()
    group = ReentrantCallbackGroup()
    received, canceled = [], []
    routes, route_cancels = [], []
    finish_route = threading.Event()
    route_failure = threading.Event()
    pose_position = [0.0, 0.0]
    conversion_delay = [0.0]
    conversion_count = [0]
    conversion_active = [0, 0]
    conversion_lock = threading.Lock()
    stop = threading.Event()
    delay_conversion = threading.Event()
    angle = math.radians(25)
    c, s = math.cos(angle), math.sin(angle)

    def to_ll(request, response):
        x, y = request.map_point.x, request.map_point.y
        response.ll_point.latitude = -27 + (s * x + c * y) / 111320
        response.ll_point.longitude = 153 + (c * x - s * y) / (111320 * math.cos(math.radians(-27)))
        return response

    def from_ll(request, response):
        with conversion_lock:
            conversion_count[0] += 1
            conversion_active[0] += 1
            conversion_active[1] = max(conversion_active)
        time.sleep(conversion_delay[0])
        if delay_conversion.is_set():
            time.sleep(0.4)
        north = (request.ll_point.latitude + 27) * 111320
        east = (request.ll_point.longitude - 153) * 111320 * math.cos(math.radians(-27))
        response.map_point.x, response.map_point.y = c * east + s * north, -s * east + c * north
        with conversion_lock:
            conversion_active[0] -= 1
        return response

    def lifecycle(request, response):
        response.current_state.id, response.current_state.label = 3, 'active'
        return response

    def execute(handle):
        received.append(handle.request.pose)
        while not stop.wait(0.02):
            if handle.is_cancel_requested:
                canceled.append(handle.goal_id)
                handle.canceled()
                return NavigateToPose.Result()
        handle.abort()
        return NavigateToPose.Result()

    def execute_route(handle):
        routes.append(handle.request)
        feedback = FollowWaypoints.Feedback()
        feedback.current_waypoint = 1
        handle.publish_feedback(feedback)
        while not stop.wait(0.02):
            if handle.is_cancel_requested:
                route_cancels.append(handle.goal_id)
                handle.canceled()
                return FollowWaypoints.Result()
            if finish_route.is_set():
                result = FollowWaypoints.Result()
                if route_failure.is_set():
                    result.error_code = FollowWaypoints.Result.STOP_ON_MISSED_WAYPOINT
                    result.error_msg = 'Waypoint unreachable'
                    missed = WaypointStatus()
                    missed.waypoint_index = 0
                    missed.error_code = 107
                    missed.error_msg = 'Costmap timed out waiting for update'
                    result.missed_waypoints = [missed]
                    handle.abort()
                else:
                    handle.succeed()
                return result
        handle.abort()
        return FollowWaypoints.Result()

    services = [fake.create_service(ToLL, '/toLL', to_ll, callback_group=group),
                fake.create_service(FromLL, '/fromLL', from_ll, callback_group=group)]
    services += [fake.create_service(GetState, f'/{name}/get_state', lifecycle, callback_group=group)
                 for name in ('bt_navigator', 'planner_server', 'controller_server', 'waypoint_follower')]
    action = ActionServer(fake, NavigateToPose, '/navigate_to_pose', execute,
                          cancel_callback=lambda request: CancelResponse.ACCEPT, callback_group=group)
    route_action = ActionServer(fake, FollowWaypoints, '/follow_waypoints', execute_route,
                                cancel_callback=lambda request: CancelResponse.ACCEPT, callback_group=group)
    odom_pub = fake.create_publisher(Odometry, '/odometry/global', 10)
    image_pub = fake.create_publisher(Image, '/camera/camera/color/image_raw', 1)
    image = Image()
    image.width, image.height, image.step = 424, 240, 424 * 3
    image.encoding = 'rgb8'
    image.data = bytes([220, 40, 10]) * (424 * 240)

    def publish_pose():
        message = Odometry()
        message.header.frame_id = 'map'
        message.header.stamp = fake.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.0
        message.pose.pose.position.x, message.pose.pose.position.y = pose_position
        odom_pub.publish(message)
        image.header.stamp = message.header.stamp
        image_pub.publish(image)

    timer = fake.create_timer(0.1, publish_pose, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    executor.add_node(ui)
    executor.add_node(ground_filter)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    port = ui.http.server_port
    external = None

    def request(method, route, payload=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
        body = json.dumps(payload) if payload is not None else None
        connection.request(method, route, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        code = response.status
        connection.close()
        return code, data

    def until(predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        raise AssertionError('Timed out waiting for the ROS UI bridge')

    def state():
        code, data = request('GET', '/api/state')
        assert code == 200
        return json.loads(data)

    try:
        until(lambda: state().get('projection') and state()['components'].get('bt_navigator', {}).get('state') == 'active')
        assert b'BOXBOT UI' in request('GET', '/')[1]
        token = state()['csrf']
        headers = {'X-Boxbot-Token': token, 'Content-Type': 'application/json'}
        assert request('POST', '/api/recovery', {'max_attempts': True}, headers)[0] == 400
        assert request('POST', '/api/recovery', {'max_attempts': 0}, headers)[0] == 200
        until(lambda: state()['traction']['recovery']['max_attempts'] == 0)
        assert request('POST', '/api/recovery', {'max_attempts': 2}, headers)[0] == 200
        assert request('POST', '/api/recovery', {'enabled': True})[0] == 403
        assert request('POST', '/api/recovery', {'enabled': 1}, headers)[0] == 400
        assert request('POST', '/api/recovery', {'enabled': True}, headers)[0] == 200
        until(lambda: state()['traction']['recovery']['enabled'] is True)
        assert request('POST', '/api/recovery', {'enabled': False}, headers)[0] == 200
        until(lambda: state()['traction']['recovery']['enabled'] is False)
        target = {'latitude': -27 + 5 / 111320,
                  'longitude': 153 + 2 / (111320 * math.cos(math.radians(-27))), 'heading': 0}
        assert request('POST', '/api/goal', target)[0] == 403
        assert request('POST', '/api/goal', target, dict(headers, Origin='http://foreign.example'))[0] == 403
        assert request('GET', '/api/state', headers={'Host': 'foreign.example'})[0] == 403
        assert request('GET', '/../package.xml')[0] == 404
        assert not received
        assert request('POST', '/api/goal', target, headers)[0] == 202
        until(lambda: len(received) == 1)
        pose = received[0]
        assert pose.header.frame_id == 'map'
        assert pose.pose.position.x == pytest.approx(c * 2 + s * 5, abs=1e-6)
        assert pose.pose.position.y == pytest.approx(-s * 2 + c * 5, abs=1e-6)
        heading = 2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        assert heading == pytest.approx(math.radians(65), abs=1e-5)
        assert request('POST', '/api/goal', target, headers)[0] == 409
        assert request('POST', '/api/cancel', {}, headers)[0] == 202
        until(lambda: len(canceled) == 1 and state()['navigation']['state'] == 'Canceled')
        # The UI cancel service also reaches goals submitted by another client.
        external = ActionClient(fake, NavigateToPose, '/navigate_to_pose')
        assert external.wait_for_server(timeout_sec=3)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        future = external.send_goal_async(goal)
        until(future.done)
        until(lambda: len(received) == 2)
        assert request('POST', '/api/cancel', {}, headers)[0] == 202
        until(lambda: len(canceled) == 2)
        until(lambda: not state()['navigation']['busy'])
        # Canceling while coordinate conversion is pending must prevent a late send.
        delay_conversion.set()
        assert request('POST', '/api/goal', target, headers)[0] == 202
        assert request('POST', '/api/cancel', {}, headers)[0] == 202
        time.sleep(0.7)
        assert len(received) == 2
        delay_conversion.clear()
        route = {'waypoints': [target, dict(target, latitude=target['latitude'] + 5 / 111320)]}
        assert request('POST', '/api/waypoints', {'waypoints': []}, headers)[0] == 400
        assert request('POST', '/api/waypoints', route, headers)[0] == 202
        until(lambda: len(routes) == 1 and state()['navigation'].get('current_waypoint') == 1)
        assert len(routes[0].poses) == 2
        assert routes[0].poses[0].pose.position.x == pytest.approx(c * 2 + s * 5, abs=1e-6)
        assert routes[0].poses[1].pose.position.y == pytest.approx(-s * 2 + c * 10, abs=1e-6)
        assert routes[0].number_of_loops == 0
        assert request('POST', '/api/goal', target, headers)[0] == 409
        assert request('POST', '/api/cancel', {}, headers)[0] == 202
        until(lambda: len(route_cancels) == 1 and not state()['navigation']['busy'])
        # Successful and failed routes retain their final outcome.
        assert request('POST', '/api/waypoints', route, headers)[0] == 202
        until(lambda: len(routes) == 2)
        finish_route.set()
        until(lambda: state()['navigation']['state'] == 'Arrived' and not state()['navigation']['busy'])
        route_failure.set()
        assert request('POST', '/api/waypoints', route, headers)[0] == 202
        until(lambda: len(routes) == 3 and state()['navigation']['state'] == 'Waypoint 1 failed: Costmap timed out waiting for update'
              and not state()['navigation']['busy'])
        finish_route.clear()
        # Routes exceeding the service queue depth must convert without dropped
        # requests. Total conversion time can exceed the per-request deadline.
        for count, delay in [(8, 0.03), (50, 0.09)]:
            conversion_delay[0] = delay
            before = conversion_count[0]
            expected_routes = len(routes) + 1
            expected_cancels = len(route_cancels) + 1
            large_route = {'waypoints': [dict(target, latitude=target['latitude'] + i / 111320)
                                         for i in range(count)]}
            assert request('POST', '/api/waypoints', large_route, headers)[0] == 202
            until(lambda: len(routes) == expected_routes, seconds=25)
            assert len(routes[-1].poses) == count
            assert conversion_count[0] - before == 2 * count
            assert conversion_active[1] == 1
            assert request('POST', '/api/cancel', {}, headers)[0] == 202
            until(lambda: len(route_cancels) == expected_cancels and not state()['navigation']['busy'])
        conversion_delay[0] = 0.0
        # Apply the slider through real ROS parameter services and report only
        # confirmed values. Rejection must preserve the previous cutoff.
        until(lambda: state().get('ground_filter', {}).get('age', 99) < 6)
        assert request('POST', '/api/filter', {'min_obstacle_height': 0.25}, headers)[0] == 202
        until(lambda: state()['ground_filter'].get('min_obstacle_height') == 0.25
              and not state()['ground_filter']['pending'])
        assert ground_filter.get_parameter('min_obstacle_height').value == 0.25
        ground_filter.add_on_set_parameters_callback(lambda parameters: SetParametersResult(
            successful=False, reason='Test filter rejection'))
        assert request('POST', '/api/filter', {'min_obstacle_height': 0.4}, headers)[0] == 202
        until(lambda: state()['ground_filter'].get('error') == 'Test filter rejection')
        assert state()['ground_filter']['min_obstacle_height'] == 0.25
        assert ground_filter.get_parameter('min_obstacle_height').value == 0.25
        # A 200 m target is accepted. A leg beyond 300 m is rejected after conversion.
        far_target = dict(target, latitude=-27 + 200 / 111320)
        assert request('POST', '/api/goal', far_target, headers)[0] == 202
        until(lambda: len(received) == 3)
        assert request('POST', '/api/cancel', {}, headers)[0] == 202
        until(lambda: len(canceled) == 3 and not state()['navigation']['busy'])
        far_target['latitude'] = -27 + 400 / 111320
        assert request('POST', '/api/goal', far_target, headers)[0] == 202
        until(lambda: '300 m' in state()['navigation']['state'])
        assert len(received) == 3
        # Track survives browser requests and filters sub-threshold movement.
        until(lambda: len(state().get('trail', [])) >= 1)
        initial_count = len(state()['trail'])
        pose_position[0] = 1.0
        until(lambda: len(state()['trail']) > initial_count)
        assert state()['trail'][-1][:2] == [1.0, 0.0]
        assert request('POST', '/api/clear_trail', {}, headers)[0] == 200
        until(lambda: len(state()['trail']) <= 1)
        # Non-finite optional battery fields become valid JSON null values.
        battery = BatteryState()
        battery.voltage, battery.current, battery.percentage = 39.8, -0.2, 0.92
        battery.temperature = math.nan
        ui.battery_state(battery)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 50
        grid.info.resolution = 0.1
        grid.info.origin.orientation.w = 1.0
        grid.data = [100] * 2500
        ui.costmap(grid)
        until(lambda: state().get('battery', {}).get('voltage') is not None)
        snapshot = state()
        assert snapshot['battery']['temperature'] is None
        assert 0 < len(snapshot['obstacles']) <= 1600
        # A delayed exact-stamp transform must not discard the obstacle grid.
        delayed = OccupancyGrid()
        delayed.header.frame_id = 'test_odom'
        delayed.header.stamp = fake.get_clock().now().to_msg()
        delayed.info.width = delayed.info.height = 1
        delayed.info.resolution = 0.1
        delayed.info.origin.orientation.w = 1.0
        delayed.data = [100]
        ui.costmap(delayed)
        time.sleep(0.6)
        assert len(state()['obstacles']) > 1
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'test_odom'
        transform.header.stamp = delayed.header.stamp
        transform.transform.translation.x = 10.0
        transform.transform.rotation.w = 1.0
        ui.tf_buffer.set_transform(transform, 'test')
        until(lambda: len(state()['obstacles']) == 1)
        assert state()['obstacles'][0] == pytest.approx([10.05, 0.05, 100])
        # Preview demand creates the subscription. Frames are resized, correctly
        # converted from RGB to JPEG/BGR, and expire when the viewer goes away.
        assert ui.preview_subscription is None
        assert request('GET', '/api/camera.jpg')[0] == 503
        until(lambda: request('GET', '/api/camera.jpg')[0] == 200)
        code, jpeg = request('GET', '/api/camera.jpg')
        decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape[:2] == (181, 320)
        assert decoded[50, 50, 2] > 200
        assert decoded[50, 50, 0] < 30
        until(lambda: ui.preview_subscription is None, seconds=5)
        encoded_at = ui.preview_encoded
        time.sleep(2.1)
        assert ui.preview_encoded == encoded_at
        assert request('GET', '/api/camera.jpg')[0] == 503
        until(lambda: request('GET', '/api/camera.jpg')[0] == 200)
        # Independent ROS measurements reach the UI and its tuning recording.
        command_pub = fake.create_publisher(TwistStamped, '/cmd_vel', 10)
        wheel_pub = fake.create_publisher(Odometry, '/hoverboard_base_controller/odom', 10)
        imu_pub = fake.create_publisher(Imu, '/imu/data', 10)
        gps_pub = fake.create_publisher(UBXNavPVT, '/gps_rover/navpvt', 10)
        traction_mode = ['turn']

        def publish_traction():
            stamp = fake.get_clock().now().to_msg()
            command = TwistStamped()
            command.header.stamp = stamp
            command.twist.angular.z = .4 if traction_mode[0] == 'turn' else 0.
            command.twist.linear.x = .2 if traction_mode[0] == 'forward' else 0.
            wheel = Odometry()
            wheel.header.stamp = stamp
            wheel.twist.twist = command.twist
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'base_footprint'
            imu.orientation.w = 1.
            gps = UBXNavPVT()
            utc = datetime.now(timezone.utc)
            gps.year, gps.month, gps.day = utc.year, utc.month, utc.day
            gps.hour, gps.min, gps.sec = utc.hour, utc.minute, utc.second
            gps.nano = utc.microsecond * 1000
            gps.valid_date = gps.valid_time = gps.gnss_fix_ok = True
            gps.gps_fix.fix_type = 3
            gps.num_sv, gps.s_acc = 20, 10
            command_pub.publish(command)
            wheel_pub.publish(wheel)
            imu_pub.publish(imu)
            gps_pub.publish(gps)

        motion_counts = len(received), len(routes)
        traction_timer = fake.create_timer(.1, publish_traction, callback_group=group)
        until(lambda: state()['traction']['state'] == 'suspected_turn_slip')
        log_path = Path(state()['traction']['log_path'])

        def recorded_slip():
            with log_path.open() as recording:
                return any(row['state'] == 'suspected_turn_slip' for row in csv.DictReader(recording))

        until(recorded_slip)
        traction_mode[0] = 'forward'
        until(lambda: state()['traction']['state'] == 'suspected_forward_slip')
        assert state()['traction']['metrics']['gps_usable']
        assert (len(received), len(routes)) == motion_counts
        traction_mode[0] = 'idle'
        until(lambda: state()['traction']['state'] == 'idle')
        traction_timer.cancel()
        # A stale localization stream prevents accepting another target.
        timer.cancel()
        time.sleep(2.2)
        assert request('POST', '/api/goal', target, headers)[0] == 503
    finally:
        stop.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        ui.close()
        if external is not None:
            external.destroy()
        action.destroy()
        route_action.destroy()
        fake.destroy_node()
        ui.destroy_node()
        ground_filter.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('payload', [None, {}, {'waypoints': []}, {'waypoints': [None]},
                                   {'waypoints': [{}] * 51}])
def test_invalid_waypoint_routes(payload):
    with pytest.raises(ValueError):
        validate_waypoints(payload)


@pytest.mark.parametrize('payload', [None, {}, {'min_obstacle_height': True},
                                   {'min_obstacle_height': -0.1}, {'min_obstacle_height': 0.51},
                                   {'min_obstacle_height': math.nan}])
def test_invalid_ground_filter_settings(payload):
    with pytest.raises(ValueError):
        validate_filter(payload)


def test_terminal_status_unlocks_lost_result_and_late_result_cannot_clear_new_goal():
    from concurrent.futures import Future
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from unique_identifier_msgs.msg import UUID

    rclpy.init(domain_id=176, args=['--ros-args', '-p', 'port:=0', '-p', 'host:=127.0.0.1'])
    ui = BoxbotUI()
    try:
        goal = SimpleNamespace(goal_id=UUID(uuid=[7]*16))
        ui.goal_handle = goal
        ui.goal_result_future = Future()
        ui.nav.update(active=True, state='Navigating')
        status = GoalStatus()
        status.goal_info.goal_id = goal.goal_id
        status.status = 5
        ui.action_status(GoalStatusArray(status_list=[status]), 'follow_waypoints')
        ui.terminal_goal = (goal, 5, time.monotonic()-2)
        ui.reconcile_goal()
        assert ui.goal_handle is None
        assert ui.nav['active'] is False
        assert ui.nav['state'] == 'Canceled'
        newer = SimpleNamespace(goal_id=UUID(uuid=[8]*16))
        ui.goal_handle = newer
        ui.nav.update(active=True, state='New mission')
        ui.goal_finished(Future(), goal)
        assert ui.goal_handle is newer
        assert ui.nav['state'] == 'New mission'
    finally:
        ui.close()
        ui.destroy_node()
        rclpy.shutdown()
