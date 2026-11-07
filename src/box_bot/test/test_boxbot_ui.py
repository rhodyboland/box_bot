"""HTTP and ROS navigation integration on an isolated domain with a fake robot."""
import http.client
import json
import math
from pathlib import Path
import sys
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import BatteryState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from boxbot_ui import BoxbotUI
from ui_http import validate_goal


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
    group = ReentrantCallbackGroup()
    received, canceled = [], []
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
        if delay_conversion.is_set():
            time.sleep(0.4)
        north = (request.ll_point.latitude + 27) * 111320
        east = (request.ll_point.longitude - 153) * 111320 * math.cos(math.radians(-27))
        response.map_point.x, response.map_point.y = c * east + s * north, -s * east + c * north
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

    services = [fake.create_service(ToLL, '/toLL', to_ll, callback_group=group),
                fake.create_service(FromLL, '/fromLL', from_ll, callback_group=group)]
    services += [fake.create_service(GetState, f'/{name}/get_state', lifecycle, callback_group=group)
                 for name in ('bt_navigator', 'planner_server', 'controller_server')]
    action = ActionServer(fake, NavigateToPose, '/navigate_to_pose', execute,
                          cancel_callback=lambda request: CancelResponse.ACCEPT, callback_group=group)
    odom_pub = fake.create_publisher(Odometry, '/odometry/global', 10)

    def publish_pose():
        message = Odometry()
        message.header.frame_id = 'map'
        message.header.stamp = fake.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.0
        odom_pub.publish(message)

    timer = fake.create_timer(0.1, publish_pose, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    executor.add_node(ui)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    port = ui.http.server_port

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
        external.destroy()
        until(lambda: not state()['navigation']['busy'])
        # Canceling while coordinate conversion is pending must prevent a late send.
        delay_conversion.set()
        assert request('POST', '/api/goal', target, headers)[0] == 202
        assert request('POST', '/api/cancel', {}, headers)[0] == 202
        time.sleep(0.7)
        assert len(received) == 2
        delay_conversion.clear()
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
        # A stale localization stream prevents accepting another target.
        timer.cancel()
        time.sleep(2.2)
        assert request('POST', '/api/goal', target, headers)[0] == 503
    finally:
        stop.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        ui.close()
        action.destroy()
        fake.destroy_node()
        ui.destroy_node()
        rclpy.shutdown()
