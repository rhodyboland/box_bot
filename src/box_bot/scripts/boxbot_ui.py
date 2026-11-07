#!/usr/bin/env python3
"""BOXBOT UI ROS bridge and local web server."""
import math
import os
from pathlib import Path
from datetime import datetime, timezone
import queue
import threading
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from ublox_ubx_msgs.msg import UBXMonComms, UBXNavPVT, UBXNavRelPosNED

from gnss_helpers import baseline_length, relpos_flags
from imu_tilt_cloud_filter import _roll_pitch_from_quat
from ui_http import UiHttpServer


def finite(value):
    return float(value) if math.isfinite(value) else None


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class BoxbotUI(Node):
    def __init__(self):
        super().__init__('boxbot_ui')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('from_ll_service', '/fromLL')
        self.declare_parameter('to_ll_service', '/toLL')
        self.declare_parameter('max_goal_distance', 75.0)
        self.components = {}
        self.pose = None
        self.plan = []
        self.controller_plan = []
        self.obstacles = []
        self.resolution = 0.1
        self.projection = None
        self.projection_pending = None
        self.projection_updated = 0.0
        self.last_map = 0.0
        self.battery = {}
        self.nav = {'state': 'Idle', 'active': False}
        self.pending_goal = None
        self.goal_handle = None
        self.external_active = False
        self.events = []
        self.lifecycle = {}
        self.polls = {}
        self.subscriptions_ = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.from_ll = self.create_client(FromLL, self.get_parameter('from_ll_service').value)
        self.to_ll = self.create_client(ToLL, self.get_parameter('to_ll_service').value)
        self.controller_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        names = ['controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
                 'smoother_server', 'waypoint_follower', 'velocity_smoother']
        self.lifecycle_clients = {name: self.create_client(GetState, f'/{name}/get_state') for name in names}
        self.subscribe(Odometry, '/odometry/global', self.global_odom)
        self.subscribe(Odometry, '/odometry/local', lambda m: self.touch('localization_local', 'Local odometry'))
        self.subscribe(Odometry, '/hoverboard_base_controller/odom', self.wheel_odom)
        self.subscribe(NavSatFix, '/gps_rover/fix', lambda m: self.gps_fix(m, 'gps_rover'))
        self.subscribe(NavSatFix, '/gps_moving_base/fix', lambda m: self.gps_fix(m, 'gps_base'))
        self.subscribe(UBXNavPVT, '/gps_rover/navpvt', lambda m: self.gps_pvt(m, 'gps_rover'))
        self.subscribe(UBXNavPVT, '/gps_moving_base/navpvt', lambda m: self.gps_pvt(m, 'gps_base'))
        self.subscribe(UBXMonComms, '/gps_rover/ubx_mon_comms', lambda m: self.gps_comms(m, 'gps_rover'))
        self.subscribe(UBXMonComms, '/gps_moving_base/ubx_mon_comms', lambda m: self.gps_comms(m, 'gps_base'))
        self.subscribe(UBXNavRelPosNED, '/gps/relposned', self.heading)
        self.subscribe(Imu, '/imu/data', self.imu)
        self.subscribe(PointCloud2, '/camera/depth/obstacles', self.camera)
        self.subscribe(BatteryState, '/battery/state', self.battery_state)
        self.subscribe(DiagnosticArray, '/diagnostics', self.diagnostics)
        self.subscribe(TwistStamped, '/cmd_vel', lambda m: self.touch(
            'velocity', 'Commanded motion', vx=finite(m.twist.linear.x), wz=finite(m.twist.angular.z)))
        self.subscribe(RosPath, '/plan', lambda m: self.path(m, False))
        self.subscribe(RosPath, '/transformed_global_plan', lambda m: self.path(m, True))
        self.subscribe(GoalStatusArray, '/navigate_to_pose/_action/status', self.action_status)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(OccupancyGrid, '/local_costmap/costmap', self.costmap, qos)
        web_directory = Path(get_package_share_directory('box_bot')) / 'web'
        self.http = UiHttpServer((self.get_parameter('host').value, self.get_parameter('port').value), web_directory)
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.create_timer(0.1, self.controls)
        self.create_timer(0.5, self.snapshot)
        self.create_timer(2.0, self.poll_status)
        self.get_logger().info(f'BOXBOT UI listening on port {self.http.server_port}')

    def subscribe(self, message_type, topic, callback, qos=qos_profile_sensor_data):
        self.subscriptions_.append(self.create_subscription(message_type, topic, callback, qos))

    def touch(self, name, label, level='ok', **details):
        self.components[name] = dict(label=label, level=level, received=time.monotonic(), **details)

    def global_odom(self, msg):
        p = msg.pose.pose
        values = [p.position.x, p.position.y, yaw(p.orientation)]
        if not all(math.isfinite(value) for value in values):
            self.touch('localization', 'Global localization', 'error', message='Invalid pose')
            return
        if self.pose and time.monotonic() - self.pose['received'] > 3:
            self.projection = None
            self.projection_pending = None
            self.projection_updated = 0.0
        self.pose = dict(x=p.position.x, y=p.position.y, yaw=yaw(p.orientation), received=time.monotonic())
        self.touch('localization', 'Global localization', vx=finite(msg.twist.twist.linear.x))

    def wheel_odom(self, msg):
        self.touch('hoverboard', 'Hoverboard telemetry', vx=finite(msg.twist.twist.linear.x),
                   wz=finite(msg.twist.twist.angular.z))

    def gps_fix(self, msg, name):
        old = self.components.get(name, {})
        details = {key: old[key] for key in ('satellites', 'fix', 'epoch_utc', 'usb_usage', 'usb_pending') if key in old}
        variance = msg.position_covariance[0]
        accuracy = (math.sqrt(variance) if math.isfinite(variance) and variance >= 0 and
                    msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN else None)
        self.touch(name, 'Rover GNSS' if name == 'gps_rover' else 'Moving-base GNSS',
                   'ok' if msg.status.status >= 0 else 'error',
                   latitude=finite(msg.latitude), longitude=finite(msg.longitude),
                   accuracy=accuracy, **details)

    def gps_pvt(self, msg, name):
        old = self.components.get(name, {})
        details = {key: old[key] for key in ('latitude', 'longitude', 'accuracy', 'usb_usage', 'usb_pending') if key in old}
        fix = {0: 'GNSS', 1: 'RTK float', 2: 'RTK fixed'}.get(msg.carr_soln.status, 'GNSS')
        if msg.valid_date and msg.valid_time:
            try:
                epoch = datetime(msg.year, msg.month, msg.day, msg.hour, msg.min, msg.sec,
                                 tzinfo=timezone.utc).timestamp() + msg.nano * 1e-9
                details['epoch_utc'] = epoch
            except ValueError:
                pass
        self.touch(name, 'Rover GNSS' if name == 'gps_rover' else 'Moving-base GNSS',
                   'ok' if msg.gnss_fix_ok else 'error', satellites=msg.num_sv, fix=fix, **details)

    def gps_comms(self, msg, name):
        port = next((p for p in msg.ports if p.port_id == 0x0300), None)
        if port and name in self.components:
            self.components[name].update(usb_usage=port.tx_usage, usb_pending=port.tx_pending)

    def heading(self, msg):
        flags = relpos_flags(msg)
        self.touch('heading', 'Moving-baseline heading',
                   'ok' if flags & (1 << 8) and flags & (1 << 2) else 'warn',
                   baseline=baseline_length(msg), accuracy=msg.acc_heading * 1e-5)

    def imu(self, msg):
        roll, pitch = _roll_pitch_from_quat(msg.orientation)
        if not math.isfinite(roll) or not math.isfinite(pitch):
            return
        self.touch('imu', 'BNO085 IMU', roll=math.degrees(roll), pitch=math.degrees(pitch))

    def camera(self, msg):
        self.touch('realsense', 'RealSense + ground filter', points=msg.width * msg.height)

    def battery_state(self, msg):
        self.battery = dict(voltage=finite(msg.voltage), current=finite(msg.current),
                            percentage=finite(msg.percentage), temperature=finite(msg.temperature),
                            cells=[finite(v) for v in msg.cell_voltage])
        self.touch('battery', 'Battery BMS', **self.battery)

    def diagnostics(self, msg):
        for status in msg.status:
            if not status.name.startswith('boxbot_hub/'):
                continue
            level = status.level[0] if isinstance(status.level, bytes) else status.level
            self.touch(status.name, status.name.replace('boxbot_hub/', 'ESP32 '),
                       ['ok', 'warn', 'error', 'stale'][min(3, level)], message=status.message)

    def map_points(self, points, frame, stamp):
        if frame == 'map':
            return points
        try:
            transform = self.tf_buffer.lookup_transform('map', frame, Time.from_msg(stamp)).transform
        except TransformException:
            self.touch('transforms', 'Frame transforms', 'warn', message=f'Waiting for map to {frame}')
            return []
        self.touch('transforms', 'Frame transforms', message=f'map to {frame}')
        angle = yaw(transform.rotation)
        c, s = math.cos(angle), math.sin(angle)
        return [[transform.translation.x + c * p[0] - s * p[1],
                 transform.translation.y + s * p[0] + c * p[1], *p[2:]] for p in points]

    def path(self, msg, controller):
        step = max(1, math.ceil(len(msg.poses) / 400))
        selected = msg.poses[::step]
        if selected and selected[-1] is not msg.poses[-1]:
            selected.append(msg.poses[-1])
        points = [[p.pose.position.x, p.pose.position.y] for p in selected
                  if math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y)]
        points = self.map_points(points, msg.header.frame_id, msg.header.stamp)
        if controller:
            self.controller_plan = points
        else:
            self.plan = points

    def costmap(self, msg):
        if msg.info.width <= 0 or not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0:
            return
        if time.monotonic() - self.last_map < 0.45:
            return
        self.last_map = time.monotonic()
        occupied = [i for i, value in enumerate(msg.data) if value >= 65]
        step = max(1, math.ceil(len(occupied) / 1600))
        angle = yaw(msg.info.origin.orientation)
        c, s = math.cos(angle), math.sin(angle)
        points = []
        for index in occupied[::step]:
            x = (index % msg.info.width + 0.5) * msg.info.resolution
            y = (index // msg.info.width + 0.5) * msg.info.resolution
            points.append([msg.info.origin.position.x + c * x - s * y,
                           msg.info.origin.position.y + s * x + c * y, msg.data[index]])
        self.obstacles = self.map_points(points, msg.header.frame_id, msg.header.stamp)
        self.resolution = msg.info.resolution
        self.touch('costmap', 'Local obstacle map', occupied=len(occupied))

    def action_status(self, msg):
        self.external_active = any(s.status in (1, 2, 3) for s in msg.status_list)
        if msg.status_list and not self.goal_handle and not self.pending_goal:
            latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
            state = {1: 'External navigation goal', 2: 'External navigation goal',
                     3: 'Canceling', 4: 'Arrived', 5: 'Canceled', 6: 'Navigation aborted'}
            self.nav.update(state=state.get(latest.status, 'Idle'), active=self.external_active)

    def poll_status(self):
        for name, client in self.lifecycle_clients.items():
            self.poll_request(name, client, GetState.Request(),
                              lambda f, key=name: self.lifecycle_done(f, key))
        self.poll_request('drive', self.controller_client, ListControllers.Request(), self.controllers_done)
        self.update_projection()

    def poll_request(self, name, client, request, callback):
        previous = self.polls.get(name)
        if previous and not previous[0].done():
            if time.monotonic() - previous[1] < 5:
                return
            client.remove_pending_request(previous[0])
            previous[0].cancel()
        if client.service_is_ready():
            future = client.call_async(request)
            self.polls[name] = future, time.monotonic()
            future.add_done_callback(callback)

    def lifecycle_done(self, future, name):
        try:
            state = future.result().current_state
            self.lifecycle[name] = state.label
            self.touch(name, name.replace('_', ' '), 'ok' if state.id == 3 else 'warn', state=state.label)
        except Exception as exc:
            self.get_logger().debug(str(exc))

    def controllers_done(self, future):
        try:
            controller = next((c for c in future.result().controller if c.name == 'hoverboard_base_controller'), None)
            state = controller.state if controller else 'missing'
            self.touch('drive_controller', 'Drive controller', 'ok' if state == 'active' else 'error', state=state)
        except Exception as exc:
            self.get_logger().debug(str(exc))

    def update_projection(self):
        if not self.pose or not self.to_ll.service_is_ready():
            return
        if self.projection_pending:
            futures, origin, started = self.projection_pending
            if time.monotonic() - started > 4:
                self.projection_pending = None
                return
            if not all(f.done() for f in futures):
                return
            self.projection_pending = None
            try:
                coordinates = [[f.result().ll_point.latitude, f.result().ll_point.longitude] for f in futures]
                if not all(math.isfinite(v) for point in coordinates for v in point):
                    return
                a, b, c = coordinates
                determinant = (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])
                if abs(determinant) < 1e-12:
                    return
                self.projection = dict(x=origin[0], y=origin[1], latitude=a[0], longitude=a[1],
                                       x_lat=(b[0]-a[0])/10, x_lon=(b[1]-a[1])/10,
                                       y_lat=(c[0]-a[0])/10, y_lon=(c[1]-a[1])/10)
                self.projection_updated = time.monotonic()
            except Exception as exc:
                self.get_logger().debug(str(exc))
            return
        if time.monotonic() - self.projection_updated < 20:
            return
        origin = [self.pose['x'], self.pose['y']]
        futures = []
        for dx, dy in [(0, 0), (10, 0), (0, 10)]:
            request = ToLL.Request()
            request.map_point.x, request.map_point.y = origin[0] + dx, origin[1] + dy
            futures.append(self.to_ll.call_async(request))
        self.projection_pending = futures, origin, time.monotonic()

    def event(self, message):
        self.events.append(dict(time=time.time(), message=message))
        self.events = self.events[-20:]

    def controls(self):
        try:
            command = self.http.commands.get_nowait()
        except queue.Empty:
            command = None
        if command:
            if command['deadline'] < time.monotonic():
                command['response'] = 503, {'error': 'Control request expired'}
            elif command['action'] == 'cancel':
                command['response'] = self.cancel()
            else:
                command['response'] = self.begin_goal(command['payload'])
            command['done'].set()
        if self.pending_goal and time.monotonic() - self.pending_goal['started'] > 8:
            self.pending_goal['cancel'] = True
            self.nav.update(state='Goal request timed out')
            self.pending_goal = None
            self.nav['active'] = False

    def begin_goal(self, target):
        if self.pending_goal or self.goal_handle or self.external_active:
            return 409, {'error': 'Cancel the current goal before sending another target'}
        pose_fresh = self.pose and time.monotonic() - self.pose['received'] < 2
        required = ('bt_navigator', 'planner_server', 'controller_server')
        ready = all(self.lifecycle.get(name) == 'active' and
                    time.monotonic() - self.components.get(name, {}).get('received', 0) < 4
                    for name in required)
        if not pose_fresh or not ready or self.projection is None:
            return 503, {'error': 'Navigation and localization must be active'}
        if not self.navigator.server_is_ready() or not self.from_ll.service_is_ready():
            return 503, {'error': 'Navigation or GPS coordinate conversion is unavailable'}
        # A second converted geographic point determines map yaw, including map rotation.
        bearing = math.radians(target['heading'])
        delta = 2 / 6378137.0
        latitude = math.radians(target['latitude'])
        if abs(math.cos(latitude)) < 0.01:
            return 400, {'error': 'Target latitude is outside the supported operating area'}
        coordinates = [(target['latitude'], target['longitude']),
                       (target['latitude'] + math.degrees(delta * math.cos(bearing)),
                        target['longitude'] + math.degrees(delta * math.sin(bearing) / math.cos(latitude)))]
        futures = []
        for lat, lon in coordinates:
            request = FromLL.Request()
            request.ll_point.latitude, request.ll_point.longitude = lat, lon
            futures.append(self.from_ll.call_async(request))
        pending = dict(phase='converting', target=target, futures=futures, cancel=False, started=time.monotonic())
        self.pending_goal = pending
        for future in futures:
            future.add_done_callback(lambda f: self.goal_converted(pending))
        self.nav = dict(state='Converting target', active=True, target=target)
        return 202, {'accepted': True}

    def goal_converted(self, pending):
        if self.pending_goal is not pending or pending['cancel'] or pending['phase'] != 'converting':
            return
        if not all(f.done() for f in pending['futures']):
            return
        try:
            position, direction = [f.result().map_point for f in pending['futures']]
            values = [position.x, position.y, direction.x, direction.y]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Invalid coordinate conversion')
            if math.hypot(direction.x-position.x, direction.y-position.y) < 0.1:
                raise ValueError('GPS map alignment is not initialized')
            distance = math.hypot(position.x-self.pose['x'], position.y-self.pose['y'])
            limit = self.get_parameter('max_goal_distance').value
            if distance > limit:
                raise ValueError(f'Target exceeds the {limit:g} m navigation window')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = position.x, position.y
            angle = math.atan2(direction.y-position.y, direction.x-position.x)
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            pending['phase'] = 'sending'
            self.nav['state'] = 'Sending goal'
            future = self.navigator.send_goal_async(goal, feedback_callback=self.feedback)
            future.add_done_callback(lambda f: self.goal_accepted(f, pending))
        except Exception as exc:
            self.pending_goal = None
            self.nav.update(state=str(exc), active=False)
            self.event(str(exc))

    def goal_accepted(self, future, pending):
        try:
            handle = future.result()
            if self.pending_goal is not pending:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.pending_goal = None
            if not handle.accepted:
                self.nav.update(state='Goal rejected', active=False)
                return
            self.goal_handle = handle
            self.nav.update(state='Navigating', active=True)
            self.event('Navigation goal accepted')
            handle.get_result_async().add_done_callback(self.goal_finished)
            if pending['cancel']:
                handle.cancel_goal_async()
        except Exception as exc:
            self.pending_goal = None
            self.nav.update(state=str(exc), active=False)

    def feedback(self, message):
        feedback = message.feedback
        self.nav.update(distance=finite(feedback.distance_remaining),
                        eta=feedback.estimated_time_remaining.sec,
                        recoveries=feedback.number_of_recoveries)

    def goal_finished(self, future):
        self.goal_handle = None
        try:
            result = future.result()
            self.nav.update(state={4: 'Arrived', 5: 'Canceled', 6: 'Navigation aborted'}.get(result.status, 'Finished'), active=False)
            self.event(self.nav['state'])
        except Exception as exc:
            self.nav.update(state=str(exc), active=False)

    def cancel(self):
        if self.pending_goal:
            self.pending_goal['cancel'] = True
            if self.pending_goal['phase'] == 'converting':
                self.pending_goal = None
        self.nav.update(state='Cancel requested')
        if not self.cancel_client.service_is_ready():
            self.nav.update(state='Cancel service unavailable')
            return 503, {'error': 'Navigation cancel service unavailable'}
        future = self.cancel_client.call_async(CancelGoal.Request())
        future.add_done_callback(self.canceled)
        self.event('Cancel requested')
        return 202, {'accepted': True}

    def canceled(self, future):
        try:
            response = future.result()
            self.nav.update(state='Canceling' if response.goals_canceling else 'No active goal')
            if not response.goals_canceling and not self.goal_handle:
                self.nav['active'] = False
        except Exception as exc:
            self.event(str(exc))

    def snapshot(self):
        now = time.monotonic()
        components = {}
        for name, item in self.components.items():
            age = now - item['received']
            components[name] = dict(item, age=round(age, 2))
            if 'epoch_utc' in item:
                components[name]['epoch_age'] = time.time() - item['epoch_utc']
                if components[name]['epoch_age'] > 1 and components[name]['level'] == 'ok':
                    components[name]['level'] = 'warn'
            if age > (6 if name.startswith(('gps', 'battery', 'boxbot_hub')) else 3):
                components[name]['level'] = 'stale'
        pose = dict(self.pose) if self.pose else None
        if pose:
            pose['age'] = now - pose.pop('received')
        try:
            memory = {}
            for line in Path('/proc/meminfo').read_text().splitlines():
                key, value = line.split(':', 1)
                if key in ('MemTotal', 'MemAvailable'):
                    memory[key] = int(value.split()[0]) / 1024
            system = dict(load=list(os.getloadavg()), memory=memory)
        except OSError:
            system = {}
        self.http.update(dict(time=time.time(), pose=pose, projection=self.projection,
                              plan=self.plan, controller_plan=self.controller_plan,
                              obstacles=self.obstacles, resolution=self.resolution,
                              components=components, battery=self.battery,
                              navigation=dict(self.nav, busy=bool(self.pending_goal or self.goal_handle or self.external_active)),
                              events=self.events, system=system))

    def close(self):
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(timeout=3)


def main():
    rclpy.init()
    node = BoxbotUI()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
