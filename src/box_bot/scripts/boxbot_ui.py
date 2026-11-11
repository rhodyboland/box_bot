#!/usr/bin/env python3
"""BOXBOT UI ROS bridge and local web server."""
import math
import csv
from collections import deque
import os
from pathlib import Path
from datetime import datetime, timezone
import queue
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import BatteryState, Image, Imu, NavSatFix, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from ublox_ubx_msgs.msg import UBXMonComms, UBXNavPVT, UBXNavRelPosNED

from gnss_helpers import baseline_length, relpos_flags
from imu_tilt_cloud_filter import _roll_pitch_from_quat, _quat_to_matrix
from traction_monitor import TractionMonitor
from traction_recovery import TractionRecovery, corridor_clear
from ui_http import UiHttpServer
from ui_tuning import NavigationTuning


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
        self.declare_parameter('max_goal_distance', 300.0)
        self.declare_parameter('ground_filter_node', '/imu_tilt_cloud_filter')
        self.declare_parameter('preview_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('traction_recovery_max_attempts', 2, ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=20, step=1)]))
        self.declare_parameter('tuning_config_file', str(Path(__file__).resolve().parents[1]/'config/nav2_minimal.yaml'))
        self.declare_parameter('traction_recovery_enabled', False)
        self.declare_parameter('traction_recovery_delay_seconds', 2.0, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=2.0, to_value=10.0, step=0.0)]))
        self.recovery_grid = None
        self.camera_observation_stamp = None
        self.declare_parameter('traction_dwell_seconds', 2.0)
        self.declare_parameter('traction_log_path', '')
        self.traction = TractionMonitor(dwell=self.get_parameter('traction_dwell_seconds').value)
        self.traction_state = None
        self.traction_log = None
        self.traction_log_updated = 0.0
        log_name = f'boxbot-traction-{datetime.now().strftime("%Y%m%d-%H%M%S")}-{os.getpid()}.csv'
        self.traction_log_path = self.get_parameter('traction_log_path').value or str(
            Path(os.environ.get('ROS_LOG_DIR', str(Path.home() / '.ros/log'))) / log_name)
        try:
            Path(self.traction_log_path).parent.mkdir(parents=True, exist_ok=True)
            self.traction_log = open(self.traction_log_path, 'w', newline='')
            self.traction_writer = csv.DictWriter(self.traction_log, fieldnames=[
                'time_utc', 'state', 'duration', 'command_vx', 'command_wz', 'wheel_vx',
                'wheel_wz', 'gyro_wz', 'gps_speed', 'gps_accuracy', 'wheel_usable', 'gyro_usable', 'gps_usable'])
            self.traction_writer.writeheader()
        except OSError as exc:
            self.get_logger().warning(f'Traction recording unavailable: {exc}')
        self.preview_subscription = None
        self.preview_bridge = CvBridge()
        self.preview_encoded = 0.0
        self.components = {}
        self.pose = None
        self.plan = []
        self.controller_plan = []
        self.obstacles = []
        self.pending_maps = deque(maxlen=10)
        self.trail = deque(maxlen=10000)
        self.trail_sampled = 0.0
        self.resolution = 0.1
        self.projection = None
        self.projection_pending = None
        self.projection_updated = 0.0
        self.battery = {}
        self.filter_settings = {}
        self.filter_pending = None
        self.filter_revision = 0
        self.nav = {'state': 'Idle', 'active': False}
        self.pending_goal = None
        self.goal_handle = None
        self.goal_result_future = None
        self.terminal_goal = None
        self.cancel_requested_at = None
        self.external_active = False
        self.external_actions = {}
        self.events = []
        self.lifecycle = {}
        self.polls = {}
        self.subscriptions_ = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.waypoint_navigator = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.cancel_clients = [self.create_client(CancelGoal, f'/{name}/_action/cancel_goal')
                               for name in ('follow_waypoints', 'navigate_to_pose')]
        self.from_ll = self.create_client(FromLL, self.get_parameter('from_ll_service').value)
        self.to_ll = self.create_client(ToLL, self.get_parameter('to_ll_service').value)
        filter_node = self.get_parameter('ground_filter_node').value.rstrip('/')
        self.filter_get = self.create_client(GetParameters, filter_node + '/get_parameters')
        self.filter_set = self.create_client(SetParametersAtomically, filter_node + '/set_parameters_atomically')
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
        self.subscribe(TwistStamped, '/cmd_vel', self.commanded_motion)
        self.subscribe(TwistStamped, '/cmd_vel_nav', self.navigation_motion)
        self.subscribe(RosPath, '/plan', lambda m: self.path(m, False))
        self.subscribe(RosPath, '/transformed_global_plan', lambda m: self.path(m, True))
        for name in ('navigate_to_pose', 'follow_waypoints'):
            self.subscribe(GoalStatusArray, f'/{name}/_action/status',
                           lambda m, action=name: self.action_status(m, action))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(OccupancyGrid, '/local_costmap/costmap', self.costmap, qos)
        web_directory = Path(get_package_share_directory('box_bot')) / 'web'
        self.http = UiHttpServer((self.get_parameter('host').value, self.get_parameter('port').value), web_directory)
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.recovery = TractionRecovery(self, self.recovery_clearance, self.cancel)
        self.subscribe(GoalStatusArray, '/follow_path/_action/status', self.recovery.follow_path_status)
        self.tuning = NavigationTuning(self, self.tuning_gate, self.get_parameter('tuning_config_file').value)
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
        now = time.monotonic()
        gap = self.pose and now - self.pose['received'] > 3
        if gap:
            self.projection = None
            self.projection_pending = None
            self.projection_updated = 0.0
        if now - self.trail_sampled >= 0.5:
            point = [p.position.x, p.position.y]
            previous = self.trail[-1] if self.trail else None
            if gap or previous is None or math.dist(point, previous[:2]) >= 0.15:
                self.trail.append([*point, bool(gap)])
                self.trail_sampled = now
        self.pose = dict(x=p.position.x, y=p.position.y, yaw=yaw(p.orientation), received=now)
        self.touch('localization', 'Global localization', vx=finite(msg.twist.twist.linear.x))

    def traction_stamp(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return None
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        return time.monotonic() - age

    def navigation_motion(self, msg):
        self.touch('navigation_velocity', 'Nav2 command', vx=finite(msg.twist.linear.x))

    def commanded_motion(self, msg):
        self.touch('velocity', 'Commanded motion', vx=finite(msg.twist.linear.x), wz=finite(msg.twist.angular.z))
        self.traction.update('command', (msg.twist.linear.x, msg.twist.angular.z),
                             self.traction_stamp(msg), time.monotonic())

    def wheel_odom(self, msg):
        self.traction.update('wheel', (msg.twist.twist.linear.x, msg.twist.twist.angular.z),
                             self.traction_stamp(msg), time.monotonic())
        self.touch('hoverboard', 'Hoverboard telemetry', vx=finite(msg.twist.twist.linear.x),
                   wz=finite(msg.twist.twist.angular.z), stamp=self.traction_stamp(msg))

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
        epoch = None
        if msg.valid_date and msg.valid_time:
            try:
                epoch = datetime(msg.year, msg.month, msg.day, msg.hour, msg.min, msg.sec,
                                 tzinfo=timezone.utc).timestamp() + msg.nano * 1e-9
                details['epoch_utc'] = epoch
            except ValueError:
                pass
        self.touch(name, 'Rover GNSS' if name == 'gps_rover' else 'Moving-base GNSS',
                   'ok' if msg.gnss_fix_ok else 'error', satellites=msg.num_sv, fix=fix, **details)
        if name == 'gps_rover':
            accuracy = msg.s_acc * 1e-3
            valid = (msg.gnss_fix_ok and msg.gps_fix.fix_type == 3 and not msg.invalid_llh and
                     msg.num_sv >= 6 and 0 < accuracy <= 0.10 and epoch is not None)
            stamp = time.monotonic() - (time.time() - epoch) if valid else None
            self.traction.update('gps', (math.hypot(msg.vel_n, msg.vel_e) * 1e-3, accuracy),
                                 stamp, time.monotonic())

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
        rate = math.nan
        if msg.angular_velocity_covariance[0] != -1:
            try:
                rotation = self.tf_buffer.lookup_transform('base_footprint', msg.header.frame_id, Time()).transform.rotation
                matrix = _quat_to_matrix(rotation)
                vector = msg.angular_velocity
                rate = matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z
            except TransformException:
                pass
        self.traction.update('gyro', (rate,), self.traction_stamp(msg), time.monotonic())
        roll, pitch = _roll_pitch_from_quat(msg.orientation)
        if not math.isfinite(roll) or not math.isfinite(pitch):
            return
        self.touch('imu', 'BNO085 IMU', roll=math.degrees(roll), pitch=math.degrees(pitch))

    def camera(self, msg):
        self.camera_observation_stamp = self.traction_stamp(msg)
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
        self.pending_maps.append(msg)
        self.recovery_grid = msg

    def recovery_clearance(self):
        grid = self.recovery_grid
        camera = self.components.get('realsense', {})
        if (grid is None or self.camera_observation_stamp is None or
                not -.1 <= time.monotonic()-self.camera_observation_stamp <= 1.0 or
                not grid.header.stamp.sec or
                (self.get_clock().now()-Time.from_msg(grid.header.stamp)).nanoseconds > 1e9 or
                time.monotonic()-camera.get('received', 0) > 1.0 or
                self.lifecycle.get('behavior_server') != 'active' or
                time.monotonic()-self.components.get('behavior_server', {}).get('received', 0) > 4):
            return []
        try:
            transform = self.tf_buffer.lookup_transform(grid.header.frame_id, 'base_footprint', Time()).transform
        except TransformException:
            return []
        pose = (transform.translation.x, transform.translation.y, yaw(transform.rotation))
        return [direction for direction in (-1, 1) if corridor_clear(grid, pose, direction)]

    def update_costmap(self):
        # Costmap stamps often arrive before the corresponding localization TF.
        # Retain a bounded backlog and render the newest grid with an exact TF.
        for msg in reversed(self.pending_maps):
            if msg.header.frame_id == 'map' or self.tf_buffer.can_transform(
                    'map', msg.header.frame_id, Time.from_msg(msg.header.stamp)):
                break
        else:
            return
        while self.pending_maps.popleft() is not msg:
            pass
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

    def action_status(self, msg, action='navigate_to_pose'):
        if self.goal_handle is not None:
            goal_id = bytes(self.goal_handle.goal_id.uuid)
            terminal = next((status for status in msg.status_list
                             if bytes(status.goal_info.goal_id.uuid) == goal_id and status.status in (4, 5, 6)), None)
            if terminal is not None and self.terminal_goal is None:
                self.terminal_goal = (self.goal_handle, terminal.status, time.monotonic())
        self.external_actions[action] = any(s.status in (1, 2, 3) for s in msg.status_list)
        self.external_active = any(self.external_actions.values())
        if msg.status_list and not self.goal_handle and not self.pending_goal:
            if not self.external_active and self.nav.get('mode') != 'external':
                return
            if action == 'navigate_to_pose' and self.external_actions.get('follow_waypoints'):
                return
            latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
            identity = (action, bytes(latest.goal_info.goal_id.uuid))
            if latest.status in (1, 2) and identity != getattr(self, 'external_mission_id', None):
                self.external_mission_id = identity
                if self.recovery.busy:
                    self.cancel()
                else:
                    self.recovery.new_mission()
            state = {1: 'External navigation goal', 2: 'External navigation goal',
                     3: 'Canceling', 4: 'Arrived', 5: 'Canceled', 6: 'Navigation aborted'}
            self.nav.update(state=state.get(latest.status, 'Idle'), active=self.external_active, mode='external')

    def poll_status(self):
        for name, client in self.lifecycle_clients.items():
            self.poll_request(name, client, GetState.Request(),
                              lambda f, key=name: self.lifecycle_done(f, key))
        self.poll_request('drive', self.controller_client, ListControllers.Request(), self.controllers_done)
        self.update_projection()
        if not self.filter_pending:
            request = GetParameters.Request()
            request.names = ['min_obstacle_height', 'ground_height', 'max_obstacle_height']
            self.poll_request('ground_filter', self.filter_get, request,
                              lambda f, revision=self.filter_revision: self.filter_read(f, revision))

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

    def filter_read(self, future, revision):
        if revision != self.filter_revision or self.filter_pending:
            return
        try:
            values = [parameter_value_to_python(value) for value in future.result().values]
            if len(values) != 3 or not all(isinstance(v, float) and math.isfinite(v) for v in values):
                raise ValueError('Ground filter parameters are unavailable')
            self.filter_settings.update(min_obstacle_height=values[0], ground_height=values[1],
                                        max_obstacle_height=values[2], received=time.monotonic())
        except Exception as exc:
            self.filter_settings['error'] = str(exc)

    def set_filter(self, settings):
        if self.filter_pending:
            return 409, {'error': 'A ground filter update is already pending'}
        if not self.filter_set.service_is_ready() or time.monotonic() - self.filter_settings.get('received', 0) > 6:
            return 503, {'error': 'Ground filter is unavailable. Wait for its live settings.'}
        height = settings['min_obstacle_height']
        if height >= self.filter_settings['max_obstacle_height']:
            return 400, {'error': 'Minimum height must be below the maximum obstacle height'}
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('min_obstacle_height', value=height).to_parameter_msg()]
        self.filter_revision += 1
        self.filter_settings.pop('error', None)
        self.filter_started = time.monotonic()
        self.filter_pending = self.filter_set.call_async(request)
        self.filter_pending.add_done_callback(lambda f: self.filter_applied(f, height))
        return 202, {'accepted': True}

    def filter_applied(self, future, height):
        if self.filter_pending is not future:
            return
        self.filter_pending = None
        try:
            result = future.result().result
            if not result.successful:
                raise ValueError(result.reason or 'Ground filter rejected the update')
            self.filter_settings.update(min_obstacle_height=height, received=time.monotonic())
            self.event(f'Minimum obstacle height set to {height * 100:g} cm')
        except Exception as exc:
            self.filter_settings['error'] = str(exc)
            self.event(str(exc))

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

    def update_preview_subscription(self):
        active = self.http.preview_active()
        if active and self.preview_subscription is None:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.preview_subscription = self.create_subscription(
                Image, self.get_parameter('preview_topic').value, self.preview_frame, qos)
        elif not active and self.preview_subscription is not None:
            self.destroy_subscription(self.preview_subscription)
            self.preview_subscription = None

    def preview_frame(self, msg):
        now = time.monotonic()
        if not self.http.preview_active() or now - self.preview_encoded < 0.5:
            return
        if msg.header.stamp.sec and (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds > 2e9:
            return
        self.preview_encoded = now
        try:
            frame = self.preview_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame.shape[1] > 320:
                height = max(1, round(frame.shape[0] * 320 / frame.shape[1]))
                frame = cv2.resize(frame, (320, height), interpolation=cv2.INTER_AREA)
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok:
                self.http.update_preview(jpeg.tobytes())
        except (CvBridgeError, cv2.error, ValueError) as exc:
            self.get_logger().debug(f'Camera preview unavailable: {exc}')

    def controls(self):
        self.tuning.tick()
        self.reconcile_goal()
        self.update_preview_subscription()
        try:
            command = self.http.commands.get_nowait()
        except queue.Empty:
            command = None
        if command:
            if command['deadline'] < time.monotonic():
                command['response'] = 503, {'error': 'Control request expired'}
            elif command['action'] == 'cancel':
                command['response'] = self.cancel()
            elif command['action'] == 'recovery':
                self.set_parameters([Parameter('traction_recovery_'+name, value=value)
                                     for name, value in command['payload'].items()])
                command['response'] = 200, {name: self.get_parameter('traction_recovery_'+name).value
                                            for name in ('enabled', 'max_attempts')}
            elif command['action'] == 'tuning':
                command['response'] = self.tuning.request(command['payload'])
            elif command['action'] == 'filter':
                command['response'] = self.set_filter(command['payload'])
            elif command['action'] == 'clear_trail':
                self.trail.clear()
                command['response'] = 200, {'cleared': True}
            else:
                command['response'] = self.begin_goal(
                    command['payload'], waypoints=command['action'] == 'waypoints')
            command['done'].set()
        if self.pending_goal and time.monotonic() - self.pending_goal['started'] > 8:
            self.pending_goal['cancel'] = True
            pending = self.pending_goal
            if pending['phase'] == 'converting':
                self.from_ll.remove_pending_request(pending['future'])
                pending['future'].cancel()
                message = f'Coordinate conversion timed out at waypoint {len(pending["points"]) // 2 + 1}'
            else:
                message = 'Navigation goal acknowledgement timed out'
            self.pending_goal = None
            self.nav.update(state=message, active=False)
            self.event(message)
            self.get_logger().error(message)
        if self.filter_pending and time.monotonic() - self.filter_started > 5:
            future = self.filter_pending
            self.filter_pending = None
            self.filter_set.remove_pending_request(future)
            future.cancel()
            self.filter_settings['error'] = 'No update acknowledgement. Refreshing live filter settings.'

    def tuning_gate(self):
        if self.pending_goal or self.goal_handle or self.external_active or self.recovery.busy:
            return 'Cancel navigation and recovery before trying tuning settings.'
        wheel = self.components.get('hoverboard', {})
        stamp = wheel.get('stamp')
        if stamp is None or not -.1 <= time.monotonic()-stamp <= 1:
            return 'Fresh wheel feedback is required before trying settings.'
        vx, wz = wheel.get('vx'), wheel.get('wz')
        if vx is None or wz is None or abs(vx) >= .02 or abs(wz) >= .05:
            return 'Stop the wheels before trying tuning settings.'
        return ''

    def begin_goal(self, target, waypoints=False):
        if self.tuning.busy or self.tuning.block_navigation:
            return 409, {'error': 'Wait for tuning to be confirmed or restored before navigating'}
        if self.pending_goal or self.goal_handle or self.external_active or self.recovery.busy:
            return 409, {'error': 'Cancel the current goal before sending another target'}
        pose_fresh = self.pose and time.monotonic() - self.pose['received'] < 2
        required = ['bt_navigator', 'planner_server', 'controller_server']
        if waypoints:
            required.append('waypoint_follower')
        ready = all(self.lifecycle.get(name) == 'active' and
                    time.monotonic() - self.components.get(name, {}).get('received', 0) < 4
                    for name in required)
        if not pose_fresh or not ready or self.projection is None:
            return 503, {'error': 'Navigation and localization must be active'}
        navigator = self.waypoint_navigator if waypoints else self.navigator
        if not navigator.server_is_ready() or not self.from_ll.service_is_ready():
            return 503, {'error': 'Navigation or GPS coordinate conversion is unavailable'}
        targets = target if waypoints else [target]
        coordinates = []
        for item in targets:
            # Convert a second point to obtain map yaw, including map rotation.
            bearing = math.radians(item['heading'])
            delta = 2 / 6378137.0
            latitude = math.radians(item['latitude'])
            if abs(math.cos(latitude)) < 0.01:
                return 400, {'error': 'Target latitude is outside the supported operating area'}
            coordinates.extend([(item['latitude'], item['longitude']),
                                (item['latitude'] + math.degrees(delta * math.cos(bearing)),
                                 item['longitude'] + math.degrees(delta * math.sin(bearing) / math.cos(latitude)))])
        pending = dict(phase='converting', coordinates=coordinates, points=[], waypoints=waypoints,
                       cancel=False, started=time.monotonic())
        self.pending_goal = pending
        self.nav = dict(state='Converting route' if waypoints else 'Converting target',
                        active=True, mode='waypoints' if waypoints else 'goal',
                        targets=targets, total_waypoints=len(targets), current_waypoint=0)
        self.request_conversion(pending)
        return 202, {'accepted': True}

    def request_conversion(self, pending):
        # FromLL uses a bounded service queue. Keep one request in flight so
        # routes of up to 50 poses cannot overflow it with a 100-request burst.
        latitude, longitude = pending['coordinates'][len(pending['points'])]
        request = FromLL.Request()
        request.ll_point.latitude, request.ll_point.longitude = latitude, longitude
        pending['started'] = time.monotonic()
        pending['future'] = self.from_ll.call_async(request)
        pending['future'].add_done_callback(lambda f: self.goal_converted(pending, f))

    def goal_converted(self, pending, future):
        if self.pending_goal is not pending or pending['cancel'] or pending['phase'] != 'converting':
            return
        try:
            pending['points'].append(future.result().map_point)
            completed = len(pending['points'])
            self.nav['state'] = f'Converting waypoint {(completed + 1) // 2} / {self.nav["total_waypoints"]}'
            if completed < len(pending['coordinates']):
                self.request_conversion(pending)
                return
            points = pending['points']
            if not self.pose or time.monotonic() - self.pose['received'] >= 2:
                raise ValueError('Localization became stale during coordinate conversion')
            poses = []
            origin = [self.pose['x'], self.pose['y']]
            limit = self.get_parameter('max_goal_distance').value
            for index in range(0, len(points), 2):
                position, direction = points[index:index+2]
                values = [position.x, position.y, direction.x, direction.y]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('Invalid coordinate conversion')
                if math.hypot(direction.x-position.x, direction.y-position.y) < 0.1:
                    raise ValueError('GPS map alignment is not initialized')
                if math.dist([position.x, position.y], origin) > limit:
                    raise ValueError(f'Target exceeds the {limit:g} m navigation window')
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y = position.x, position.y
                angle = math.atan2(direction.y-position.y, direction.x-position.x)
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
                poses.append(pose)
                # Later waypoints are planned after the previous one is reached.
                origin = [position.x, position.y]
            if pending['waypoints']:
                goal = FollowWaypoints.Goal()
                goal.poses = poses
                navigator = self.waypoint_navigator
                feedback = self.waypoint_feedback
            else:
                goal = NavigateToPose.Goal()
                goal.pose = poses[0]
                navigator = self.navigator
                feedback = self.feedback
            pending['phase'] = 'sending'
            pending['started'] = time.monotonic()
            self.nav['state'] = 'Sending goal'
            future = navigator.send_goal_async(goal, feedback_callback=feedback)
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
            self.recovery.new_mission()
            self.goal_handle = handle
            self.terminal_goal = None
            self.cancel_requested_at = None
            self.nav.update(state='Navigating', active=True)
            self.event('Navigation goal accepted')
            self.goal_result_future = handle.get_result_async()
            self.goal_result_future.add_done_callback(lambda future, goal=handle: self.goal_finished(future, goal))
            if pending['cancel']:
                handle.cancel_goal_async()
        except Exception as exc:
            self.pending_goal = None
            self.nav.update(state=str(exc), active=False)

    def waypoint_feedback(self, message):
        index = message.feedback.current_waypoint
        self.nav.update(current_waypoint=index,
                        state=f'Waypoint {index + 1} / {self.nav["total_waypoints"]}')

    def feedback(self, message):
        feedback = message.feedback
        self.nav.update(distance=finite(feedback.distance_remaining),
                        eta=feedback.estimated_time_remaining.sec,
                        recoveries=feedback.number_of_recoveries)

    def reconcile_goal(self):
        if self.terminal_goal is not None:
            handle, status, received = self.terminal_goal
            if self.goal_handle is handle and time.monotonic()-received >= 1:
                self.goal_handle = None
                self.terminal_goal = None
                self.cancel_requested_at = None
                message = {4: 'Arrived', 5: 'Canceled', 6: 'Navigation aborted'}[status]
                self.nav.update(state=message, active=False)
                self.event(message+' confirmed by action status. Result reply unavailable.')
                if self.goal_result_future is not None:
                    self.goal_result_future.cancel()
                    self.goal_result_future = None
        if (self.cancel_requested_at is not None and self.goal_handle is not None and
                time.monotonic()-self.cancel_requested_at > 5):
            self.nav['state'] = 'Cancellation stalled. Waiting for the waypoint server to finish.'

    def goal_finished(self, future, handle=None):
        if handle is not None and self.goal_handle is not handle:
            return
        self.goal_handle = None
        self.goal_result_future = None
        self.terminal_goal = None
        self.cancel_requested_at = None
        try:
            result = future.result()
            self.nav.update(state={4: 'Arrived', 5: 'Canceled', 6: 'Navigation aborted'}.get(result.status, 'Finished'), active=False)
            if self.nav.get('mode') == 'waypoints':
                missed = result.result.missed_waypoints
                self.nav['missed_waypoints'] = [w.waypoint_index for w in missed]
                if result.result.error_code or result.result.missed_waypoints:
                    self.nav['state'] = result.result.error_msg or 'Route finished with missed waypoints'
                if missed and missed[0].error_msg:
                    self.nav['state'] = f'Waypoint {missed[0].waypoint_index + 1} failed: {missed[0].error_msg}'
            self.event(self.nav['state'])
        except Exception as exc:
            self.nav.update(state=str(exc), active=False)

    def cancel(self):
        self.recovery.stop(False)
        if self.pending_goal:
            self.pending_goal['cancel'] = True
            if self.pending_goal['phase'] == 'converting':
                future = self.pending_goal['future']
                self.pending_goal = None
                self.from_ll.remove_pending_request(future)
                future.cancel()
        if self.cancel_requested_at is None:
            self.cancel_requested_at = time.monotonic()
        self.nav.update(state='Cancel requested')
        clients = [client for client in self.cancel_clients if client.service_is_ready()]
        if not clients:
            self.nav.update(state='Cancel service unavailable')
            return 503, {'error': 'Navigation cancel service unavailable'}
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        futures = [client.call_async(CancelGoal.Request()) for client in clients]
        for future in futures:
            future.add_done_callback(lambda f: self.canceled(futures))
        self.event('Cancel requested')
        return 202, {'accepted': True}

    def canceled(self, futures):
        if not all(f.done() for f in futures):
            return
        if not self.goal_handle and not self.pending_goal and not self.nav.get('active'):
            return
        try:
            canceling = any(f.result().goals_canceling for f in futures)
            still_active = bool(canceling or self.goal_handle or self.external_active)
            self.nav.update(state='Canceling. Waiting for navigation to finish.' if still_active else 'No active goal',
                            active=still_active)
        except Exception as exc:
            self.event(str(exc))

    def update_traction(self, now):
        result = self.traction.evaluate(now)
        result['recovery'] = self.recovery.tick(result, bool(self.goal_handle or self.external_active))
        if result['state'] != self.traction_state:
            self.traction_state = result['state']
            message = f'Traction {result["state"]}: {result["message"]}'
            self.event(message)
            self.get_logger().info(message + f' {result["metrics"]}')
        if self.traction_log and now - self.traction_log_updated >= 1.0:
            try:
                self.traction_writer.writerow(dict(time_utc=time.time(), state=result['state'],
                                                   duration=result['duration'], **result['metrics']))
                self.traction_log.flush()
                self.traction_log_updated = now
            except OSError as exc:
                self.get_logger().warning(f'Traction recording stopped: {exc}')
                self.traction_log.close()
                self.traction_log = None
        result['log_path'] = self.traction_log_path if self.traction_log else None
        return result

    def snapshot(self):
        self.update_costmap()
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
                              trail=list(self.trail),
                              components=components, battery=self.battery, tuning=self.tuning.snapshot(),
                              traction=self.update_traction(now),
                              ground_filter=dict(self.filter_settings, pending=bool(self.filter_pending),
                                                 age=now - self.filter_settings.get('received', 0)),
                              navigation=dict(self.nav, busy=bool(self.pending_goal or self.goal_handle or self.external_active or self.recovery.busy)),
                              events=self.events, system=system))

    def close(self):
        self.tuning.close()
        if self.context.ok():
            self.recovery.stop(False)
        if self.traction_log:
            self.traction_log.close()
            self.traction_log = None
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
