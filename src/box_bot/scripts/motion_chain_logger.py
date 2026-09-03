#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import Buffer, TransformException, TransformListener
from ublox_msgs.msg import NavCOV, NavPVT, NavRELPOSNED, NavSTATUS, RxmRTCM

try:
    from ublox_msgs.msg import NavRELPOSNED9
except ImportError:
    NavRELPOSNED9 = None


FIX_TYPES = {
    0: 'none',
    1: 'dead_reckoning',
    2: '2d',
    3: '3d',
    4: 'gnss_dead_reckoning',
    5: 'time_only',
}

CARRIER_SOLUTIONS = {
    0: 'none',
    1: 'float',
    2: 'fixed',
}

NAVSAT_STATUSES = {
    -1: 'no_fix',
    0: 'fix',
    1: 'sbas',
    2: 'gbas',
}


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_degrees(degrees):
    while degrees > 180.0:
        degrees -= 360.0
    while degrees < -180.0:
        degrees += 360.0
    return degrees


def _deg(rad):
    if rad is None:
        return None
    return math.degrees(rad)


def _age_seconds(now, stamp):
    if stamp is None:
        return None
    if stamp.nanoseconds == 0:
        return 0.0
    return (now - stamp).nanoseconds * 1e-9


def _latlon_delta_en_m(lat, lon, ref_lat, ref_lon):
    radius_m = 6378137.0
    east_m = (
        math.radians(lon - ref_lon)
        * radius_m
        * math.cos(math.radians(ref_lat))
    )
    north_m = math.radians(lat - ref_lat) * radius_m
    return east_m, north_m


def _pvt_carrier_solution(flags):
    return CARRIER_SOLUTIONS.get((int(flags) & 0xC0) >> 6, 'reserved')


def _relpos_carrier_solution(flags):
    return CARRIER_SOLUTIONS.get((int(flags) >> 3) & 0x03, 'reserved')


def _relpos_component_m(cm, hp):
    return float(cm) * 0.01 + float(hp) * 0.0001


def _relpos_length_m(msg):
    if hasattr(msg, 'rel_pos_length'):
        return _relpos_component_m(msg.rel_pos_length, msg.rel_pos_hp_length)
    n = _relpos_component_m(msg.rel_pos_n, msg.rel_pos_hpn)
    e = _relpos_component_m(msg.rel_pos_e, msg.rel_pos_hpe)
    d = _relpos_component_m(msg.rel_pos_d, msg.rel_pos_hpd)
    return math.sqrt(n * n + e * e + d * d)


def _format(value, precision=6):
    if value is None:
        return ''
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return ''
        return f'{value:.{precision}f}'
    return value


def _format_field(name, value):
    if name.endswith('_lat_deg') or name.endswith('_lon_deg'):
        return _format(value, 9)
    return _format(value)


class MotionChainLogger(Node):
    def __init__(self):
        super().__init__('motion_chain_logger')

        self.declare_parameter('output_path', '')
        self.declare_parameter('sample_period', 0.1)
        self.declare_parameter('console_period', 1.0)
        self.declare_parameter('duration_seconds', 0.0)
        self.declare_parameter('tf_timeout_seconds', 0.02)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('gps_frame', 'gps')
        self.declare_parameter('moving_base_frame', 'gps_moving_base')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_odom_topic', '/hoverboard_base_controller/odom')
        self.declare_parameter('local_odom_topic', '/odometry/local')
        self.declare_parameter('global_odom_topic', '/odometry/global')
        self.declare_parameter('gps_odom_topic', '/odometry/gps')
        self.declare_parameter('gps_odom_alt_topic', '/navsat_transform/odometry/gps')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('global_plan_topic', '/plan')
        self.declare_parameter('transformed_plan_topic', '/transformed_global_plan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_heading_topic', '/gps/heading')
        self.declare_parameter('corrected_heading_topic', '/gps/heading_corrected')
        self.declare_parameter('relposned_topic', '/gps/relposned')
        self.declare_parameter('rover_fix_topic', '/gps_rover/fix')
        self.declare_parameter('moving_base_fix_topic', '/gps_moving_base/fix')
        self.declare_parameter('rover_navpvt_topic', '/gps_rover/navpvt')
        self.declare_parameter('moving_base_navpvt_topic', '/gps_moving_base/navpvt')
        self.declare_parameter('rover_navstatus_topic', '/gps_rover/navstatus')
        self.declare_parameter('moving_base_navstatus_topic', '/gps_moving_base/navstatus')
        self.declare_parameter('rover_navcov_topic', '/gps_rover/navcov')
        self.declare_parameter('rover_rxmrtcm_topic', '/gps_rover/rxmrtcm')
        self.declare_parameter('moving_base_rxmrtcm_topic', '/gps_moving_base/rxmrtcm')

        self._latest = {}
        self._unwrapped_yaws = {}
        self._samples = 0
        self._start_time = self.get_clock().now()
        self._last_console_time = self._start_time

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._create_subscriptions()
        self._fields = self._fieldnames()
        self._csv_file = open(self._output_path(), 'w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._csv_file, fieldnames=self._fields)
        self._writer.writeheader()
        self._csv_file.flush()

        sample_period = float(self.get_parameter('sample_period').value)
        self.create_timer(sample_period, self._sample)
        self.get_logger().info(
            f'Logging motion chain at {1.0 / sample_period:.1f} Hz to '
            f'{self._csv_file.name}'
        )

    def destroy_node(self):
        if hasattr(self, '_csv_file') and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
        super().destroy_node()

    def _output_path(self):
        output_path = str(self.get_parameter('output_path').value)
        if output_path:
            return os.path.expanduser(output_path)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser('~/.ros/log')
        os.makedirs(log_dir, exist_ok=True)
        return os.path.join(log_dir, f'boxbot_motion_chain_{stamp}.csv')

    def _create_subscriptions(self):
        relpos_type = NavRELPOSNED9 if NavRELPOSNED9 is not None else NavRELPOSNED
        subscriptions = [
            ('cmd_vel', Twist, 'cmd_vel_topic'),
            ('wheel_odom', Odometry, 'wheel_odom_topic'),
            ('local_odom', Odometry, 'local_odom_topic'),
            ('global_odom', Odometry, 'global_odom_topic'),
            ('goal_pose', PoseStamped, 'goal_pose_topic'),
            ('global_plan', Path, 'global_plan_topic'),
            ('transformed_plan', Path, 'transformed_plan_topic'),
            ('imu', Imu, 'imu_topic'),
            ('gps_heading', Imu, 'gps_heading_topic'),
            ('corrected_heading', Imu, 'corrected_heading_topic'),
            ('relpos', relpos_type, 'relposned_topic'),
            ('rover_fix', NavSatFix, 'rover_fix_topic'),
            ('moving_base_fix', NavSatFix, 'moving_base_fix_topic'),
            ('rover_pvt', NavPVT, 'rover_navpvt_topic'),
            ('moving_base_pvt', NavPVT, 'moving_base_navpvt_topic'),
            ('rover_status', NavSTATUS, 'rover_navstatus_topic'),
            ('moving_base_status', NavSTATUS, 'moving_base_navstatus_topic'),
            ('rover_cov', NavCOV, 'rover_navcov_topic'),
            ('rover_rtcm', RxmRTCM, 'rover_rxmrtcm_topic'),
            ('moving_base_rtcm', RxmRTCM, 'moving_base_rxmrtcm_topic'),
        ]

        for key, msg_type, parameter_name in subscriptions:
            self.create_subscription(
                msg_type,
                self.get_parameter(parameter_name).value,
                lambda msg, key=key: self._store(key, msg),
                50,
            )

        for parameter_name in ('gps_odom_topic', 'gps_odom_alt_topic'):
            topic = self.get_parameter(parameter_name).value
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic=topic: self._store_gps_odom(msg, topic),
                50,
            )

    def _store(self, key, msg):
        self._latest[key] = (msg, self.get_clock().now())

    def _store_gps_odom(self, msg, source_topic):
        self._latest['gps_odom'] = (msg, self.get_clock().now())
        self._latest['gps_odom_source'] = (source_topic, self.get_clock().now())

    def _fieldnames(self):
        fields = [
            'sample_time',
            'elapsed_s',
            'cmd_age_s',
            'cmd_vx_mps',
            'cmd_wz_radps',
        ]

        for prefix in ('wheel_odom', 'local_odom', 'global_odom', 'gps_odom'):
            fields.extend([
                f'{prefix}_age_s',
                f'{prefix}_source',
                f'{prefix}_frame_id',
                f'{prefix}_child_frame_id',
                f'{prefix}_x_m',
                f'{prefix}_y_m',
                f'{prefix}_yaw_deg',
                f'{prefix}_yaw_unwrapped_deg',
                f'{prefix}_vx_mps',
                f'{prefix}_vy_mps',
                f'{prefix}_wz_radps',
                f'{prefix}_pose_cov_x',
                f'{prefix}_pose_cov_y',
                f'{prefix}_pose_cov_yaw',
                f'{prefix}_twist_cov_vx',
                f'{prefix}_twist_cov_vy',
                f'{prefix}_twist_cov_wz',
            ])

        for prefix in ('imu', 'gps_heading', 'corrected_heading'):
            fields.extend([
                f'{prefix}_age_s',
                f'{prefix}_yaw_deg',
                f'{prefix}_yaw_unwrapped_deg',
                f'{prefix}_yaw_cov',
                f'{prefix}_wz_radps',
                f'{prefix}_wz_cov',
                f'{prefix}_ax_mps2',
                f'{prefix}_ay_mps2',
                f'{prefix}_az_mps2',
            ])

        fields.extend([
            'goal_pose_age_s',
            'goal_pose_frame_id',
            'goal_pose_x_m',
            'goal_pose_y_m',
            'goal_pose_yaw_deg',
            'goal_distance_global_m',
            'goal_bearing_global_deg',
            'goal_heading_error_global_deg',
        ])

        for prefix in ('global_plan', 'transformed_plan'):
            fields.extend([
                f'{prefix}_age_s',
                f'{prefix}_frame_id',
                f'{prefix}_pose_count',
                f'{prefix}_start_x_m',
                f'{prefix}_start_y_m',
                f'{prefix}_end_x_m',
                f'{prefix}_end_y_m',
                f'{prefix}_length_m',
                f'{prefix}_end_distance_global_m',
            ])

        fields.extend([
            'relpos_age_s',
            'relpos_itow_ms',
            'relpos_valid',
            'relpos_heading_valid',
            'relpos_diff',
            'relpos_carrier',
            'relpos_flags',
            'relpos_ref_station_id',
            'relpos_n_m',
            'relpos_e_m',
            'relpos_d_m',
            'relpos_length_m',
            'relpos_heading_deg',
            'relpos_heading_unwrapped_deg',
            'relpos_heading_acc_deg',
            'relpos_acc_n_m',
            'relpos_acc_e_m',
            'relpos_acc_d_m',
            'relpos_acc_length_m',
        ])

        for prefix in ('rover', 'moving_base'):
            fields.extend([
                f'{prefix}_fix_age_s',
                f'{prefix}_fix_status',
                f'{prefix}_lat_deg',
                f'{prefix}_lon_deg',
                f'{prefix}_alt_m',
                f'{prefix}_fix_cov_x',
                f'{prefix}_fix_cov_y',
                f'{prefix}_fix_cov_z',
                f'{prefix}_pvt_age_s',
                f'{prefix}_pvt_itow_ms',
                f'{prefix}_pvt_lat_deg',
                f'{prefix}_pvt_lon_deg',
                f'{prefix}_pvt_height_m',
                f'{prefix}_pvt_h_msl_m',
                f'{prefix}_pvt_fix_type',
                f'{prefix}_pvt_carrier',
                f'{prefix}_pvt_diff',
                f'{prefix}_pvt_num_sv',
                f'{prefix}_pvt_h_acc_m',
                f'{prefix}_pvt_v_acc_m',
                f'{prefix}_pvt_vel_n_mps',
                f'{prefix}_pvt_vel_e_mps',
                f'{prefix}_pvt_vel_d_mps',
                f'{prefix}_pvt_g_speed_mps',
                f'{prefix}_pvt_s_acc_mps',
                f'{prefix}_pvt_head_mot_deg',
                f'{prefix}_pvt_head_veh_deg',
                f'{prefix}_pvt_head_acc_deg',
                f'{prefix}_status_age_s',
                f'{prefix}_status_itow_ms',
                f'{prefix}_status_fix',
                f'{prefix}_status_fix_ok',
                f'{prefix}_status_diff',
                f'{prefix}_rtcm_age_s',
                f'{prefix}_rtcm_msg_type',
                f'{prefix}_rtcm_ref_station',
                f'{prefix}_rtcm_crc_ok',
            ])

        fields.extend([
            'dual_fix_baseline_e_m',
            'dual_fix_baseline_n_m',
            'dual_fix_baseline_length_m',
            'dual_fix_baseline_heading_deg',
            'dual_pvt_baseline_e_m',
            'dual_pvt_baseline_n_m',
            'dual_pvt_baseline_length_m',
            'dual_pvt_baseline_heading_deg',
            'rover_cov_age_s',
            'rover_cov_itow_ms',
            'rover_cov_pos_valid',
            'rover_cov_vel_valid',
            'rover_cov_pos_nn',
            'rover_cov_pos_ee',
            'rover_cov_pos_dd',
            'rover_cov_vel_nn',
            'rover_cov_vel_ee',
            'rover_cov_vel_dd',
        ])

        for prefix in ('tf_map_odom', 'tf_odom_base', 'tf_map_base',
                       'tf_base_gps', 'tf_base_moving_base'):
            fields.extend([
                f'{prefix}_ok',
                f'{prefix}_age_s',
                f'{prefix}_x_m',
                f'{prefix}_y_m',
                f'{prefix}_z_m',
                f'{prefix}_yaw_deg',
                f'{prefix}_yaw_unwrapped_deg',
            ])

        return fields

    def _sample(self):
        now = self.get_clock().now()
        row = {
            'sample_time': f'{now.nanoseconds * 1e-9:.9f}',
            'elapsed_s': _format((now - self._start_time).nanoseconds * 1e-9),
        }

        self._add_cmd(row, now)
        self._add_odometry(row, now)
        self._add_goal_and_plans(row, now)
        self._add_imus(row, now)
        self._add_relpos(row, now)
        self._add_gps(row, now)
        self._add_dual_gps_baselines(row)
        self._add_nav_cov(row, now)
        self._add_transforms(row, now)

        self._writer.writerow({
            name: _format_field(name, row.get(name))
            for name in self._fields
        })
        self._samples += 1
        if self._samples % 10 == 0:
            self._csv_file.flush()

        self._maybe_log_console(now, row)
        self._maybe_stop(now)

    def _latest_msg(self, key):
        return self._latest.get(key, (None, None))

    def _unwrap_yaw_deg(self, key, yaw_deg):
        if yaw_deg is None:
            return None
        state = self._unwrapped_yaws.get(key)
        if state is None:
            self._unwrapped_yaws[key] = (yaw_deg, yaw_deg)
            return yaw_deg

        previous_wrapped, previous_unwrapped = state
        delta = yaw_deg - previous_wrapped
        while delta > 180.0:
            delta -= 360.0
        while delta < -180.0:
            delta += 360.0

        unwrapped = previous_unwrapped + delta
        self._unwrapped_yaws[key] = (yaw_deg, unwrapped)
        return unwrapped

    def _add_cmd(self, row, now):
        msg, stamp = self._latest_msg('cmd_vel')
        row['cmd_age_s'] = _age_seconds(now, stamp)
        if msg is None:
            return
        row['cmd_vx_mps'] = msg.linear.x
        row['cmd_wz_radps'] = msg.angular.z

    def _add_odometry(self, row, now):
        for key in ('wheel_odom', 'local_odom', 'global_odom', 'gps_odom'):
            msg, stamp = self._latest_msg(key)
            row[f'{key}_age_s'] = _age_seconds(now, stamp)
            if msg is None:
                continue

            if key == 'gps_odom':
                source, _ = self._latest_msg('gps_odom_source')
                row[f'{key}_source'] = source

            row[f'{key}_frame_id'] = msg.header.frame_id
            row[f'{key}_child_frame_id'] = msg.child_frame_id
            yaw_deg = _deg(_yaw_from_quaternion(msg.pose.pose.orientation))
            row[f'{key}_x_m'] = msg.pose.pose.position.x
            row[f'{key}_y_m'] = msg.pose.pose.position.y
            row[f'{key}_yaw_deg'] = yaw_deg
            row[f'{key}_yaw_unwrapped_deg'] = self._unwrap_yaw_deg(key, yaw_deg)
            row[f'{key}_vx_mps'] = msg.twist.twist.linear.x
            row[f'{key}_vy_mps'] = msg.twist.twist.linear.y
            row[f'{key}_wz_radps'] = msg.twist.twist.angular.z
            row[f'{key}_pose_cov_x'] = msg.pose.covariance[0]
            row[f'{key}_pose_cov_y'] = msg.pose.covariance[7]
            row[f'{key}_pose_cov_yaw'] = msg.pose.covariance[35]
            row[f'{key}_twist_cov_vx'] = msg.twist.covariance[0]
            row[f'{key}_twist_cov_vy'] = msg.twist.covariance[7]
            row[f'{key}_twist_cov_wz'] = msg.twist.covariance[35]

    def _global_pose_xy_yaw(self):
        msg, _ = self._latest_msg('global_odom')
        if msg is None:
            return None
        return (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _deg(_yaw_from_quaternion(msg.pose.pose.orientation)),
        )

    def _add_goal_and_plans(self, row, now):
        global_pose = self._global_pose_xy_yaw()

        goal, goal_stamp = self._latest_msg('goal_pose')
        row['goal_pose_age_s'] = _age_seconds(now, goal_stamp)
        if goal is not None:
            row['goal_pose_frame_id'] = goal.header.frame_id
            goal_x = goal.pose.position.x
            goal_y = goal.pose.position.y
            goal_yaw_deg = _deg(_yaw_from_quaternion(goal.pose.orientation))
            row['goal_pose_x_m'] = goal_x
            row['goal_pose_y_m'] = goal_y
            row['goal_pose_yaw_deg'] = goal_yaw_deg

            if global_pose is not None:
                robot_x, robot_y, robot_yaw_deg = global_pose
                dx = goal_x - robot_x
                dy = goal_y - robot_y
                bearing_deg = _deg(math.atan2(dy, dx))
                row['goal_distance_global_m'] = math.hypot(dx, dy)
                row['goal_bearing_global_deg'] = bearing_deg
                row['goal_heading_error_global_deg'] = _wrap_degrees(
                    bearing_deg - robot_yaw_deg
                )

        for key in ('global_plan', 'transformed_plan'):
            plan, plan_stamp = self._latest_msg(key)
            row[f'{key}_age_s'] = _age_seconds(now, plan_stamp)
            if plan is None:
                continue

            row[f'{key}_frame_id'] = plan.header.frame_id
            row[f'{key}_pose_count'] = len(plan.poses)
            if not plan.poses:
                continue

            first = plan.poses[0].pose.position
            last = plan.poses[-1].pose.position
            row[f'{key}_start_x_m'] = first.x
            row[f'{key}_start_y_m'] = first.y
            row[f'{key}_end_x_m'] = last.x
            row[f'{key}_end_y_m'] = last.y

            length = 0.0
            previous = first
            for pose_stamped in plan.poses[1:]:
                current = pose_stamped.pose.position
                length += math.hypot(current.x - previous.x, current.y - previous.y)
                previous = current
            row[f'{key}_length_m'] = length

            if global_pose is not None:
                robot_x, robot_y, _ = global_pose
                row[f'{key}_end_distance_global_m'] = math.hypot(
                    last.x - robot_x,
                    last.y - robot_y,
                )

    def _add_imus(self, row, now):
        for key in ('imu', 'gps_heading', 'corrected_heading'):
            msg, stamp = self._latest_msg(key)
            row[f'{key}_age_s'] = _age_seconds(now, stamp)
            if msg is None:
                continue

            yaw_deg = _deg(_yaw_from_quaternion(msg.orientation))
            row[f'{key}_yaw_deg'] = yaw_deg
            row[f'{key}_yaw_unwrapped_deg'] = self._unwrap_yaw_deg(key, yaw_deg)
            row[f'{key}_yaw_cov'] = msg.orientation_covariance[8]
            row[f'{key}_wz_radps'] = msg.angular_velocity.z
            row[f'{key}_wz_cov'] = msg.angular_velocity_covariance[8]
            row[f'{key}_ax_mps2'] = msg.linear_acceleration.x
            row[f'{key}_ay_mps2'] = msg.linear_acceleration.y
            row[f'{key}_az_mps2'] = msg.linear_acceleration.z

    def _add_relpos(self, row, now):
        msg, stamp = self._latest_msg('relpos')
        row['relpos_age_s'] = _age_seconds(now, stamp)
        if msg is None:
            return

        flags = int(msg.flags)
        heading_deg = math.fmod(float(getattr(msg, 'rel_pos_heading', 0)) * 1.0e-5, 360.0)
        row['relpos_itow_ms'] = msg.i_tow
        row['relpos_valid'] = bool(flags & int(msg.FLAGS_REL_POS_VALID))
        row['relpos_heading_valid'] = bool(flags & getattr(msg, 'FLAGS_REL_POS_HEAD_VALID', 0))
        row['relpos_diff'] = bool(flags & int(msg.FLAGS_DIFF_SOLN))
        row['relpos_carrier'] = _relpos_carrier_solution(flags)
        row['relpos_flags'] = flags
        row['relpos_ref_station_id'] = msg.ref_station_id
        row['relpos_n_m'] = _relpos_component_m(msg.rel_pos_n, msg.rel_pos_hpn)
        row['relpos_e_m'] = _relpos_component_m(msg.rel_pos_e, msg.rel_pos_hpe)
        row['relpos_d_m'] = _relpos_component_m(msg.rel_pos_d, msg.rel_pos_hpd)
        row['relpos_length_m'] = _relpos_length_m(msg)
        row['relpos_heading_deg'] = heading_deg
        row['relpos_heading_unwrapped_deg'] = self._unwrap_yaw_deg(
            'relpos_heading',
            heading_deg,
        )
        row['relpos_heading_acc_deg'] = (
            float(getattr(msg, 'acc_heading', 0)) * 1.0e-5
        )
        row['relpos_acc_n_m'] = float(msg.acc_n) * 0.0001
        row['relpos_acc_e_m'] = float(msg.acc_e) * 0.0001
        row['relpos_acc_d_m'] = float(msg.acc_d) * 0.0001
        row['relpos_acc_length_m'] = (
            float(getattr(msg, 'acc_length', 0)) * 0.0001
        )

    def _add_gps(self, row, now):
        for prefix, fix_key, pvt_key, status_key, rtcm_key in (
            ('rover', 'rover_fix', 'rover_pvt', 'rover_status', 'rover_rtcm'),
            (
                'moving_base',
                'moving_base_fix',
                'moving_base_pvt',
                'moving_base_status',
                'moving_base_rtcm',
            ),
        ):
            fix, fix_stamp = self._latest_msg(fix_key)
            row[f'{prefix}_fix_age_s'] = _age_seconds(now, fix_stamp)
            if fix is not None:
                row[f'{prefix}_fix_status'] = NAVSAT_STATUSES.get(
                    int(fix.status.status),
                    int(fix.status.status),
                )
                row[f'{prefix}_lat_deg'] = fix.latitude
                row[f'{prefix}_lon_deg'] = fix.longitude
                row[f'{prefix}_alt_m'] = fix.altitude
                row[f'{prefix}_fix_cov_x'] = fix.position_covariance[0]
                row[f'{prefix}_fix_cov_y'] = fix.position_covariance[4]
                row[f'{prefix}_fix_cov_z'] = fix.position_covariance[8]

            pvt, pvt_stamp = self._latest_msg(pvt_key)
            row[f'{prefix}_pvt_age_s'] = _age_seconds(now, pvt_stamp)
            if pvt is not None:
                flags = int(pvt.flags)
                row[f'{prefix}_pvt_itow_ms'] = pvt.i_tow
                row[f'{prefix}_pvt_lat_deg'] = float(pvt.lat) * 1.0e-7
                row[f'{prefix}_pvt_lon_deg'] = float(pvt.lon) * 1.0e-7
                row[f'{prefix}_pvt_height_m'] = float(pvt.height) * 0.001
                row[f'{prefix}_pvt_h_msl_m'] = float(pvt.h_msl) * 0.001
                row[f'{prefix}_pvt_fix_type'] = FIX_TYPES.get(
                    int(pvt.fix_type),
                    int(pvt.fix_type),
                )
                row[f'{prefix}_pvt_carrier'] = _pvt_carrier_solution(flags)
                row[f'{prefix}_pvt_diff'] = bool(flags & int(pvt.FLAGS_DIFF_SOLN))
                row[f'{prefix}_pvt_num_sv'] = pvt.num_sv
                row[f'{prefix}_pvt_h_acc_m'] = float(pvt.h_acc) * 0.001
                row[f'{prefix}_pvt_v_acc_m'] = float(pvt.v_acc) * 0.001
                row[f'{prefix}_pvt_vel_n_mps'] = float(pvt.vel_n) * 0.001
                row[f'{prefix}_pvt_vel_e_mps'] = float(pvt.vel_e) * 0.001
                row[f'{prefix}_pvt_vel_d_mps'] = float(pvt.vel_d) * 0.001
                row[f'{prefix}_pvt_g_speed_mps'] = float(pvt.g_speed) * 0.001
                row[f'{prefix}_pvt_s_acc_mps'] = float(pvt.s_acc) * 0.001
                row[f'{prefix}_pvt_head_mot_deg'] = float(pvt.heading) * 1.0e-5
                row[f'{prefix}_pvt_head_veh_deg'] = float(pvt.head_veh) * 1.0e-5
                row[f'{prefix}_pvt_head_acc_deg'] = float(pvt.head_acc) * 1.0e-5

            status, status_stamp = self._latest_msg(status_key)
            row[f'{prefix}_status_age_s'] = _age_seconds(now, status_stamp)
            if status is not None:
                flags = int(status.flags)
                row[f'{prefix}_status_itow_ms'] = status.i_tow
                row[f'{prefix}_status_fix'] = FIX_TYPES.get(
                    int(status.gps_fix),
                    int(status.gps_fix),
                )
                row[f'{prefix}_status_fix_ok'] = bool(flags & int(status.FLAGS_GPS_FIX_OK))
                row[f'{prefix}_status_diff'] = bool(flags & int(status.FLAGS_DIFF_SOLN))

            rtcm, rtcm_stamp = self._latest_msg(rtcm_key)
            row[f'{prefix}_rtcm_age_s'] = _age_seconds(now, rtcm_stamp)
            if rtcm is not None:
                row[f'{prefix}_rtcm_msg_type'] = rtcm.msg_type
                row[f'{prefix}_rtcm_ref_station'] = rtcm.ref_station
                row[f'{prefix}_rtcm_crc_ok'] = not bool(
                    int(rtcm.flags) & int(rtcm.FLAGS_CRC_FAILED)
                )

    def _add_dual_gps_baselines(self, row):
        rover_fix, _ = self._latest_msg('rover_fix')
        moving_base_fix, _ = self._latest_msg('moving_base_fix')
        if rover_fix is not None and moving_base_fix is not None:
            east_m, north_m = _latlon_delta_en_m(
                moving_base_fix.latitude,
                moving_base_fix.longitude,
                rover_fix.latitude,
                rover_fix.longitude,
            )
            row['dual_fix_baseline_e_m'] = east_m
            row['dual_fix_baseline_n_m'] = north_m
            row['dual_fix_baseline_length_m'] = math.hypot(east_m, north_m)
            row['dual_fix_baseline_heading_deg'] = (
                math.degrees(math.atan2(east_m, north_m)) + 360.0
            ) % 360.0

        rover_pvt, _ = self._latest_msg('rover_pvt')
        moving_base_pvt, _ = self._latest_msg('moving_base_pvt')
        if rover_pvt is not None and moving_base_pvt is not None:
            rover_lat = float(rover_pvt.lat) * 1.0e-7
            rover_lon = float(rover_pvt.lon) * 1.0e-7
            moving_base_lat = float(moving_base_pvt.lat) * 1.0e-7
            moving_base_lon = float(moving_base_pvt.lon) * 1.0e-7
            east_m, north_m = _latlon_delta_en_m(
                moving_base_lat,
                moving_base_lon,
                rover_lat,
                rover_lon,
            )
            row['dual_pvt_baseline_e_m'] = east_m
            row['dual_pvt_baseline_n_m'] = north_m
            row['dual_pvt_baseline_length_m'] = math.hypot(east_m, north_m)
            row['dual_pvt_baseline_heading_deg'] = (
                math.degrees(math.atan2(east_m, north_m)) + 360.0
            ) % 360.0

    def _add_nav_cov(self, row, now):
        msg, stamp = self._latest_msg('rover_cov')
        row['rover_cov_age_s'] = _age_seconds(now, stamp)
        if msg is None:
            return

        row['rover_cov_itow_ms'] = msg.i_tow
        row['rover_cov_pos_valid'] = bool(msg.pos_cov_valid)
        row['rover_cov_vel_valid'] = bool(msg.vel_cov_valid)
        row['rover_cov_pos_nn'] = msg.pos_cov_nn
        row['rover_cov_pos_ee'] = msg.pos_cov_ee
        row['rover_cov_pos_dd'] = msg.pos_cov_dd
        row['rover_cov_vel_nn'] = msg.vel_cov_nn
        row['rover_cov_vel_ee'] = msg.vel_cov_ee
        row['rover_cov_vel_dd'] = msg.vel_cov_dd

    def _add_transforms(self, row, now):
        map_frame = self.get_parameter('map_frame').value
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        gps_frame = self.get_parameter('gps_frame').value
        moving_base_frame = self.get_parameter('moving_base_frame').value

        transforms = [
            ('tf_map_odom', map_frame, odom_frame),
            ('tf_odom_base', odom_frame, base_frame),
            ('tf_map_base', map_frame, base_frame),
            ('tf_base_gps', base_frame, gps_frame),
            ('tf_base_moving_base', base_frame, moving_base_frame),
        ]

        timeout = Duration(seconds=float(self.get_parameter('tf_timeout_seconds').value))
        for prefix, target_frame, source_frame in transforms:
            try:
                tf = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout,
                )
            except TransformException:
                row[f'{prefix}_ok'] = False
                continue

            stamp = Time.from_msg(tf.header.stamp)
            yaw_deg = _deg(_yaw_from_quaternion(tf.transform.rotation))
            row[f'{prefix}_ok'] = True
            row[f'{prefix}_age_s'] = _age_seconds(now, stamp)
            row[f'{prefix}_x_m'] = tf.transform.translation.x
            row[f'{prefix}_y_m'] = tf.transform.translation.y
            row[f'{prefix}_z_m'] = tf.transform.translation.z
            row[f'{prefix}_yaw_deg'] = yaw_deg
            row[f'{prefix}_yaw_unwrapped_deg'] = self._unwrap_yaw_deg(
                prefix,
                yaw_deg,
            )

    def _maybe_log_console(self, now, row):
        console_period = float(self.get_parameter('console_period').value)
        if console_period <= 0.0:
            return
        if (now - self._last_console_time).nanoseconds * 1e-9 < console_period:
            return
        self._last_console_time = now

        self.get_logger().info(
            ' | '.join([
                f'samples={self._samples}',
                f'cmd wz={_format(row.get("cmd_wz_radps"), 3)}',
                (
                    'yaw imu/gps/corr/local='
                    f'{_format(row.get("imu_yaw_deg"), 2)}/'
                    f'{_format(row.get("gps_heading_yaw_deg"), 2)}/'
                    f'{_format(row.get("corrected_heading_yaw_deg"), 2)}/'
                    f'{_format(row.get("local_odom_yaw_deg"), 2)}'
                ),
                (
                    'relpos '
                    f'hdg={_format(row.get("relpos_heading_deg"), 2)} '
                    f'valid={row.get("relpos_heading_valid", "")} '
                    f'carr={row.get("relpos_carrier", "")} '
                    f'age={_format(row.get("relpos_age_s"), 1)}'
                ),
                (
                    'fix_base '
                    f'len={_format(row.get("dual_fix_baseline_length_m"), 2)}'
                ),
                (
                    'tf '
                    f'odom_base={row.get("tf_odom_base_ok", "")} '
                    f'map_base={row.get("tf_map_base_ok", "")}'
                ),
            ])
        )

    def _maybe_stop(self, now):
        duration = float(self.get_parameter('duration_seconds').value)
        if duration <= 0.0:
            return
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        if elapsed >= duration:
            self.get_logger().info(
                f'Finished after {elapsed:.1f}s, wrote {self._samples} samples '
                f'to {self._csv_file.name}'
            )
            rclpy.shutdown()


def main():
    rclpy.init()
    node = MotionChainLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(
            f'Wrote {node._samples} samples to {node._csv_file.name}'
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
