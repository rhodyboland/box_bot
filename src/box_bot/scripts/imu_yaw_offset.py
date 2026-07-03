#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavRELPOSNED

try:
    from ublox_msgs.msg import NavRELPOSNED9
except ImportError:
    NavRELPOSNED9 = None


RELPOS_FLAG_GNSS_FIX_OK = 1
RELPOS_FLAG_DIFF_SOLN = 2
RELPOS_FLAG_REL_POS_VALID = 4
RELPOS_FLAG_CARR_SOLN_MASK = 24
RELPOS_FLAG_CARR_SOLN_FIXED = 16
RELPOS_FLAG_REL_POS_HEAD_VALID = 256


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ImuYawOffset(Node):
    def __init__(self):
        super().__init__('gps_heading_offset')
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('filter_alpha', 0.35)
        self.declare_parameter('max_yaw_rate', 2.0)
        self.declare_parameter('yaw_covariance_floor', 0.0076154355)
        self.declare_parameter('reset_after_seconds', 1.0)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('gate_with_relposned', True)
        self.declare_parameter('relposned_timeout_seconds', 0.5)
        self.declare_parameter('baseline_length_min', 0.34)
        self.declare_parameter('baseline_length_max', 0.46)
        self.declare_parameter('require_fixed_baseline', True)
        self.declare_parameter('hold_heading_on_reject', True)
        self.declare_parameter('rejected_yaw_covariance', 1.0)
        self._yaw_offset = float(self.get_parameter('yaw_offset').value)
        self._filter_alpha = float(self.get_parameter('filter_alpha').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self._yaw_covariance_floor = float(
            self.get_parameter('yaw_covariance_floor').value)
        self._reset_after = float(self.get_parameter('reset_after_seconds').value)
        self._invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self._gate_with_relposned = bool(
            self.get_parameter('gate_with_relposned').value)
        self._relposned_timeout = float(
            self.get_parameter('relposned_timeout_seconds').value)
        self._baseline_length_min = float(
            self.get_parameter('baseline_length_min').value)
        self._baseline_length_max = float(
            self.get_parameter('baseline_length_max').value)
        self._require_fixed_baseline = bool(
            self.get_parameter('require_fixed_baseline').value)
        self._hold_heading_on_reject = bool(
            self.get_parameter('hold_heading_on_reject').value)
        self._rejected_yaw_covariance = float(
            self.get_parameter('rejected_yaw_covariance').value)
        self._filtered_yaw = None
        self._last_time = None
        self._limited_count = 0
        self._rejected_count = 0
        self._relpos_ok = False
        self._relpos_reason = 'waiting'
        self._last_relpos_time = None
        self._pub = self.create_publisher(Imu, 'imu/out', 10)
        self.create_subscription(Imu, 'imu/in', self._callback, 10)
        relpos_msg_type = NavRELPOSNED9 if NavRELPOSNED9 is not None else NavRELPOSNED
        self.create_subscription(
            relpos_msg_type, 'relposned', self._relpos_callback, 10)
        self.get_logger().info(
            f'Applying GPS heading yaw offset: {self._yaw_offset:.6f} rad, '
            f'alpha={self._filter_alpha:.2f}, '
            f'max_yaw_rate={self._max_yaw_rate:.2f} rad/s, '
            f'invert_yaw={self._invert_yaw}, '
            f'baseline_gate={self._gate_with_relposned}')

    def _relpos_callback(self, msg):
        flags = int(msg.flags)
        length_m = self._baseline_length(msg)

        required_flags = (
            RELPOS_FLAG_GNSS_FIX_OK
            | RELPOS_FLAG_DIFF_SOLN
            | RELPOS_FLAG_REL_POS_VALID
            | RELPOS_FLAG_REL_POS_HEAD_VALID
        )
        flags_ok = (flags & required_flags) == required_flags
        fixed_ok = (
            not self._require_fixed_baseline
            or (flags & RELPOS_FLAG_CARR_SOLN_MASK) == RELPOS_FLAG_CARR_SOLN_FIXED
        )
        length_ok = (
            self._baseline_length_min <= length_m <= self._baseline_length_max
        )

        self._relpos_ok = flags_ok and fixed_ok and length_ok
        self._last_relpos_time = self.get_clock().now()

        if not flags_ok:
            self._relpos_reason = f'flags={flags}'
        elif not fixed_ok:
            self._relpos_reason = f'not_fixed flags={flags}'
        elif not length_ok:
            self._relpos_reason = f'length={length_m:.3f}m'
        else:
            self._relpos_reason = 'ok'

    def _baseline_length(self, msg):
        if hasattr(msg, 'rel_pos_length'):
            return (
                float(msg.rel_pos_length) * 0.01
                + float(msg.rel_pos_hp_length) * 0.0001
            )
        n = float(msg.rel_pos_n) * 0.01 + float(msg.rel_pos_hpn) * 0.0001
        e = float(msg.rel_pos_e) * 0.01 + float(msg.rel_pos_hpe) * 0.0001
        d = float(msg.rel_pos_d) * 0.01 + float(msg.rel_pos_hpd) * 0.0001
        return math.sqrt(n * n + e * e + d * d)

    def _relpos_gate_ok(self, now):
        if not self._gate_with_relposned:
            return True
        if self._last_relpos_time is None:
            self._relpos_reason = 'waiting'
            return False
        age = (now - self._last_relpos_time).nanoseconds * 1e-9
        if age > self._relposned_timeout:
            self._relpos_reason = f'stale={age:.2f}s'
            return False
        return self._relpos_ok

    def _callback(self, msg):
        now = self.get_clock().now()
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            now = Time.from_msg(msg.header.stamp)

        if not self._relpos_gate_ok(now):
            self._rejected_count += 1
            if self._rejected_count == 1 or self._rejected_count % 25 == 0:
                self.get_logger().warn(
                    'Rejected GPS heading '
                    f'({self._rejected_count} total): {self._relpos_reason}')
            if self._hold_heading_on_reject and self._filtered_yaw is not None:
                self._publish_yaw(
                    msg,
                    self._filtered_yaw,
                    self._rejected_yaw_covariance
                )
            return

        input_yaw = _yaw_from_quaternion(msg.orientation)
        if self._invert_yaw:
            input_yaw = -input_yaw
        measured_yaw = _normalize_angle(
            input_yaw + self._yaw_offset)

        yaw = measured_yaw
        if self._filtered_yaw is not None and self._last_time is not None:
            dt = max((now - self._last_time).nanoseconds * 1e-9, 0.0)
            if 0.0 < dt <= self._reset_after:
                delta = _normalize_angle(measured_yaw - self._filtered_yaw)
                max_delta = self._max_yaw_rate * dt
                if abs(delta) > max_delta:
                    delta = math.copysign(max_delta, delta)
                    self._limited_count += 1
                    if self._limited_count == 1 or self._limited_count % 25 == 0:
                        self.get_logger().warn(
                            'Limited GPS heading jump '
                            f'({self._limited_count} total)')
                yaw = _normalize_angle(
                    self._filtered_yaw + self._filter_alpha * delta)

        self._filtered_yaw = yaw
        self._last_time = now

        self._publish_yaw(msg, yaw, self._yaw_covariance_floor)

    def _publish_yaw(self, msg, yaw, yaw_covariance_floor):
        out = Imu()
        out.header = msg.header
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = math.sin(yaw * 0.5)
        out.orientation.w = math.cos(yaw * 0.5)
        out.orientation_covariance = list(msg.orientation_covariance)
        if out.orientation_covariance[0] != -1.0:
            out.orientation_covariance[8] = max(
                out.orientation_covariance[8],
                yaw_covariance_floor
            )
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)


def main():
    rclpy.init()
    node = ImuYawOffset()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
