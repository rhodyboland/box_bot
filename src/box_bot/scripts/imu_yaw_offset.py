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
RELPOS_FLAG_CARR_SOLN_FLOAT = 8
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
        self.declare_parameter('baseline_length_soft_min', 0.25)
        self.declare_parameter('baseline_length_soft_max', 0.55)
        self.declare_parameter('require_fixed_baseline', True)
        self.declare_parameter('require_differential_baseline', True)
        self.declare_parameter('dynamic_yaw_covariance', True)
        self.declare_parameter('baseline_length_covariance_scale_max', 25.0)
        self.declare_parameter('float_yaw_covariance', 0.274155678)
        self.declare_parameter('no_carrier_yaw_covariance', 1.0)
        self.declare_parameter('rtk_loss_yaw_covariance', 1.0)
        self.declare_parameter('max_yaw_covariance', 1.0)
        self.declare_parameter('rate_limited_yaw_covariance', 1.0)
        self.declare_parameter('reference_imu_topic', '')
        self.declare_parameter('gate_with_reference_yaw_rate', False)
        self.declare_parameter('reference_imu_timeout_seconds', 0.5)
        self.declare_parameter('reference_yaw_rate_min', 0.08)
        self.declare_parameter('reference_yaw_rate_tolerance', 0.25)
        self.declare_parameter('reference_yaw_rate_reject_covariance', 1.0)
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
        self._baseline_length_soft_min = float(
            self.get_parameter('baseline_length_soft_min').value)
        self._baseline_length_soft_max = float(
            self.get_parameter('baseline_length_soft_max').value)
        self._require_fixed_baseline = bool(
            self.get_parameter('require_fixed_baseline').value)
        self._require_differential_baseline = bool(
            self.get_parameter('require_differential_baseline').value)
        self._dynamic_yaw_covariance = bool(
            self.get_parameter('dynamic_yaw_covariance').value)
        self._baseline_length_covariance_scale_max = float(
            self.get_parameter('baseline_length_covariance_scale_max').value)
        self._float_yaw_covariance = float(
            self.get_parameter('float_yaw_covariance').value)
        self._no_carrier_yaw_covariance = float(
            self.get_parameter('no_carrier_yaw_covariance').value)
        self._rtk_loss_yaw_covariance = float(
            self.get_parameter('rtk_loss_yaw_covariance').value)
        self._max_yaw_covariance = float(
            self.get_parameter('max_yaw_covariance').value)
        self._rate_limited_yaw_covariance = float(
            self.get_parameter('rate_limited_yaw_covariance').value)
        self._reference_imu_topic = str(
            self.get_parameter('reference_imu_topic').value)
        self._gate_with_reference_yaw_rate = bool(
            self.get_parameter('gate_with_reference_yaw_rate').value)
        self._reference_imu_timeout = float(
            self.get_parameter('reference_imu_timeout_seconds').value)
        self._reference_yaw_rate_min = float(
            self.get_parameter('reference_yaw_rate_min').value)
        self._reference_yaw_rate_tolerance = float(
            self.get_parameter('reference_yaw_rate_tolerance').value)
        self._reference_yaw_rate_reject_covariance = float(
            self.get_parameter('reference_yaw_rate_reject_covariance').value)
        self._rejected_yaw_covariance = float(
            self.get_parameter('rejected_yaw_covariance').value)
        self._filtered_yaw = None
        self._last_time = None
        self._last_measured_yaw = None
        self._last_measured_time = None
        self._limited_count = 0
        self._rejected_count = 0
        self._reference_inflated_count = 0
        self._relpos_ok = False
        self._relpos_reason = 'waiting'
        self._last_relpos_time = None
        self._relpos_flags = 0
        self._relpos_length_m = None
        self._reference_imu = None
        self._reference_imu_time = None
        self._pub = self.create_publisher(Imu, 'imu/out', 10)
        self.create_subscription(Imu, 'imu/in', self._callback, 10)
        if self._reference_imu_topic:
            self.create_subscription(
                Imu,
                self._reference_imu_topic,
                self._reference_imu_callback,
                10,
            )
        relpos_msg_type = NavRELPOSNED9 if NavRELPOSNED9 is not None else NavRELPOSNED
        self.create_subscription(
            relpos_msg_type, 'relposned', self._relpos_callback, 10)
        self.get_logger().info(
            f'Applying GPS heading yaw offset: {self._yaw_offset:.6f} rad, '
            f'alpha={self._filter_alpha:.2f}, '
            f'max_yaw_rate={self._max_yaw_rate:.2f} rad/s, '
            f'invert_yaw={self._invert_yaw}, '
            f'baseline_gate={self._gate_with_relposned}, '
            f'reference_yaw_rate_gate={self._gate_with_reference_yaw_rate}')

    def _reference_imu_callback(self, msg):
        self._reference_imu = msg
        self._reference_imu_time = self.get_clock().now()

    def _relpos_callback(self, msg):
        flags = int(msg.flags)
        length_m = self._baseline_length(msg)
        self._relpos_flags = flags
        self._relpos_length_m = length_m

        required_flags = (
            RELPOS_FLAG_GNSS_FIX_OK
            | RELPOS_FLAG_REL_POS_VALID
            | RELPOS_FLAG_REL_POS_HEAD_VALID
        )
        flags_ok = (flags & required_flags) == required_flags
        diff_ok = (
            not self._require_differential_baseline
            or bool(flags & RELPOS_FLAG_DIFF_SOLN)
        )
        fixed_ok = (
            not self._require_fixed_baseline
            or (flags & RELPOS_FLAG_CARR_SOLN_MASK) == RELPOS_FLAG_CARR_SOLN_FIXED
        )
        length_ok = (
            self._baseline_length_soft_min <= length_m <= self._baseline_length_soft_max
        )

        self._relpos_ok = flags_ok and diff_ok and fixed_ok and length_ok
        self._last_relpos_time = self.get_clock().now()

        if not flags_ok:
            self._relpos_reason = f'flags={flags}'
        elif not diff_ok:
            self._relpos_reason = f'no_diff flags={flags}'
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

        input_yaw = _yaw_from_quaternion(msg.orientation)
        if self._invert_yaw:
            input_yaw = -input_yaw
        measured_yaw = _normalize_angle(
            input_yaw + self._yaw_offset)

        relpos_ok = self._relpos_gate_ok(now)
        reference_covariance_floor = self._reference_yaw_rate_covariance(
            now,
            measured_yaw,
        )

        yaw = measured_yaw
        limited = False
        if self._filtered_yaw is not None and self._last_time is not None:
            dt = max((now - self._last_time).nanoseconds * 1e-9, 0.0)
            if 0.0 < dt <= self._reset_after:
                delta = _normalize_angle(measured_yaw - self._filtered_yaw)
                max_delta = self._max_yaw_rate * dt
                if abs(delta) > max_delta:
                    delta = math.copysign(max_delta, delta)
                    limited = True
                    self._limited_count += 1
                    if self._limited_count == 1 or self._limited_count % 25 == 0:
                        self.get_logger().warn(
                            'Limited GPS heading jump '
                            f'({self._limited_count} total)')
                yaw = _normalize_angle(
                    self._filtered_yaw + self._filter_alpha * delta)

        self._filtered_yaw = yaw
        self._last_time = now
        self._last_measured_yaw = measured_yaw
        self._last_measured_time = now

        yaw_covariance_floor = self._yaw_covariance_floor
        if not relpos_ok:
            self._rejected_count += 1
            if self._rejected_count == 1 or self._rejected_count % 25 == 0:
                self.get_logger().warn(
                    'Inflated GPS heading covariance '
                    f'({self._rejected_count} total): {self._relpos_reason}')
            yaw_covariance_floor = max(
                yaw_covariance_floor,
                self._rejected_yaw_covariance,
            )
        if reference_covariance_floor is not None:
            yaw_covariance_floor = max(
                yaw_covariance_floor,
                reference_covariance_floor,
            )
        if limited:
            yaw_covariance_floor = max(
                yaw_covariance_floor,
                self._rate_limited_yaw_covariance,
            )

        self._publish_yaw(msg, yaw, yaw_covariance_floor)

    def _reference_yaw_rate_covariance(self, now, measured_yaw):
        if not self._gate_with_reference_yaw_rate:
            return None
        if self._reference_imu is None or self._reference_imu_time is None:
            return None

        reference_age = (now - self._reference_imu_time).nanoseconds * 1e-9
        if reference_age > self._reference_imu_timeout:
            return None
        if self._last_measured_yaw is None or self._last_measured_time is None:
            return None

        dt = (now - self._last_measured_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > self._reset_after:
            return None

        measured_rate = _normalize_angle(measured_yaw - self._last_measured_yaw) / dt
        reference_rate = self._reference_imu.angular_velocity.z
        active = (
            abs(measured_rate) >= self._reference_yaw_rate_min
            or abs(reference_rate) >= self._reference_yaw_rate_min
        )
        if not active:
            return None

        error = measured_rate - reference_rate
        if abs(error) <= self._reference_yaw_rate_tolerance:
            return None

        self._reference_inflated_count += 1
        if (
            self._reference_inflated_count == 1
            or self._reference_inflated_count % 25 == 0
        ):
            self.get_logger().warn(
                'Inflated GPS heading covariance by reference yaw-rate check '
                f'({self._reference_inflated_count} total): '
                f'gps_rate={measured_rate:.3f}rad/s '
                f'ref_rate={reference_rate:.3f}rad/s '
                f'error={error:.3f}rad/s')
        return self._reference_yaw_rate_reject_covariance

    def _publish_yaw(self, msg, yaw, yaw_covariance_floor):
        out = Imu()
        out.header = msg.header
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = math.sin(yaw * 0.5)
        out.orientation.w = math.cos(yaw * 0.5)
        out.orientation_covariance = list(msg.orientation_covariance)
        if out.orientation_covariance[0] != -1.0:
            out.orientation_covariance[8] = self._yaw_covariance(
                out.orientation_covariance[8],
                yaw_covariance_floor,
            )
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)

    def _yaw_covariance(self, input_covariance, covariance_floor):
        covariance = max(input_covariance, covariance_floor)
        if not self._dynamic_yaw_covariance:
            return covariance

        flags = self._relpos_flags
        carrier = flags & RELPOS_FLAG_CARR_SOLN_MASK

        if self._relpos_length_m is not None:
            scale = self._baseline_length_covariance_scale(self._relpos_length_m)
            covariance *= scale

        if not (flags & RELPOS_FLAG_DIFF_SOLN):
            covariance = max(covariance, self._rtk_loss_yaw_covariance)
        elif carrier == RELPOS_FLAG_CARR_SOLN_FLOAT:
            covariance = max(covariance, self._float_yaw_covariance)
        elif carrier != RELPOS_FLAG_CARR_SOLN_FIXED:
            covariance = max(covariance, self._no_carrier_yaw_covariance)

        return min(covariance, self._max_yaw_covariance)

    def _baseline_length_covariance_scale(self, length_m):
        if self._baseline_length_min <= length_m <= self._baseline_length_max:
            return 1.0

        max_scale = max(self._baseline_length_covariance_scale_max, 1.0)
        if length_m < self._baseline_length_min:
            span = max(self._baseline_length_min - self._baseline_length_soft_min, 1e-6)
            ratio = (self._baseline_length_min - length_m) / span
        else:
            span = max(self._baseline_length_soft_max - self._baseline_length_max, 1e-6)
            ratio = (length_m - self._baseline_length_max) / span

        ratio = min(max(ratio, 0.0), 1.0)
        return 1.0 + ratio * (max_scale - 1.0)


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
