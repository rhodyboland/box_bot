#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu


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
        self._yaw_offset = float(self.get_parameter('yaw_offset').value)
        self._filter_alpha = float(self.get_parameter('filter_alpha').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self._yaw_covariance_floor = float(
            self.get_parameter('yaw_covariance_floor').value)
        self._reset_after = float(self.get_parameter('reset_after_seconds').value)
        self._invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self._filtered_yaw = None
        self._last_time = None
        self._limited_count = 0
        self._pub = self.create_publisher(Imu, 'imu/out', 10)
        self.create_subscription(Imu, 'imu/in', self._callback, 10)
        self.get_logger().info(
            f'Applying GPS heading yaw offset: {self._yaw_offset:.6f} rad, '
            f'alpha={self._filter_alpha:.2f}, '
            f'max_yaw_rate={self._max_yaw_rate:.2f} rad/s, '
            f'invert_yaw={self._invert_yaw}')

    def _callback(self, msg):
        input_yaw = _yaw_from_quaternion(msg.orientation)
        if self._invert_yaw:
            input_yaw = -input_yaw
        measured_yaw = _normalize_angle(
            input_yaw + self._yaw_offset)

        now = self.get_clock().now()
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            now = Time.from_msg(msg.header.stamp)

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
                self._yaw_covariance_floor
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
