#!/usr/bin/env python3

import math
import struct
from array import array

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


FLOAT32 = PointField.FLOAT32


def _quat_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    s = 2.0 / n
    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s
    return (
        (1.0 - yy - zz, xy - wz, xz + wy),
        (xy + wz, 1.0 - xx - zz, yz - wx),
        (xz - wy, yz + wx, 1.0 - xx - yy),
    )


def _mat_vec(m, x, y, z):
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def _roll_pitch_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return roll, pitch


class ImuTiltCloudFilter(Node):
    def __init__(self):
        super().__init__('imu_tilt_cloud_filter')

        self.declare_parameter('input_cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('output_cloud_topic', '/camera/depth/obstacles')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('ground_height', -0.10)
        self.declare_parameter('min_obstacle_height', 0.15)
        self.declare_parameter('max_obstacle_height', 1.0)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('downsample_step', 6)
        self.declare_parameter('max_points', 16000)
        self.declare_parameter('use_imu_tilt', True)
        self.declare_parameter('imu_timeout_seconds', 0.5)

        self._target_frame = str(self.get_parameter('target_frame').value)
        self._latest_imu = None
        self._latest_imu_time = None
        self._last_warn_time = self.get_clock().now()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_cloud_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self._imu_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_cloud_topic').value),
            self._cloud_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Filtering {self.get_parameter("input_cloud_topic").value} to '
            f'{self.get_parameter("output_cloud_topic").value} in {self._target_frame}'
        )

    def _imu_cb(self, msg):
        self._latest_imu = msg
        self._latest_imu_time = self.get_clock().now()

    def _warn_throttled(self, text):
        now = self.get_clock().now()
        if (now - self._last_warn_time) > Duration(seconds=5.0):
            self._last_warn_time = now
            self.get_logger().warn(text)

    def _cloud_cb(self, msg):
        field_offsets = {field.name: field for field in msg.fields}
        if not all(name in field_offsets for name in ('x', 'y', 'z')):
            self._warn_throttled('PointCloud2 is missing x/y/z fields')
            return
        if any(field_offsets[name].datatype != FLOAT32 for name in ('x', 'y', 'z')):
            self._warn_throttled('PointCloud2 x/y/z fields are not float32')
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.03),
            )
        except TransformException as exc:
            self._warn_throttled(
                f'Waiting for transform {self._target_frame} <- {msg.header.frame_id}: {exc}'
            )
            return

        tf_rot = _quat_to_matrix(transform.transform.rotation)
        tf_trans = transform.transform.translation
        roll, pitch = self._current_roll_pitch()

        ground_height = float(self.get_parameter('ground_height').value)
        min_height = float(self.get_parameter('min_obstacle_height').value)
        max_height = float(self.get_parameter('max_obstacle_height').value)
        min_range = float(self.get_parameter('min_range').value)
        max_range = float(self.get_parameter('max_range').value)
        step = max(1, int(self.get_parameter('downsample_step').value))
        max_points = int(self.get_parameter('max_points').value)

        endian = '>' if msg.is_bigendian else '<'
        unpack_x = struct.Struct(endian + 'f').unpack_from
        unpack_y = struct.Struct(endian + 'f').unpack_from
        unpack_z = struct.Struct(endian + 'f').unpack_from
        x_offset = field_offsets['x'].offset
        y_offset = field_offsets['y'].offset
        z_offset = field_offsets['z'].offset
        total_points = msg.width * msg.height
        if max_points > 0:
            step = max(step, math.ceil(total_points / max_points))

        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cr = math.cos(roll)
        sr = math.sin(roll)

        obstacle_points = []
        for index in range(0, total_points, step):
            row = index // msg.width
            col = index - row * msg.width
            offset = row * msg.row_step + col * msg.point_step
            x = unpack_x(msg.data, offset + x_offset)[0]
            y = unpack_y(msg.data, offset + y_offset)[0]
            z = unpack_z(msg.data, offset + z_offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            bx, by, bz = _mat_vec(tf_rot, x, y, z)
            bx += tf_trans.x
            by += tf_trans.y
            bz += tf_trans.z

            planar_range = math.hypot(bx, by)
            if planar_range < min_range or planar_range > max_range:
                continue

            level_z = (-sp * bx) + (cp * sr * by) + (cp * cr * bz)
            height = level_z - ground_height
            if height < min_height or height > max_height:
                continue

            obstacle_points.append((bx, by, bz))

        out = self._create_xyz_cloud(msg.header, obstacle_points)
        self._pub.publish(out)

    def _create_xyz_cloud(self, header, points):
        out = PointCloud2()
        out.header.stamp = header.stamp
        out.header.frame_id = self._target_frame
        out.height = 1
        out.width = len(points)
        out.fields = [
            PointField(name='x', offset=0, datatype=FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = out.point_step * out.width
        out.is_dense = True
        data = bytearray(out.row_step)
        pack = struct.Struct('<fff').pack_into
        for index, point in enumerate(points):
            pack(data, index * out.point_step, *point)
        out.data = array('B', data)
        return out

    def _current_roll_pitch(self):
        if not bool(self.get_parameter('use_imu_tilt').value):
            return 0.0, 0.0
        if self._latest_imu is None or self._latest_imu_time is None:
            return 0.0, 0.0
        timeout = float(self.get_parameter('imu_timeout_seconds').value)
        if (self.get_clock().now() - self._latest_imu_time) > Duration(seconds=timeout):
            return 0.0, 0.0
        return _roll_pitch_from_quat(self._latest_imu.orientation)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTiltCloudFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
