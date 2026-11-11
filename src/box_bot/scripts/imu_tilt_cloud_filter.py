#!/usr/bin/env python3

import math
import numpy as np
from array import array
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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


def _height_above_terrain(x, y, level_z, chassis_z, ground_height, max_slope):
    """Estimate ground from low returns connected to the visible near floor."""
    height = level_z - ground_height
    if len(x) < 3 or not 0.0 < max_slope < 45.0:
        return height

    cell_size = 0.30
    grid_x = np.floor(x / cell_size).astype(np.intp)
    grid_y = np.floor(y / cell_size).astype(np.intp)
    min_y = grid_y.min()
    width = grid_y.max() - min_y + 1
    # Sort scalar keys to avoid a row-wise sort of the whole point cloud.
    keys, cell_ids, counts = np.unique(
        grid_x * width + (grid_y - min_y), return_inverse=True, return_counts=True,
    )
    cells = list(zip(*np.divmod(keys, width)))
    # A lower quantile ignores isolated low depth noise and the tops of
    # objects in cells where the floor is also visible.
    order = np.lexsort((level_z, cell_ids))
    starts = np.cumsum(counts) - counts
    floor_sample = order[starts + (counts - 1) // 5]
    floor = level_z[floor_sample]
    supported = counts >= 3
    lookup = {cell: i for i, cell in enumerate(cells)}

    near = (x[floor_sample] >= 0.0) & (np.hypot(x[floor_sample], y[floor_sample]) <= 2.0)
    # Use the chassis plane only to seed the search. Once on a bank, its
    # first visible ground can already be well above the gravity-level plane.
    seeds = np.flatnonzero(supported & near &
                           (np.abs(chassis_z[floor_sample] - ground_height) <= 0.08))
    connected = np.zeros(len(cells), dtype=bool)
    connected[seeds] = True
    adjacent_ground = np.full(len(cells), np.inf)
    queue = deque(seeds)
    step_limit = min(0.12, cell_size * math.tan(math.radians(max_slope)) + 0.02)
    while queue:
        index = queue.popleft()
        ix, iy = cells[index]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = lookup.get((ix + dx, iy + dy))
            if neighbor is None or connected[neighbor]:
                continue
            if supported[neighbor] and abs(floor[neighbor] - floor[index]) <= step_limit:
                connected[neighbor] = True
                queue.append(neighbor)
            else:
                adjacent_ground[neighbor] = min(adjacent_ground[neighbor], floor[index])

    # A disconnected return next to the surface can still be measured
    # against it without treating that return as additional ground.
    reference = np.where(connected, floor, adjacent_ground)[cell_ids]
    known = np.isfinite(reference)
    height[known] = level_z[known] - reference[known]
    return height


class ImuTiltCloudFilter(Node):
    def __init__(self):
        super().__init__('imu_tilt_cloud_filter')

        self.declare_parameter('input_cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('output_cloud_topic', '/camera/depth/obstacles')
        self.declare_parameter('output_clearing_cloud_topic', '/camera/depth/clearing')
        self.declare_parameter('clearing_max_range', 6.0)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('ground_height', -0.10)
        self.declare_parameter('terrain_max_slope_degrees', 18.0)
        self.declare_parameter('min_obstacle_height', 0.15)
        self.declare_parameter('max_obstacle_height', 1.0)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('downsample_step', 6)
        self.declare_parameter('max_points', 16000)
        self.declare_parameter('use_imu_tilt', True)
        self.declare_parameter('imu_timeout_seconds', 0.5)
        # Robot-specific IMU/mount alignment, calibrated against flat ground.
        # Apply here only: navigation consumes the original IMU unchanged.
        self.declare_parameter('imu_roll_offset_degrees', 0.0)
        self.declare_parameter('imu_pitch_offset_degrees', 0.0)

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
        self._clearing_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_clearing_cloud_topic').value),
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
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
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
        clearing_range = float(self.get_parameter('clearing_max_range').value)
        step = max(1, int(self.get_parameter('downsample_step').value))
        max_points = int(self.get_parameter('max_points').value)

        total_points = msg.width * msg.height
        if max_points > 0:
            step = max(step, math.ceil(total_points / max_points))

        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cr = math.cos(roll)
        sr = math.sin(roll)

        # Read strided XYZ fields in one operation, including organised clouds
        # with row padding and either byte order. Only copy sampled points.
        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': [endian + 'f4'] * 3,
                          'offsets': [field_offsets[name].offset for name in ('x', 'y', 'z')],
                          'itemsize': msg.point_step})
        records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                             strides=(msg.row_step, msg.point_step))
        indices = np.arange(0, total_points, step)
        sampled = records[indices // msg.width, indices % msg.width] if total_points else records.ravel()
        x, y, z = [sampled[name].astype(np.float64) for name in ('x', 'y', 'z')]
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[finite], y[finite], z[finite]
        bx = tf_rot[0][0] * x + tf_rot[0][1] * y + tf_rot[0][2] * z + tf_trans.x
        by = tf_rot[1][0] * x + tf_rot[1][1] * y + tf_rot[1][2] * z + tf_trans.y
        bz = tf_rot[2][0] * x + tf_rot[2][1] * y + tf_rot[2][2] * z + tf_trans.z
        planar_range = np.hypot(bx, by)
        level_z = (-sp * bx) + (cp * sr * by) + (cp * cr * bz)
        points = np.column_stack((bx, by, bz))
        in_range = planar_range >= min_range
        # Ground returns stay available for ray clearing, independently of the
        # obstacle-height slider.
        clearing_points = points[in_range & (planar_range <= clearing_range)]
        marking_range = in_range & (planar_range <= max_range)
        height = _height_above_terrain(
            bx[marking_range], by[marking_range], level_z[marking_range], bz[marking_range],
            ground_height, self.get_parameter('terrain_max_slope_degrees').value,
        )
        obstacle_points = points[marking_range][(height >= min_height) & (height <= max_height)]

        out = self._create_xyz_cloud(msg.header, obstacle_points)
        self._pub.publish(out)
        self._clearing_pub.publish(self._create_xyz_cloud(msg.header, clearing_points))

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
        data = np.asarray(points, dtype='<f4').reshape(-1, 3).tobytes()
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
        roll, pitch = _roll_pitch_from_quat(self._latest_imu.orientation)
        roll += math.radians(float(self.get_parameter('imu_roll_offset_degrees').value))
        pitch += math.radians(float(self.get_parameter('imu_pitch_offset_degrees').value))
        return roll, pitch


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
