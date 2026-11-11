"""Check measured mount/IMU alignment without sacrificing real obstacles."""
import math
from array import array
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from imu_tilt_cloud_filter import ImuTiltCloudFilter


def test_flat_ground_removed_and_raised_obstacles_preserved():
    rclpy.init(domain_id=168)
    node = ImuTiltCloudFilter()
    output = []
    clearing = []
    node._clearing_pub = SimpleNamespace(publish=clearing.append)
    node._pub = SimpleNamespace(publish=output.append)
    try:
        node.set_parameters([Parameter('downsample_step', value=1)])
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'test_ground_cloud'
        transform.transform.rotation.w = 1.0
        node._tf_buffer.set_transform_static(transform, 'test')
        imu = Imu()
        roll, pitch = map(math.radians, (-1.4710568, -1.9797042))
        imu.orientation.x = math.sin(roll / 2) * math.cos(pitch / 2)
        imu.orientation.y = math.cos(roll / 2) * math.sin(pitch / 2)
        imu.orientation.z = -math.sin(roll / 2) * math.sin(pitch / 2)
        imu.orientation.w = math.cos(roll / 2) * math.cos(pitch / 2)
        node._imu_cb(imu)
        # Independently fitted physical ground from the flat-ground capture.
        points = []
        for x, y in [(1.0, 0.0), (3.0, 0.0), (4.5, 0.4)]:
            ground = 0.01713489 * x + 0.02164184 * y - 0.11681188
            points.extend([(x, y, ground), (x, y, ground + 0.20)])
        cloud = node._create_xyz_cloud(Header(), points)
        cloud.header.frame_id = 'test_ground_cloud'
        node._cloud_cb(cloud)
        # Reproduce the old false marking of distant ground.
        old = list(struct.iter_unpack('<fff', output[-1].data))
        assert any(abs(z - points[4][2]) < 1e-5 for x, y, z in old)
        node.set_parameters([
            Parameter('ground_height', value=-0.117),
            Parameter('imu_roll_offset_degrees', value=0.25),
            Parameter('imu_pitch_offset_degrees', value=3.0),
        ])
        node._cloud_cb(cloud)
        corrected = list(struct.iter_unpack('<fff', output[-1].data))
        assert len(corrected) == 3
        # Clearing retains ground returns and the original timestamp.
        assert len(list(struct.iter_unpack('<fff', clearing[-1].data))) == 6
        assert clearing[-1].header.stamp == cloud.header.stamp
        assert clearing[-1].header.frame_id == 'base_link'
        for actual, expected in zip(corrected, points[1::2]):
            assert all(abs(a - b) < 1e-5 for a, b in zip(actual, expected))
        # Raising the live cutoff for grass removes the 20 cm objects too.
        node.set_parameters([Parameter('min_obstacle_height', value=0.25)])
        node._cloud_cb(cloud)
        assert len(output[-1].data) == 0
        assert len(list(struct.iter_unpack('<fff', clearing[-1].data))) == 6
        node.set_parameters([Parameter('min_obstacle_height', value=0.15)])
        node._cloud_cb(cloud)
        assert len(list(struct.iter_unpack('<fff', output[-1].data))) == 3
        # Disabling tilt must not leave an unwanted fixed rotation applied.
        node.set_parameters([Parameter('use_imu_tilt', value=False)])
        assert node._current_roll_pitch() == (0.0, 0.0)
        node.set_parameters([Parameter('max_points', value=3)])
        sampled = node._create_xyz_cloud(Header(), [(x, 0.0, 0.3) for x in [0.1, 1, 2, 3, 4, 5, 6, 7, 8]])
        sampled.header.frame_id = 'test_ground_cloud'
        node._cloud_cb(sampled)
        assert [p[0] for p in struct.iter_unpack('<fff', clearing[-1].data)] == [3.0, 6.0]
        assert [p[0] for p in struct.iter_unpack('<fff', output[-1].data)] == [3.0]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_organised_big_endian_cloud_with_padding_and_invalid_points():
    rclpy.init(domain_id=168)
    node = ImuTiltCloudFilter()
    output, clearing = [], []
    node._pub = SimpleNamespace(publish=output.append)
    node._clearing_pub = SimpleNamespace(publish=clearing.append)
    try:
        node.set_parameters([Parameter('downsample_step', value=1), Parameter('use_imu_tilt', value=False)])
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'padded_cloud'
        transform.transform.rotation.w = 1.0
        node._tf_buffer.set_transform_static(transform, 'test')
        points = [(1, 0, 0.2), (2, 0, math.nan), (0.1, 0, 0.2),
                  (3, 1, 0.5), (7, 0, 0.3), (2, 0, -0.1)]
        cloud = node._create_xyz_cloud(Header(), [])
        cloud.header.frame_id = 'padded_cloud'
        cloud.width, cloud.height = 3, 2
        cloud.point_step, cloud.row_step = 20, 72
        cloud.is_bigendian = True
        for field, offset in zip(cloud.fields, (4, 8, 12)):
            field.offset = offset
        data = bytearray(cloud.height * cloud.row_step)
        for index, point in enumerate(points):
            struct.pack_into('>fff', data, (index // 3) * 72 + (index % 3) * 20 + 4, *point)
        cloud.data = array('B', data)
        node._cloud_cb(cloud)
        assert len(output[-1].data) == 24
        assert [p[0] for p in struct.iter_unpack('<fff', output[-1].data)] == [1.0, 3.0]
        assert [p[0] for p in struct.iter_unpack('<fff', clearing[-1].data)] == [1.0, 3.0, 2.0]
        empty = node._create_xyz_cloud(Header(), [])
        empty.header.frame_id = 'padded_cloud'
        node._cloud_cb(empty)
        assert len(output[-1].data) == len(clearing[-1].data) == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_connected_contour_is_removed_but_post_and_step_are_kept():
    rclpy.init(domain_id=168)
    node = ImuTiltCloudFilter()
    output, clearing = [], []
    node._pub = SimpleNamespace(publish=output.append)
    node._clearing_pub = SimpleNamespace(publish=clearing.append)
    try:
        node.set_parameters([Parameter('downsample_step', value=1),
                             Parameter('use_imu_tilt', value=False)])
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'contour'
        transform.transform.rotation.w = 1.0
        node._tf_buffer.set_transform_static(transform, 'test')

        def ground(x, y):
            # A curved bank with some cross slope rises over 50 cm at range.
            return -.10 + .065 * max(x - 1.5, 0.) ** 2 + .035 * y

        floor = [(x / 40, y / 40, ground(x / 40, y / 40))
                 for x in range(30, 194) for y in range(-24, 25)]
        post = [(3.85, .1, ground(3.85, .1) + height) for height in (.20, .30, .60)]
        cloud = node._create_xyz_cloud(Header(), floor + post)
        cloud.header.frame_id = 'contour'

        def filtered():
            node._cloud_cb(cloud)
            return list(struct.iter_unpack('<fff', output[-1].data))

        # A fixed floor sees the bank as an obstacle even with the slider at 50 cm.
        node.set_parameters([Parameter('terrain_max_slope_degrees', value=0.0),
                             Parameter('min_obstacle_height', value=.50)])
        assert len(filtered()) > 100
        node.set_parameters([Parameter('terrain_max_slope_degrees', value=18.0)])
        assert len(filtered()) == 1
        node.set_parameters([Parameter('min_obstacle_height', value=.15)])
        assert len(filtered()) == len(post)
        assert clearing[-1].width == len(floor) + len(post)

        # A raised surface separated by a step must never seed new ground.
        step = [(x / 40, y / 40, -.10 if x < 96 else .45)
                for x in range(30, 194) for y in range(-24, 25)]
        cloud = node._create_xyz_cloud(Header(), step)
        cloud.header.frame_id = 'contour'
        node.set_parameters([Parameter('min_obstacle_height', value=.50)])
        assert len(filtered()) == sum(x >= 2.4 for x, _, _ in step)
    finally:
        node.destroy_node()
        rclpy.shutdown()
