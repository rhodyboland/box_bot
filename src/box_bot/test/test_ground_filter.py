"""Check measured mount/IMU alignment without sacrificing real obstacles."""
import math
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
        for actual, expected in zip(corrected, points[1::2]):
            assert all(abs(a - b) < 1e-5 for a, b in zip(actual, expected))
        # Disabling tilt must not leave an unwanted fixed rotation applied.
        node.set_parameters([Parameter('use_imu_tilt', value=False)])
        assert node._current_roll_pitch() == (0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
