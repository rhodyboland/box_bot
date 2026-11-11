"""Verify wheel-spin pose cannot rotate the local EKF against a stationary gyro."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def test_local_heading_tracks_gyro_during_encoder_slip(tmp_path):
    environment = dict(os.environ, ROS_DOMAIN_ID='195')
    params = Path(__file__).resolve().parents[1] / 'config' / 'ekf_gps.yaml'
    with (tmp_path / 'ekf.log').open('w') as log:
        process = subprocess.Popen([
            '/opt/ros/lyrical/lib/robot_localization/ekf_node', '--ros-args',
            '--params-file', str(params), '-r', '__node:=ekf_filter_node_odom',
            '-r', '/odometry/filtered:=/odometry/local',
        ], env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(domain_id=195)
        node = rclpy.create_node('slip_simulation')
        wheel = node.create_publisher(Odometry, '/hoverboard_base_controller/odom', 10)
        imu = node.create_publisher(Imu, '/imu/data', 10)
        output = []
        node.create_subscription(Odometry, '/odometry/local', output.append, qos_profile_sensor_data)

        def run(seconds, gyro_rate):
            start = time.monotonic()
            while time.monotonic() - start < seconds:
                stamp = node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = stamp, 'odom'
                odom.child_frame_id = 'base_footprint'
                # Encoders claim a changed pose and turning, while body velocity
                # is zero. Only the IMU measures the actual rotation below.
                odom.pose.pose.position.x = 10.0
                odom.pose.pose.position.y = 5.0
                odom.pose.pose.orientation.z = math.sin(0.75)
                odom.pose.pose.orientation.w = math.cos(0.75)
                odom.twist.twist.angular.z = 1.0
                for index in (0, 7, 14, 21, 28, 35):
                    odom.pose.covariance[index] = 0.08
                    odom.twist.covariance[index] = 0.03
                measurement = Imu()
                measurement.header.stamp, measurement.header.frame_id = stamp, 'base_footprint'
                measurement.orientation.w = 1.0
                measurement.angular_velocity.z = gyro_rate
                for index in (0, 4, 8):
                    measurement.angular_velocity_covariance[index] = 0.0001
                wheel.publish(odom)
                imu.publish(measurement)
                rclpy.spin_once(node, timeout_sec=0.03)

        def heading():
            q = output[-1].pose.pose.orientation
            return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

        try:
            run(4.0, 0.0)
            assert output, (tmp_path / 'ekf.log').read_text()
            assert abs(heading()) < 0.03
            assert abs(output[-1].pose.pose.position.x) < 0.05
            assert abs(output[-1].pose.pose.position.y) < 0.05
            before = heading()
            run(2.0, 0.3)
            assert 0.40 < heading() - before < 0.80
            assert abs(output[-1].twist.twist.angular.z - 0.3) < 0.03
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
