from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bno085_params = LaunchConfiguration("bno085_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bno085_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("box_bot"), "config", "bno085_i2c.yaml"]
                ),
            ),
            Node(
                package="bno08x_driver",
                executable="bno08x_driver",
                name="bno08x_driver",
                output="screen",
                parameters=[bno085_params],
                remappings=[
                    ("/imu", "/imu/data"),
                    ("/magnetic_field", "/imu/mag_raw"),
                ],
            ),
        ]
    )
