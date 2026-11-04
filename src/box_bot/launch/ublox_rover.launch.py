"""Compatibility entry point for the BOXBOT dual-X20P navigation launch."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('box_bot'), 'launch', 'box_bot_nav_gps.launch.py',
            ])),
            launch_arguments={'start_realsense': 'false'}.items(),
        ),
    ])
