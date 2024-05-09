from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/rhody/ros2_dev/src/hoverboard_ros2_control/hoverboard_demo_bringup/launch/hoverboard.launch.py'
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/rhody/ros2_dev/src/nmea_navsat_driver/launch/nmea_serial_driver.launch.py'
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/rhody/ros2_dev/src/sllidar_ros2/launch/sllidar_c1_launch.py'
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/rhody/ros2_dev/src/py_magnetometer_driver/launch/magnetometer.launch.py'
            ])
        ),
        # Include more launch files as needed 
    ])
