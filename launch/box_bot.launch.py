from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('hoverboard_control'), 'launch', 'hoverboard.launch.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('py_qmc6310_driver'), 'launch', 'qmc6310.launch.py'
                ])
            ])
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('nmea_navsat_driver'), 'launch', 'nmea_serial_driver.launch.py'
        #         ])
        #     ])
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py'
                ])
            ])
        ),
    ])