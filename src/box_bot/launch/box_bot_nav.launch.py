from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# ros2 launch nav2_launcher navigation_launch.py use_sim_time:=false

# ros2 launch slam_toolbox online_async_launch.py slam_params_file:="./nav2_launcher/config/mapper_params_online_async.yaml"

# ros2 run teleop_twist_keyboard teleop_twist_keyboard

#rviz2 -d ./nav2_launcher/config/map.rviz 

def generate_launch_description():
    # Get the robot_localization config file
    pkg_share = get_package_share_directory('nav2_launcher')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # New icm node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('icm_20948'), 'launch', 'icm_20948.launch.py'
                ])
            ])
        ),
        # ROS2 Control launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('hoverboard_control'), 'launch', 'hoverboard.launch.py'
                ])
            ])
        ),
        # Delayed LIDAR launch
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py'
                        ])
                    ])
                )
            ]
        ),
        # Delayed SLAM launch
        TimerAction(
            period=10.0,  # Delay in seconds
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('nav2_launcher'), 'launch', 'online_async_launch.py'
                        ])
                    ])
                )
            ]
        ),
        # Add IMU Filter Madgwick
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter',
        #     parameters=[{
        #         'use_mag': True,
        #         'publish_tf': False,
        #         'world_frame': 'enu',
        #         'use_magnetic_field_msg': True,  # Set to True if your mag data comes as MagneticField msg
        #         'fixed_frame': 'base_link',
        #         'publish_debug_topics': True,
        #         'gain': 0.1,
        #         'zeta': 0.0,
        #         'mag_bias_x': 0.0,
        #         'mag_bias_y': 0.0,
        #         'mag_bias_z': 0.0,
        #         # Assuming your topics - adjust if different
        #         'imu/data_raw_topic': 'imu/data_raw',
        #         'imu/mag_topic': 'imu/mag_raw'
        #     }],
        #     remappings=[
        #         ('imu/data_raw', 'imu/data_raw'),
        #         ('imu/mag', 'imu/mag_raw'),
        #         ('imu/data', 'imu/data'),
        #     ],
        # ),
        # robot_localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        TimerAction(
            period=20.0,  # Delay in seconds
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('nav2_launcher'), 'launch', 'nav2_launch.py'
                        ])
                    ])
                )
            ]
        )

    ])