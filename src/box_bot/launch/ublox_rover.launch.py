from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ublox_params = LaunchConfiguration('ublox_params')
    ekf_params = LaunchConfiguration('ekf_params')
    nav2_params = LaunchConfiguration('nav2_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ublox_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'zed_f9p_rover.yaml']
            )
        ),
        DeclareLaunchArgument(
            'ekf_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'ekf_gps.yaml']
            )
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'nav2_minimal.yaml']
            )
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('box_bot'), 'launch', 'bno085.launch.py'
                ])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('hoverboard_control'), 'launch', 'hoverboard.launch.py'
                ])
            )
        ),

        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[ublox_params],
            remappings=[
                ('/fix', '/gps/fix'),
                ('/navheading', '/gps/heading'),
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_params],
            remappings=[('odometry/filtered', '/odometry/local')]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('gps/fix', '/gps/fix'),
                ('imu/data', '/gps/heading'),
                ('odometry/filtered', '/odometry/local'),
                ('odometry/gps', '/odometry/gps'),
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_params],
            remappings=[('odometry/filtered', '/odometry/global')]
        ),

        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('nav2_launcher'), 'launch', 'nav2_launch.py'
                        ])
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'params_file': nav2_params,
                        'log_level': log_level,
                    }.items()
                )
            ]
        ),
    ])
