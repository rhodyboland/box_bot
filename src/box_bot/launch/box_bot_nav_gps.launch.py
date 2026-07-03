from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    f9p_params = LaunchConfiguration('f9p_params')
    ntrip_params = LaunchConfiguration('ntrip_params')
    ekf_params = LaunchConfiguration('ekf_params')
    nav2_params = LaunchConfiguration('nav2_params')
    rover_device = LaunchConfiguration('rover_device')
    moving_base_device = LaunchConfiguration('moving_base_device')
    rover_baudrate = LaunchConfiguration('rover_baudrate')
    moving_base_baudrate = LaunchConfiguration('moving_base_baudrate')
    start_ntrip = LaunchConfiguration('start_ntrip')
    start_gps_status = LaunchConfiguration('start_gps_status')
    gps_status_period = LaunchConfiguration('gps_status_period')
    gps_heading_yaw_offset = LaunchConfiguration('gps_heading_yaw_offset')
    gps_heading_filter_alpha = LaunchConfiguration('gps_heading_filter_alpha')
    gps_heading_max_yaw_rate = LaunchConfiguration('gps_heading_max_yaw_rate')
    gps_heading_covariance_floor = LaunchConfiguration('gps_heading_covariance_floor')
    gps_heading_invert = LaunchConfiguration('gps_heading_invert')
    gps_heading_baseline_min = LaunchConfiguration('gps_heading_baseline_min')
    gps_heading_baseline_max = LaunchConfiguration('gps_heading_baseline_max')
    navsat_log_level = LaunchConfiguration('navsat_log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    ntrip_defaults = {
        'host': '127.0.0.1',
        'port': 2101,
        'mode': 'ntrip',
        'mountpoint': '',
        'ntrip_version': '',
        'authenticate': False,
        'username': '',
        'password': '',
        'ssl': False,
        'cert': 'None',
        'key': 'None',
        'ca_cert': 'None',
        'rtcm_frame_id': 'gps',
        'rtcm_message_package': 'rtcm_msgs',
        'nmea_max_length': 128,
        'nmea_min_length': 3,
        'reconnect_attempt_max': 10,
        'reconnect_attempt_wait_seconds': 5,
        'rtcm_timeout_seconds': 4,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'f9p_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'zed_f9p_dual.yaml']
            )
        ),
        DeclareLaunchArgument(
            'ekf_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'ekf_gps.yaml']
            )
        ),
        DeclareLaunchArgument(
            'ntrip_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'ntrip.yaml']
            )
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'nav2_minimal.yaml']
            )
        ),
        DeclareLaunchArgument('rover_device', default_value='/dev/ttyACM2'),
        DeclareLaunchArgument('moving_base_device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('rover_baudrate', default_value='115200'),
        DeclareLaunchArgument('moving_base_baudrate', default_value='115200'),
        DeclareLaunchArgument('start_ntrip', default_value='true'),
        DeclareLaunchArgument('start_gps_status', default_value='true'),
        DeclareLaunchArgument('gps_status_period', default_value='10.0'),
        DeclareLaunchArgument('gps_heading_yaw_offset', default_value='3.14159265359'),
        DeclareLaunchArgument('gps_heading_filter_alpha', default_value='0.35'),
        DeclareLaunchArgument('gps_heading_max_yaw_rate', default_value='2.0'),
        DeclareLaunchArgument('gps_heading_covariance_floor', default_value='0.030461742'),
        # With ACM2 as the rear rover antenna and ACM0 40 cm forward, the
        # ublox moving-baseline heading has the same yaw sign as wheel odom.
        DeclareLaunchArgument('gps_heading_invert', default_value='false'),
        DeclareLaunchArgument('gps_heading_baseline_min', default_value='0.34'),
        DeclareLaunchArgument('gps_heading_baseline_max', default_value='0.46'),
        DeclareLaunchArgument('navsat_log_level', default_value='warn'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('icm_20948'), 'launch', 'icm_20948.launch.py'
                ])
            ),
            launch_arguments={
                'imu_topic': '/imu/data',
                'frame_id': 'imu_link',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('hoverboard_control'), 'launch', 'hoverboard.launch.py'
                ])
            )
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='box_bot',
                    executable='rtk2go_ntrip_client.py',
                    name='ntrip_client',
                    output='screen',
                    condition=IfCondition(start_ntrip),
                    parameters=[ntrip_defaults, ntrip_params],
                    remappings=[
                        ('fix', '/gps_rover/fix'),
                        ('nmea', '/gps/nmea'),
                        ('rtcm', '/rtcm'),
                    ],
                ),
            ],
        ),

        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='gps_rover',
            output='screen',
            parameters=[
                f9p_params,
                {
                    'device': rover_device,
                    'baudrate': ParameterValue(rover_baudrate, value_type=int),
                },
            ],
            remappings=[
                ('navheading', '/gps/heading'),
                ('navrelposned', '/gps/relposned'),
                ('nmea', '/gps/nmea'),
                ('rtcm', '/rtcm'),
            ],
        ),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='gps_moving_base',
            output='screen',
            parameters=[
                f9p_params,
                {
                    'device': moving_base_device,
                    'baudrate': ParameterValue(moving_base_baudrate, value_type=int),
                },
            ],
            remappings=[
                ('fix', '/gps_moving_base/fix'),
                ('fix_velocity', '/gps_moving_base/fix_velocity'),
                ('navpvt', '/gps_moving_base/navpvt'),
                ('nmea', '/gps_moving_base/nmea'),
                ('rtcm', '/rtcm'),
            ],
        ),

        Node(
            package='box_bot',
            executable='gps_status_logger.py',
            name='gps_status_logger',
            output='screen',
            condition=IfCondition(start_gps_status),
            parameters=[{
                'period': ParameterValue(gps_status_period, value_type=float),
                'rover_fix_topic': '/gps_rover/fix',
                'rover_navpvt_topic': '/gps_rover/navpvt',
                'moving_base_fix_topic': '/gps_moving_base/fix',
                'moving_base_navpvt_topic': '/gps_moving_base/navpvt',
                'relposned_topic': '/gps/relposned',
            }],
        ),

        Node(
            package='box_bot',
            executable='imu_yaw_offset.py',
            name='gps_heading_offset',
            output='screen',
            parameters=[{
                'yaw_offset': ParameterValue(
                    gps_heading_yaw_offset,
                    value_type=float
                ),
                'filter_alpha': ParameterValue(
                    gps_heading_filter_alpha,
                    value_type=float
                ),
                'max_yaw_rate': ParameterValue(
                    gps_heading_max_yaw_rate,
                    value_type=float
                ),
                'yaw_covariance_floor': ParameterValue(
                    gps_heading_covariance_floor,
                    value_type=float
                ),
                'invert_yaw': ParameterValue(
                    gps_heading_invert,
                    value_type=bool
                ),
                'baseline_length_min': ParameterValue(
                    gps_heading_baseline_min,
                    value_type=float
                ),
                'baseline_length_max': ParameterValue(
                    gps_heading_baseline_max,
                    value_type=float
                ),
                'hold_heading_on_reject': True,
                'rejected_yaw_covariance': 1.0,
            }],
            remappings=[
                ('imu/in', '/gps/heading'),
                ('imu/out', '/gps/heading_corrected'),
                ('relposned', '/gps/relposned'),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('/odometry/filtered', '/odometry/local'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_params],
            arguments=['--ros-args', '--log-level', navsat_log_level],
            remappings=[
                ('/gps/fix', '/gps_rover/fix'),
                ('/imu', '/gps/heading_corrected'),
                ('/odometry/filtered', '/odometry/local'),
                ('/odometry/gps', '/odometry/gps'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('/odometry/filtered', '/odometry/global'),
            ],
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
                        'default_nav_to_pose_bt_xml': PathJoinSubstitution([
                            FindPackageShare('box_bot'),
                            'behavior_trees',
                            'gps_nav_to_pose.xml'
                        ]),
                    }.items()
                )
            ]
        ),
    ])
