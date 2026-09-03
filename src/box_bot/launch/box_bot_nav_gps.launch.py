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
    navsat_fix_topic = LaunchConfiguration('navsat_fix_topic')
    ntrip_fix_topic = LaunchConfiguration('ntrip_fix_topic')
    rover_rtcm_topic = LaunchConfiguration('rover_rtcm_topic')
    moving_base_rtcm_topic = LaunchConfiguration('moving_base_rtcm_topic')
    start_lidar = LaunchConfiguration('start_lidar')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    lidar_angle_compensate = LaunchConfiguration('lidar_angle_compensate')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')
    start_ntrip = LaunchConfiguration('start_ntrip')
    start_gps_status = LaunchConfiguration('start_gps_status')
    start_motion_chain_logger = LaunchConfiguration('start_motion_chain_logger')
    gps_status_period = LaunchConfiguration('gps_status_period')
    motion_chain_log_path = LaunchConfiguration('motion_chain_log_path')
    motion_chain_sample_period = LaunchConfiguration('motion_chain_sample_period')
    motion_chain_duration = LaunchConfiguration('motion_chain_duration')
    gps_heading_yaw_offset = LaunchConfiguration('gps_heading_yaw_offset')
    gps_heading_filter_alpha = LaunchConfiguration('gps_heading_filter_alpha')
    gps_heading_max_yaw_rate = LaunchConfiguration('gps_heading_max_yaw_rate')
    gps_heading_covariance_floor = LaunchConfiguration('gps_heading_covariance_floor')
    gps_heading_invert = LaunchConfiguration('gps_heading_invert')
    gps_heading_baseline_min = LaunchConfiguration('gps_heading_baseline_min')
    gps_heading_baseline_max = LaunchConfiguration('gps_heading_baseline_max')
    gps_heading_baseline_soft_min = LaunchConfiguration('gps_heading_baseline_soft_min')
    gps_heading_baseline_soft_max = LaunchConfiguration('gps_heading_baseline_soft_max')
    gps_heading_dynamic_covariance = LaunchConfiguration('gps_heading_dynamic_covariance')
    gps_heading_reference_rate_gate = LaunchConfiguration('gps_heading_reference_rate_gate')
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
        'send_gga': False,
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
        DeclareLaunchArgument('rover_device', default_value='/dev/boxbot/gps_rover'),
        DeclareLaunchArgument(
            'moving_base_device',
            default_value='/dev/boxbot/gps_moving_base',
        ),
        DeclareLaunchArgument('rover_baudrate', default_value='115200'),
        DeclareLaunchArgument('moving_base_baudrate', default_value='115200'),
        DeclareLaunchArgument('navsat_fix_topic', default_value='/gps_rover/fix'),
        DeclareLaunchArgument('ntrip_fix_topic', default_value='/gps_moving_base/fix'),
        DeclareLaunchArgument('rover_rtcm_topic', default_value='/rtcm_rover_disabled'),
        DeclareLaunchArgument('moving_base_rtcm_topic', default_value='/rtcm'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/boxbot/lidar'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser'),
        DeclareLaunchArgument('lidar_inverted', default_value='false'),
        DeclareLaunchArgument('lidar_angle_compensate', default_value='true'),
        DeclareLaunchArgument('lidar_scan_mode', default_value='Standard'),
        DeclareLaunchArgument('start_ntrip', default_value='true'),
        DeclareLaunchArgument('start_gps_status', default_value='true'),
        DeclareLaunchArgument('start_motion_chain_logger', default_value='false'),
        DeclareLaunchArgument('gps_status_period', default_value='10.0'),
        DeclareLaunchArgument('motion_chain_log_path', default_value=''),
        DeclareLaunchArgument('motion_chain_sample_period', default_value='0.1'),
        DeclareLaunchArgument('motion_chain_duration', default_value='0.0'),
        DeclareLaunchArgument('gps_heading_yaw_offset', default_value='3.14159265359'),
        DeclareLaunchArgument('gps_heading_filter_alpha', default_value='1.0'),
        DeclareLaunchArgument('gps_heading_max_yaw_rate', default_value='10.0'),
        DeclareLaunchArgument('gps_heading_covariance_floor', default_value='0.030461742'),
        # With the rover antenna at the rear and moving-base antenna 40 cm
        # forward, the ublox heading has the same yaw sign as wheel odom.
        DeclareLaunchArgument('gps_heading_invert', default_value='false'),
        DeclareLaunchArgument('gps_heading_baseline_min', default_value='0.34'),
        DeclareLaunchArgument('gps_heading_baseline_max', default_value='0.46'),
        DeclareLaunchArgument('gps_heading_baseline_soft_min', default_value='0.25'),
        DeclareLaunchArgument('gps_heading_baseline_soft_max', default_value='0.55'),
        DeclareLaunchArgument('gps_heading_dynamic_covariance', default_value='true'),
        DeclareLaunchArgument('gps_heading_reference_rate_gate', default_value='false'),
        DeclareLaunchArgument('navsat_log_level', default_value='warn'),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py'
                ])
            ),
            condition=IfCondition(start_lidar),
            launch_arguments={
                'serial_port': lidar_serial_port,
                'serial_baudrate': lidar_serial_baudrate,
                'frame_id': lidar_frame_id,
                'inverted': lidar_inverted,
                'angle_compensate': lidar_angle_compensate,
                'scan_mode': lidar_scan_mode,
            }.items(),
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
                        ('fix', ntrip_fix_topic),
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
                    'uart1.baudrate': ParameterValue(
                        rover_baudrate,
                        value_type=int
                    ),
                },
            ],
            remappings=[
                ('fix', '/gps_rover/fix'),
                ('fix_velocity', '/gps_rover/fix_velocity'),
                ('navpvt', '/gps_rover/navpvt'),
                ('navheading', '/gps/heading'),
                ('navrelposned', '/gps/relposned'),
                ('navstatus', '/gps_rover/navstatus'),
                ('navcov', '/gps_rover/navcov'),
                ('nmea', '/gps/nmea'),
                ('rxmrtcm', '/gps_rover/rxmrtcm'),
                ('rtcm', rover_rtcm_topic),
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
                    'uart1.baudrate': ParameterValue(
                        moving_base_baudrate,
                        value_type=int
                    ),
                },
            ],
            remappings=[
                ('fix', '/gps_moving_base/fix'),
                ('fix_velocity', '/gps_moving_base/fix_velocity'),
                ('navpvt', '/gps_moving_base/navpvt'),
                ('navstatus', '/gps_moving_base/navstatus'),
                ('nmea', '/gps_moving_base/nmea'),
                ('rxmrtcm', '/gps_moving_base/rxmrtcm'),
                ('rtcm', moving_base_rtcm_topic),
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
            executable='motion_chain_logger.py',
            name='motion_chain_logger',
            output='screen',
            condition=IfCondition(start_motion_chain_logger),
            parameters=[{
                'output_path': motion_chain_log_path,
                'sample_period': ParameterValue(
                    motion_chain_sample_period,
                    value_type=float,
                ),
                'duration_seconds': ParameterValue(
                    motion_chain_duration,
                    value_type=float,
                ),
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
                'baseline_length_soft_min': ParameterValue(
                    gps_heading_baseline_soft_min,
                    value_type=float
                ),
                'baseline_length_soft_max': ParameterValue(
                    gps_heading_baseline_soft_max,
                    value_type=float
                ),
                'dynamic_yaw_covariance': ParameterValue(
                    gps_heading_dynamic_covariance,
                    value_type=bool
                ),
                'reference_imu_topic': '/imu/data',
                'gate_with_reference_yaw_rate': ParameterValue(
                    gps_heading_reference_rate_gate,
                    value_type=bool
                ),
                'require_fixed_baseline': False,
                'require_differential_baseline': False,
                'rejected_yaw_covariance': 1.0,
                'relposned_timeout_seconds': 2.0,
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
                ('gps/fix', navsat_fix_topic),
                ('imu', '/gps/heading_corrected'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
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
