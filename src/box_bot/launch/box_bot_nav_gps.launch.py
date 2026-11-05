from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gnss_params = LaunchConfiguration('gnss_params')
    ntrip_params = LaunchConfiguration('ntrip_params')
    ekf_params = LaunchConfiguration('ekf_params')
    nav2_params = LaunchConfiguration('nav2_params')
    navsat_fix_topic = LaunchConfiguration('navsat_fix_topic')
    ntrip_fix_topic = LaunchConfiguration('ntrip_fix_topic')
    moving_base_rtcm_topic = LaunchConfiguration('moving_base_rtcm_topic')
    start_lidar = LaunchConfiguration('start_lidar')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    lidar_angle_compensate = LaunchConfiguration('lidar_angle_compensate')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')
    start_realsense = LaunchConfiguration('start_realsense')
    realsense_camera_namespace = LaunchConfiguration('realsense_camera_namespace')
    realsense_camera_name = LaunchConfiguration('realsense_camera_name')
    realsense_serial_no = LaunchConfiguration('realsense_serial_no')
    realsense_depth_enable = LaunchConfiguration('realsense_depth_enable')
    realsense_color_enable = LaunchConfiguration('realsense_color_enable')
    realsense_depth_profile = LaunchConfiguration('realsense_depth_profile')
    realsense_color_profile = LaunchConfiguration('realsense_color_profile')
    realsense_clip_distance = LaunchConfiguration('realsense_clip_distance')
    realsense_emitter_enabled = LaunchConfiguration('realsense_emitter_enabled')
    realsense_initial_reset = LaunchConfiguration('realsense_initial_reset')
    realsense_align_depth_enable = LaunchConfiguration('realsense_align_depth_enable')
    realsense_sync_enable = LaunchConfiguration('realsense_sync_enable')
    realsense_filters_enable = LaunchConfiguration('realsense_filters_enable')
    realsense_pointcloud_enable = LaunchConfiguration('realsense_pointcloud_enable')
    realsense_pointcloud_stream_filter = LaunchConfiguration('realsense_pointcloud_stream_filter')
    realsense_start_delay = LaunchConfiguration('realsense_start_delay')
    realsense_lrs_log_level = LaunchConfiguration('realsense_lrs_log_level')
    realsense_obstacle_filter_enable = LaunchConfiguration('realsense_obstacle_filter_enable')
    realsense_obstacle_min_height = LaunchConfiguration('realsense_obstacle_min_height')
    realsense_obstacle_max_height = LaunchConfiguration('realsense_obstacle_max_height')
    realsense_obstacle_ground_height = LaunchConfiguration('realsense_obstacle_ground_height')
    realsense_obstacle_downsample_step = LaunchConfiguration('realsense_obstacle_downsample_step')
    realsense_obstacle_max_points = LaunchConfiguration('realsense_obstacle_max_points')
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
            'gnss_params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'zed_x20p_dual.yaml']
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
        DeclareLaunchArgument('navsat_fix_topic', default_value='/gps_rover/fix'),
        DeclareLaunchArgument('ntrip_fix_topic', default_value='/gps_moving_base/fix'),
        DeclareLaunchArgument('moving_base_rtcm_topic', default_value='/rtcm'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/boxbot/lidar'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser'),
        DeclareLaunchArgument('lidar_inverted', default_value='false'),
        DeclareLaunchArgument('lidar_angle_compensate', default_value='true'),
        DeclareLaunchArgument('lidar_scan_mode', default_value='Standard'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('realsense_camera_namespace', default_value='camera'),
        DeclareLaunchArgument('realsense_camera_name', default_value='camera'),
        DeclareLaunchArgument('realsense_serial_no', default_value=''),
        DeclareLaunchArgument('realsense_depth_enable', default_value='true'),
        DeclareLaunchArgument('realsense_color_enable', default_value='false'),
        DeclareLaunchArgument('realsense_depth_profile', default_value='640x480x15'),
        DeclareLaunchArgument('realsense_color_profile', default_value='640x480x15'),
        DeclareLaunchArgument('realsense_clip_distance', default_value='6.0'),
        # Daylight usually overwhelms the active IR projector; passive stereo is cleaner outside.
        DeclareLaunchArgument('realsense_emitter_enabled', default_value='0'),
        DeclareLaunchArgument('realsense_initial_reset', default_value='false'),
        DeclareLaunchArgument('realsense_align_depth_enable', default_value='false'),
        DeclareLaunchArgument('realsense_sync_enable', default_value='false'),
        DeclareLaunchArgument('realsense_filters_enable', default_value='false'),
        DeclareLaunchArgument('realsense_pointcloud_enable', default_value='true'),
        DeclareLaunchArgument('realsense_pointcloud_stream_filter', default_value='0'),
        DeclareLaunchArgument('realsense_start_delay', default_value='12.0'),
        DeclareLaunchArgument('realsense_lrs_log_level', default_value='ERROR'),
        DeclareLaunchArgument('realsense_obstacle_filter_enable', default_value='true'),
        DeclareLaunchArgument('realsense_obstacle_min_height', default_value='0.15'),
        DeclareLaunchArgument('realsense_obstacle_max_height', default_value='1.0'),
        DeclareLaunchArgument('realsense_obstacle_ground_height', default_value='-0.10'),
        DeclareLaunchArgument('realsense_obstacle_downsample_step', default_value='6'),
        DeclareLaunchArgument('realsense_obstacle_max_points', default_value='16000'),
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
            period=realsense_start_delay,
            condition=IfCondition(start_realsense),
            actions=[
                Node(
                    package='realsense2_camera',
                    executable='realsense2_camera_node',
                    namespace=realsense_camera_namespace,
                    name=realsense_camera_name,
                    output='screen',
                    additional_env={
                        'LRS_LOG_LEVEL': realsense_lrs_log_level,
                    },
                    parameters=[{
                        'camera_name': ParameterValue(
                            realsense_camera_name,
                            value_type=str
                        ),
                        'serial_no': ParameterValue(
                            realsense_serial_no,
                            value_type=str
                        ),
                        'enable_depth': ParameterValue(
                            realsense_depth_enable,
                            value_type=bool
                        ),
                        'enable_color': ParameterValue(
                            realsense_color_enable,
                            value_type=bool
                        ),
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'enable_gyro': False,
                        'enable_accel': False,
                        'publish_tf': False,
                        'tf_publish_rate': 0.0,
                        'depth_module.depth_profile': ParameterValue(
                            realsense_depth_profile,
                            value_type=str
                        ),
                        'rgb_camera.color_profile': ParameterValue(
                            realsense_color_profile,
                            value_type=str
                        ),
                        'depth_module.emitter_enabled': ParameterValue(
                            realsense_emitter_enabled,
                            value_type=int
                        ),
                        'clip_distance': ParameterValue(
                            realsense_clip_distance,
                            value_type=float
                        ),
                        'initial_reset': ParameterValue(
                            realsense_initial_reset,
                            value_type=bool
                        ),
                        'pointcloud.enable': ParameterValue(
                            realsense_pointcloud_enable,
                            value_type=bool
                        ),
                        'pointcloud.stream_filter': ParameterValue(
                            realsense_pointcloud_stream_filter,
                            value_type=int
                        ),
                        'pointcloud.allow_no_texture_points': True,
                        'pointcloud.ordered_pc': False,
                        'pointcloud.pointcloud_qos': 'SENSOR_DATA',
                        'align_depth.enable': ParameterValue(
                            realsense_align_depth_enable,
                            value_type=bool
                        ),
                        'enable_sync': ParameterValue(
                            realsense_sync_enable,
                            value_type=bool
                        ),
                        'spatial_filter.enable': ParameterValue(
                            realsense_filters_enable,
                            value_type=bool
                        ),
                        'temporal_filter.enable': ParameterValue(
                            realsense_filters_enable,
                            value_type=bool
                        ),
                        'decimation_filter.enable': ParameterValue(
                            realsense_filters_enable,
                            value_type=bool
                        ),
                    }],
                    arguments=['--ros-args', '--log-level', log_level],
                ),
            ],
        ),
        Node(
            package='box_bot',
            executable='imu_tilt_cloud_filter.py',
            name='imu_tilt_cloud_filter',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_realsense, "'.lower() == 'true' and '",
                realsense_obstacle_filter_enable, "'.lower() == 'true'",
            ])),
            parameters=[{
                'input_cloud_topic': '/camera/depth/color/points',
                'output_cloud_topic': '/camera/depth/obstacles',
                'imu_topic': '/imu/data',
                'target_frame': 'base_link',
                'ground_height': ParameterValue(
                    realsense_obstacle_ground_height,
                    value_type=float,
                ),
                'min_obstacle_height': ParameterValue(
                    realsense_obstacle_min_height,
                    value_type=float,
                ),
                'max_obstacle_height': ParameterValue(
                    realsense_obstacle_max_height,
                    value_type=float,
                ),
                'downsample_step': ParameterValue(
                    realsense_obstacle_downsample_step,
                    value_type=int,
                ),
                'max_points': ParameterValue(
                    realsense_obstacle_max_points,
                    value_type=int,
                ),
            }],
            arguments=['--ros-args', '--log-level', log_level],
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('box_bot'), 'launch', 'gnss.launch.py',
            ])),
            launch_arguments={
                'gnss_params': gnss_params,
                'moving_base_rtcm_topic': moving_base_rtcm_topic,
            }.items(),
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
                ('imu/raw', '/gps/heading'),
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
                ('odometry/filtered', '/odometry/local'),
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
