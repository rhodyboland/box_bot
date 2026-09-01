from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ---- Config files (override on CLI if you like) ----
    ekf_yaml   = LaunchConfiguration('ekf_yaml')
    navsat_yaml = LaunchConfiguration('navsat_yaml')
    nav2_yaml  = LaunchConfiguration('nav2_yaml')

    return LaunchDescription([
        # ====== Args (defaults to installed package share) ======
        DeclareLaunchArgument(
            'ekf_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'ekf_gps.yaml']
            )
        ),
        DeclareLaunchArgument(
            'navsat_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'navsat.yaml']
            )
        ),
        DeclareLaunchArgument(
            'nav2_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'config', 'nav2_minimal.yaml']
            )
        ),

        # ====== Sensors / low-level control ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('box_bot'), 'launch', 'bno085.launch.py']))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('hoverboard_control'), 'launch', 'hoverboard.launch.py']))
        ),

        # GNSS NMEA TCP client (Reach)
        Node(
            package='nmea_navsat_driver',
            executable='nmea_tcpclient_driver',
            name='nmea_tcpclient_driver',
            parameters=[{ 'ip': '192.168.0.109', 'port': 9001, 'frame_id': 'gps' }],
            remappings=[
                ('fix', '/gps/fix'),          # OK
                ('vel', '/gps/vel'),
                ('time_reference', '/gps/time_reference'),
                # DO NOT remap anything to /gps/filtered here
            ],
            ),

        # ====== robot_localization + navsat_transform ======
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_odom', output='screen',
            parameters=[ekf_yaml],
            remappings=[('/odometry/filtered', '/odometry/filtered')]
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_map', output='screen',
            parameters=[ekf_yaml]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[{
                'use_odometry_yaw': True,
                'wait_for_datum': False,
                'zero_altitude': True,
                'publish_filtered_gps': False,   # we’re using /odometry/gps path
                'broadcast_utm_transform': True,
                'delay': 3.0                     # give odom time to appear
            }],
            remappings=[
                # Remap the topics navsat_transform expects by default
                ('/gps/fix', '/gps/fix'),                         # NavSatFix in
                ('/odometry/filtered', '/hoverboard_base_controller/odom'),  # <-- key: use your odom
                ('/imu/data', '/imu/data'),                       # ignored with use_odometry_yaw=True
                ('/odometry/gps', '/odometry/gps'),               # Odometry out (unchanged)
            ],
        ),

        # ====== Nav2 core (no map, rolling costmaps) ======
        Node(
            package='nav2_planner', executable='planner_server',
            output='screen', parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            output='screen', parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_smoother', executable='smoother_server',
            output='screen', parameters=[nav2_yaml]
        ),
        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('box_bot'), 'config', 'nav2_minimal.yaml'  # your params file
                ]),
                {'use_sim_time': False}
            ],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            output='screen', parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            output='screen', parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),

    ])
