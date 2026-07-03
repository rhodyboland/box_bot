from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=['config/nav2_minimal.yaml']),
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=['config/nav2_minimal.yaml']),
        Node(package='nav2_smoother', executable='smoother_server',
             output='screen', parameters=['config/nav2_minimal.yaml']),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             output='screen', parameters=['config/nav2_minimal.yaml']),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             output='screen', parameters=['config/nav2_minimal.yaml']),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False},
                         {'autostart': True},
                         {'node_names': ['controller_server','planner_server',
                                         'smoother_server','bt_navigator',
                                         'waypoint_follower']}]),
    ])
