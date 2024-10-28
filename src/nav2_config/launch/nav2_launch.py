# launch/nav2_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the nav2_params.yaml file
    nav2_config_dir = os.path.join(
        get_package_share_directory('nav2_config'), 'config')
    nav2_params_file = os.path.join(nav2_config_dir, 'nav2_params.yaml')

    return LaunchDescription([
        # Bringup the map server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/map_server_launch.py']),
            launch_arguments={'params_file': nav2_params_file}.items(),
        ),

        # Bringup the AMCL localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/amcl_launch.py']),
            launch_arguments={'params_file': nav2_params_file}.items(),
        ),

        # Bringup the planner server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/planner_server_launch.py']),
            launch_arguments={'params_file': nav2_params_file}.items(),
        ),

        # Bringup the controller server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/controller_server_launch.py']),
            launch_arguments={'params_file': nav2_params_file}.items(),
        ),

        # Bringup the bt_navigator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/bt_navigator_launch.py']),
            launch_arguments={'params_file': nav2_params_file}.items(),
        ),

        # Bringup the recoveries server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                '/recoveries_server_launch.py']),
            launch_arguments={'params_file': nav2_params_file}.items(),
        ),

        # Launch RViz with Nav2 configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(nav2_config_dir, 'nav2.rviz')],
        ),
    ])
