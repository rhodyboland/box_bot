from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_magnetometer_driver',
            executable='magnetometer_node',
            name='magnetometer_node',
            output='screen',
            parameters=[]
        )
    ])
