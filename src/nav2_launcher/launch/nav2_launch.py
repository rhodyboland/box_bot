"""Boxbot's seven-server Nav2 bringup using Lyrical components.

Keep this subset because the stock bringup also starts route, docking, following,
and collision-monitor servers that Boxbot does not configure.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
from nav2_launcher.recoverable_components import RecoverableLoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')
    configured_params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=namespace,
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'default_nav_to_pose_bt_xml': LaunchConfiguration('default_nav_to_pose_bt_xml'),
        },
        convert_types=True,
    ), allow_substs=True)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # Both velocity producers pass through the same smoother before /cmd_vel.
    servers = [
        ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', 'nav2_smoother::SmootherServer', []),
        ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower',
         'nav2_waypoint_follower::WaypointFollower', []),
        ('nav2_velocity_smoother', 'velocity_smoother',
         'nav2_velocity_smoother::VelocitySmoother',
         [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]
    lifecycle_parameters = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
        'node_names': [server[1] for server in servers],
        'bond_timeout': ParameterValue(LaunchConfiguration('bond_timeout'), value_type=float),
    }
    nodes = [Node(
        package=package, executable=name, name=name,
        parameters=[configured_params], remappings=tf_remaps + remaps,
        output='screen', sigterm_timeout='15', respawn=LaunchConfiguration('use_respawn'), respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', log_level],
    ) for package, name, _, remaps in servers if name != "waypoint_follower"]
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', parameters=[lifecycle_parameters],
        output='screen', sigterm_timeout='15', arguments=['--ros-args', '--log-level', log_level],
    ))
    components = [ComposableNode(
        package=package, plugin=plugin, name=name, namespace=namespace,
        parameters=[configured_params], remappings=tf_remaps + remaps,
    ) for package, name, plugin, remaps in servers if name != "waypoint_follower"]
    components.append(ComposableNode(
        package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation', namespace=namespace,
        parameters=[lifecycle_parameters],
    ))
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('bond_timeout', default_value='10.0',
                              description='Nav2 lifecycle bond timeout in seconds'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('nav2_launcher'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('default_nav_to_pose_bt_xml', default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bt_navigator'), 'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'])),
        # The patched waypoint executable keeps its result callback nonblocking.
        # Run it separately in both modes so the shared library uses that fix.
        GroupAction(actions=[PushROSNamespace(namespace), Node(
            package='nav2_launcher', executable='boxbot_waypoint_follower', name='waypoint_follower',
            parameters=[configured_params], remappings=tf_remaps,
            output='screen', sigterm_timeout='15',
            arguments=['--ros-args', '--log-level', log_level])]),
        GroupAction(condition=UnlessCondition(composition), actions=[
            PushROSNamespace(namespace), *nodes]),
        RecoverableLoadComposableNodes(
            condition=IfCondition(composition),
            target_container=[namespace, '/', LaunchConfiguration('container_name')],
            composable_node_descriptions=components),
    ])
