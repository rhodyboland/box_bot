"""Dual X20P receivers and ROS fix conversion, without starting robot motion nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    actions = [
        DeclareLaunchArgument('gnss_params', default_value=PathJoinSubstitution(
            [FindPackageShare('box_bot'), 'config', 'zed_x20p_dual.yaml'])),
        DeclareLaunchArgument('moving_base_rtcm_topic', default_value='/rtcm'),
    ]
    for namespace in ('gps_rover', 'gps_moving_base'):
        remappings = [
            ('ubx_nav_pvt', 'navpvt'),
            ('ubx_nav_hp_pos_llh', 'hpposllh'),
            ('ubx_nav_cov', 'navcov'),
            ('ubx_nav_status', 'navstatus'),
            ('ubx_nav_sat', 'navsat'),
            ('ubx_rxm_cor', 'rxmcor'),
            ('ubx_sec_sig', 'secsig'),
            ('ubx_nav_rel_pos_ned', '/gps/relposned' if namespace == 'gps_rover'
             else 'relposned'),
        ]
        if namespace == 'gps_moving_base':
            remappings.append(('/ntrip_client/rtcm', LaunchConfiguration('moving_base_rtcm_topic')))
        actions.extend([
            Node(package='ublox_dgnss_node', executable='ublox_dgnss_node',
                 namespace=namespace, name='receiver', output='screen',
                 parameters=[LaunchConfiguration('gnss_params')], remappings=remappings),
            Node(package='box_bot', executable='gnss_navsat.py',
                 namespace=namespace, name='navsat', output='screen'),
        ])
    return LaunchDescription(actions)
