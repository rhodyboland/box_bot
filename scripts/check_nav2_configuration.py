#!/usr/bin/env python3
"""Hardware-free lifecycle configuration smoke test. No nodes are activated."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile

import yaml

import rclpy
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python


def main():
    parser = argparse.ArgumentParser(description='Configure Nav2 servers without activation or robot hardware.')
    parser.add_argument('--params', type=Path, default=Path(__file__).resolve().parents[1] / 'src/box_bot/config/nav2_minimal.yaml')
    parser.add_argument('--domain-id', type=int, default=138)
    parser.add_argument('--composition', action='store_true')
    parser.add_argument('--audit-parameters', action='store_true')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

    packages = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    namespace = f'boxbot_config_test_{os.getpid()}'
    temporary = tempfile.TemporaryDirectory(prefix='boxbot-nav2-')
    log_dir = Path(temporary.name)
    params = log_dir / 'params.yaml'
    params.write_text(yaml.safe_dump({namespace: yaml.safe_load(args.params.read_text())}))
    processes = []
    files = []
    rclpy.init()
    node = rclpy.create_node('boxbot_configuration_test')
    try:
        def start(command, name):
            log = (log_dir / f'{name}.log').open('w')
            files.append(log)
            processes.append(subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

        if args.composition:
            start(['/opt/ros/lyrical/lib/rclcpp_components/component_container_isolated',
                   '--ros-args', '--params-file', str(params), '-r', '__node:=nav2_container',
                   '-r', f'__ns:=/{namespace}'], 'components')
            start(['ros2', 'launch', 'nav2_launcher', 'nav2_launch.py',
                   f'namespace:={namespace}', 'use_composition:=true',
                   'autostart:=false', f'params_file:={args.params.resolve()}'], 'launch')
        else:
            for package, executable in packages:
                start([f'/opt/ros/lyrical/lib/{package}/{executable}', '--ros-args',
                       '--params-file', str(params), '-r', f'__ns:=/{namespace}'], executable)

        def failure(name, reason):
            logs = ''.join(path.read_text() for path in log_dir.glob('*.log'))
            return RuntimeError(f'{name}: {reason}\n{logs}')

        for _, name in packages:
            client = node.create_client(ChangeState, f'/{namespace}/{name}/change_state')
            if not client.wait_for_service(timeout_sec=15):
                raise failure(name, 'lifecycle service not available')
            request = ChangeState.Request()
            request.transition.id = 1  # Configure only. Never activate a server.
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20)
            if not future.done() or not future.result().success:
                raise failure(name, 'configure failed')
            print(f'{name}: configure passed', flush=True)
            if args.audit_parameters:
                listing = node.create_client(ListParameters, f'/{namespace}/{name}/list_parameters')
                if not listing.wait_for_service(timeout_sec=5):
                    raise failure(name, 'parameter service unavailable')
                query = listing.call_async(ListParameters.Request())
                rclpy.spin_until_future_complete(node, query, timeout_sec=5)
                if not query.done() or query.result() is None:
                    raise failure(name, 'parameter listing failed')
                declared = set(query.result().result.names)

                # Controller plugins declare some settings only at activation.
                # Audit server-level overrides here; activation is deliberately excluded.
                configured = yaml.safe_load(args.params.read_text())[name]['ros__parameters']
                server_keys = {key for key, value in configured.items()
                               if not isinstance(value, dict) and '.' not in key}
                unknown = server_keys - declared
                if unknown:
                    raise failure(name, f'undeclared server overrides: {sorted(unknown)}')
        if args.audit_parameters:
            configuration = yaml.safe_load(args.params.read_text())
            for costmap in ('local_costmap', 'global_costmap'):
                expected = configuration[costmap][costmap]['ros__parameters']
                keys = ['plugins', 'global_frame', 'robot_base_frame', 'rolling_window',
                        'width', 'height', 'resolution', 'robot_radius']
                keys = [key for key in keys if key in expected]
                expected_values = {key: expected[key] for key in keys}
                for layer in expected['plugins']:
                    settings = expected.get(layer, {})
                    if 'observation_sources' in settings:
                        expected_values[f'{layer}.observation_sources'] = settings['observation_sources']
                        for source in settings['observation_sources'].split():
                            for key in ('topic', 'marking', 'clearing', 'observation_persistence'):
                                if key in settings[source]:
                                    expected_values[f'{layer}.{source}.{key}'] = settings[source][key]
                    if 'num_threads' in settings:
                        expected_values[f'{layer}.num_threads'] = settings['num_threads']
                keys = list(expected_values)
                client = node.create_client(
                    GetParameters, f'/{namespace}/{costmap}/{costmap}/get_parameters')
                if not client.wait_for_service(timeout_sec=5):
                    raise failure(costmap, 'child parameter service unavailable')
                request = GetParameters.Request()
                request.names = keys
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or future.result() is None:
                    raise failure(costmap, 'child parameter request timed out')
                actual = dict(zip(keys, map(parameter_value_to_python, future.result().values)))
                mismatches = {key: (expected_values[key], actual[key]) for key in keys
                              if expected_values[key] != actual[key]}
                if mismatches:
                    raise failure(costmap, f'child parameters differ (expected, actual): {mismatches}')
                print(f'{costmap}: child parameters passed', flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in files:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        temporary.cleanup()


if __name__ == "__main__":
    main()
