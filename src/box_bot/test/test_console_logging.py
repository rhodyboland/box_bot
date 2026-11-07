"""Exercise real launch stdout/stderr routing without launching robot hardware."""
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('verbose', [False, True])
def test_console_filter_preserves_complete_launch_log(tmp_path, verbose):
    launch_path = Path(__file__).resolve().parents[1] / 'launch/box_bot_nav_gps.launch.py'
    child = r'''
import importlib.util
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, OpaqueFunction, SetLaunchConfiguration
spec = importlib.util.spec_from_file_location('boxbot_launch', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
# Construct the real description, but execute only a harmless dummy process.
module.generate_launch_description()
payload = """
import sys
print('[INFO] [1] [test_node]: info_disk_marker', flush=True)
print('[WARN] [1] [hoverboard_base_controller]: Velocity command timed out. Braking.', flush=True)
print('[WARN] [1] [test_node]: genuine_warning_marker', file=sys.stderr, flush=True)
print('[ERROR] [1] [test_node]: genuine_error_marker', file=sys.stderr, flush=True)
print('Traceback unstructured_marker', file=sys.stderr, flush=True)
"""
service = LaunchService()
service.include_launch_description(LaunchDescription([
    SetLaunchConfiguration('console_verbose', sys.argv[2]),
    OpaqueFunction(function=module.configure_console),
    ExecuteProcess(cmd=[sys.executable, '-u', '-c', payload], name='console_probe', output='screen'),
]))
sys.exit(service.run())
'''
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path))
    result = subprocess.run(
        [sys.executable, '-c', child, str(launch_path), str(verbose).lower()],
        env=env, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    screen = result.stdout + result.stderr
    files = list(tmp_path.rglob('launch.log'))
    assert files
    disk = '\n'.join(path.read_text() for path in files)
    for marker in ('info_disk_marker', 'Velocity command timed out. Braking.',
                   'genuine_warning_marker', 'genuine_error_marker', 'unstructured_marker'):
        assert marker in disk
    for marker in ('genuine_warning_marker', 'genuine_error_marker', 'unstructured_marker'):
        assert marker in screen
    assert ('info_disk_marker' in screen) is verbose
    assert ('Velocity command timed out. Braking.' in screen) is verbose
