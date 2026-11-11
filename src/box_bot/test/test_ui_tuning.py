"""Runtime tuning, rollback, source persistence and HTTP authorization."""
import copy
import json
import math
from pathlib import Path
import stat
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tuning_settings import (SETTINGS, PARAMETERS, parameter_values, read_config,
                             save_config, validate_tuning)


@pytest.fixture
def config(tmp_path):
    path = tmp_path/'nav2.yaml'
    path.write_text((Path(__file__).resolve().parents[1]/'config/nav2_minimal.yaml').read_text())
    return path


@pytest.mark.parametrize('key,value', [('cruise_speed', True), ('turn_rate', math.nan),
                                      ('goal_radius', 0), ('heading_tolerance', 181),
                                      ('progress_timeout', '30')])
def test_tuning_validation(config, key, value):
    values = read_config(config)['values']
    values[key] = value
    with pytest.raises(ValueError):
        validate_tuning(dict(mode='try', values=values))


def test_comment_preserving_atomic_save_and_conflict(config):
    original = config.read_text()
    config.chmod(0o640)
    saved = read_config(config)
    values = dict(saved['values'], cruise_speed=.6, turn_rate=.9, goal_radius=.4, heading_tolerance=90.)
    result = save_config(config, values, saved['revision'])
    document = yaml.safe_load(config.read_text())
    params = document['controller_server']['ros__parameters']
    smooth = document['velocity_smoother']['ros__parameters']
    assert params['FollowPath']['max_vel_x'] == .6
    assert smooth['min_velocity'] == [-.6, 0., -.9]
    assert params['general_goal_checker']['yaw_goal_tolerance'] == pytest.approx(math.pi/2)
    assert stat.S_IMODE(config.stat().st_mode) == 0o640
    assert result['values'] == pytest.approx(values)
    assert [line for line in config.read_text().splitlines() if '#' in line] == [
        line for line in original.splitlines() if '#' in line]
    current = config.read_text()+'\n# external edit\n'
    config.write_text(current)
    with pytest.raises(ValueError, match='changed'):
        save_config(config, values, result['revision'])
    assert config.read_text() == current


def test_failed_replace_keeps_original_and_cleans_temporary(config, monkeypatch):
    saved = read_config(config)
    original = config.read_bytes()

    def fail(*args):
        raise OSError('Simulated disk failure')

    monkeypatch.setattr('tuning_settings.os.replace', fail)
    with pytest.raises(OSError):
        save_config(config, dict(saved['values'], cruise_speed=.7), saved['revision'])
    assert config.read_bytes() == original
    assert list(config.parent.iterdir()) == [config]


def test_installed_copy_cannot_be_mistaken_for_source(config):
    path = config.parent/'install-lyrical/nav2.yaml'
    path.parent.mkdir()
    path.write_bytes(config.read_bytes())
    saved = read_config(path)
    with pytest.raises(ValueError, match='source YAML'):
        save_config(path, saved['values'], saved['revision'])


def test_runtime_http_trial_save_rollback_and_motion_gate(config):
    from http.client import HTTPConnection
    import threading
    import time
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rcl_interfaces.msg import SetParametersResult
    from nav_msgs.msg import Odometry
    from boxbot_ui import BoxbotUI

    rclpy.init(domain_id=174, args=['--ros-args', '-p', 'port:=0', '-p', 'host:=127.0.0.1',
                                   '-p', 'tuning_config_file:='+str(config)])
    ui = BoxbotUI()
    document = yaml.safe_load(config.read_text())
    nodes = {name: rclpy.create_node(name) for name in PARAMETERS}
    flags = dict(reject=False, delay=False, vx=0.)

    def initial(name, parameter):
        value = document[name]['ros__parameters']
        for part in parameter.split('.'):
            value = value[part]
        return value

    def accept(name, parameters):
        if flags['reject'] and name == 'velocity_smoother':
            if any(p.name == 'max_velocity' and p.value[0] == .9 for p in parameters):
                return SetParametersResult(successful=False, reason='Simulated smoother rejection')
        if flags['delay'] and name == 'controller_server':
            flags['delay'] = False
            time.sleep(3.6)  # The change completes, but its reply exceeds our deadline.
        return SetParametersResult(successful=True)

    for name, node in nodes.items():
        for parameter in PARAMETERS[name]:
            node.declare_parameter(parameter, initial(name, parameter))
        node.add_on_set_parameters_callback(lambda parameters, key=name: accept(key, parameters))
    publisher = nodes['velocity_smoother'].create_publisher(Odometry, '/hoverboard_base_controller/odom', 10)

    def wheel():
        message = Odometry()
        message.header.stamp = ui.get_clock().now().to_msg()
        message.twist.twist.linear.x = flags['vx']
        publisher.publish(message)

    timer = nodes['velocity_smoother'].create_timer(.1, wheel)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in [ui, *nodes.values()]:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def http(method, route, body=None, token=None):
        connection = HTTPConnection('127.0.0.1', ui.http.server_port, timeout=5)
        headers = {'Content-Type': 'application/json'}
        if token:
            headers['X-Boxbot-Token'] = token
        connection.request(method, route, body=json.dumps(body) if body is not None else None, headers=headers)
        response = connection.getresponse()
        code, result = response.status, json.loads(response.read())
        connection.close()
        return code, result

    def state():
        return http('GET', '/api/state')[1]

    def until(predicate, seconds=10):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            if predicate():
                return
            time.sleep(.05)
        raise AssertionError(state().get('tuning'))

    try:
        until(lambda: state().get('tuning', {}).get('can_apply'))
        token = state()['csrf']
        original = config.read_bytes()
        values = dict(state()['tuning']['values'], cruise_speed=.6, turn_rate=.9, goal_radius=.4,
                      linear_acceleration=.3, angular_acceleration=.9, heading_tolerance=90., progress_timeout=40.)
        assert http('POST', '/api/tuning', dict(mode='try', values=values))[0] == 403
        assert http('POST', '/api/tuning', dict(mode='try', values={}), token)[0] == 400
        assert http('POST', '/api/tuning', dict(mode='try', values=values), token)[0] == 202
        until(lambda: not state()['tuning']['busy'] and state()['tuning']['values']['cruise_speed'] == .6)
        assert config.read_bytes() == original
        assert list(nodes['velocity_smoother'].get_parameter('min_velocity').value) == [-.6, 0., -.9]
        assert ui.recovery.speed == .6
        live = state()['tuning']
        assert http('POST', '/api/tuning', dict(mode='save', values=live['values'], revision=live['saved']['revision']), token)[0] == 202
        until(lambda: not state()['tuning']['busy'] and state()['tuning']['saved']['values']['cruise_speed'] == .6)
        permanent = config.read_bytes()
        flags['reject'] = True
        assert http('POST', '/api/tuning', dict(mode='try', values=dict(values, cruise_speed=.9)), token)[0] == 202
        until(lambda: not state()['tuning']['busy'] and 'rejection' in state()['tuning']['error'])
        assert nodes['controller_server'].get_parameter('FollowPath.max_vel_x').value == .6
        assert nodes['velocity_smoother'].get_parameter('max_velocity').value[0] == .6
        assert state()['tuning']['block_navigation'] is False
        assert config.read_bytes() == permanent
        flags['reject'] = False
        flags['delay'] = True
        assert http('POST', '/api/tuning', dict(mode='try', values=dict(values, cruise_speed=.7)), token)[0] == 202
        until(lambda: not state()['tuning']['busy'] and 'timed out' in state()['tuning']['error'])
        assert nodes['controller_server'].get_parameter('FollowPath.max_vel_x').value == .6
        assert state()['tuning']['block_navigation'] is False
        flags['vx'] = .2
        until(lambda: 'Stop the wheels' in state()['tuning']['blocked_reason'])
        assert http('POST', '/api/tuning', dict(mode='try', values=values), token)[0] == 409
        flags['vx'] = 0.
        until(lambda: state()['tuning']['can_apply'])
        live = state()['tuning']
        config.write_text(config.read_text()+'\n# Concurrent source edit\n')
        assert http('POST', '/api/tuning', dict(mode='save', values=live['values'], revision=live['saved']['revision']), token)[0] == 202
        until(lambda: not state()['tuning']['busy'] and 'changed' in state()['tuning']['error'])
        assert config.read_text().endswith('# Concurrent source edit\n')
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        ui.close()
        for node in [ui, *nodes.values()]:
            node.destroy_node()
        rclpy.shutdown()
