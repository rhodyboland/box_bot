"""Allowlisted navigation tuning and comment-preserving source-config updates."""
import copy
import fcntl
import hashlib
import math
import os
from pathlib import Path
import stat
import tempfile

import yaml
from yaml.nodes import MappingNode, SequenceNode, ScalarNode


SETTINGS = {
    'cruise_speed': dict(label='Cruise speed limit', unit='m/s', min=.05, max=1., step=.05),
    'turn_rate': dict(label='Maximum turn rate', unit='rad/s', min=.1, max=1., step=.05),
    'linear_acceleration': dict(label='Drive acceleration / braking', unit='m/s²', min=.05, max=1., step=.05),
    'angular_acceleration': dict(label='Turn acceleration / braking', unit='rad/s²', min=.1, max=1., step=.1),
    'goal_radius': dict(label='Waypoint acceptance radius', unit='m', min=.1, max=2., step=.05),
    'heading_tolerance': dict(label='Waypoint heading tolerance', unit='°', min=5., max=180., step=1.),
    'progress_timeout': dict(label='No-progress timeout', unit='s', min=5., max=120., step=5.),
}
# A binding is (node, ROS parameter, array index or None, value multiplier).
BINDINGS = {
    'cruise_speed': [('controller_server', 'FollowPath.max_vel_x', None, 1),
                     ('controller_server', 'FollowPath.max_speed_xy', None, 1),
                     ('velocity_smoother', 'max_velocity', 0, 1),
                     ('velocity_smoother', 'min_velocity', 0, -1)],
    'turn_rate': [('controller_server', 'FollowPath.max_vel_theta', None, 1),
                  ('velocity_smoother', 'max_velocity', 2, 1),
                  ('velocity_smoother', 'min_velocity', 2, -1)],
    'linear_acceleration': [('controller_server', 'FollowPath.acc_lim_x', None, 1),
                            ('controller_server', 'FollowPath.decel_lim_x', None, -1),
                            ('velocity_smoother', 'max_accel', 0, 1),
                            ('velocity_smoother', 'max_decel', 0, -1)],
    'angular_acceleration': [('controller_server', 'FollowPath.acc_lim_theta', None, 1),
                             ('controller_server', 'FollowPath.decel_lim_theta', None, -1),
                             ('velocity_smoother', 'max_accel', 2, 1),
                             ('velocity_smoother', 'max_decel', 2, -1)],
    'goal_radius': [('controller_server', 'general_goal_checker.xy_goal_tolerance', None, 1)],
    'heading_tolerance': [('controller_server', 'general_goal_checker.yaw_goal_tolerance', None, math.pi/180)],
    'progress_timeout': [('controller_server', 'progress_checker.movement_time_allowance', None, 1)],
}
PARAMETERS = {}
for bindings in BINDINGS.values():
    for node, name, _, _ in bindings:
        PARAMETERS.setdefault(node, [])
        if name not in PARAMETERS[node]:
            PARAMETERS[node].append(name)


def validate_tuning(payload):
    if (not isinstance(payload, dict) or payload.get('mode') not in ('try', 'save') or
            set(payload)-{'mode', 'values', 'revision'}):
        raise ValueError('Expected tuning mode try or save and values')
    values = payload.get('values')
    if not isinstance(values, dict) or set(values) != set(SETTINGS):
        raise ValueError('Supply all seven tuning settings')
    checked = {}
    for key, setting in SETTINGS.items():
        value = values[key]
        if (type(value) not in (int, float) or not math.isfinite(value) or
                not setting['min'] <= value <= setting['max']):
            raise ValueError(f'{setting["label"]} must be {setting["min"]}–{setting["max"]} {setting["unit"]}')
        checked[key] = float(value)
    if payload['mode'] == 'save' and not isinstance(payload.get('revision'), str):
        raise ValueError('Refresh saved settings before making permanent')
    return dict(mode=payload['mode'], values=checked, revision=payload.get('revision'))


def parameter_values(values, current):
    result = copy.deepcopy(current)
    for key, bindings in BINDINGS.items():
        for node, name, index, multiplier in bindings:
            value = values[key]*multiplier
            if index is None:
                result[node][name] = value
            else:
                result[node][name][index] = value
    return result


def displayed_values(parameters):
    values = {}
    for key, bindings in BINDINGS.items():
        node, name, index, multiplier = bindings[0]
        value = parameters[node][name]
        if index is not None:
            value = value[index]
        if type(value) not in (float, int) or not math.isfinite(value):
            raise ValueError(f'Live {key} is unavailable')
        values[key] = value/multiplier
    return values


def matches(actual, expected):
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(key in actual and matches(actual[key], value)
                                                for key, value in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(
            matches(left, right) for left, right in zip(actual, expected))
    return type(actual) in (int, float) and math.isfinite(actual) and math.isclose(actual, expected, abs_tol=1e-6)


def config_node(document, node, name, index):
    # Accept both nested YAML and ROS's dotted parameter spelling.
    current = document
    keys = [node, 'ros__parameters']
    for key in keys:
        current = next(value for label, value in current.value if label.value == key)
    pieces = name.split('.')
    while pieces:
        dotted = '.'.join(pieces)
        entries = {key.value: value for key, value in current.value}
        if dotted in entries:
            current = entries[dotted]
            break
        current = entries[pieces.pop(0)]
        if not isinstance(current, MappingNode) and pieces:
            raise ValueError('Expected a parameter mapping')
    if index is not None:
        if not isinstance(current, SequenceNode):
            raise ValueError('Expected a parameter array')
        current = current.value[index]
    if not isinstance(current, ScalarNode):
        raise ValueError('Expected a scalar parameter')
    return current


def read_config(path):
    text = Path(path).read_text()
    tree = yaml.compose(text)
    parameters = {node: {} for node in PARAMETERS}
    for node, names in PARAMETERS.items():
        for name in names:
            scalar = config_node(tree, node, name, None) if name not in ('max_velocity', 'min_velocity', 'max_accel', 'max_decel') else None
            if scalar is not None:
                parameters[node][name] = yaml.safe_load(text[scalar.start_mark.index:scalar.end_mark.index])
            else:
                parameters[node][name] = [yaml.safe_load(text[item.start_mark.index:item.end_mark.index])
                    for item in [config_node(tree, node, name, i) for i in range(3)]]
    return dict(values=displayed_values(parameters), revision=hashlib.sha256(text.encode()).hexdigest(), path=str(path))


def save_config(path, values, revision):
    path = Path(path).resolve(strict=True)
    if any(part == 'install' or part.startswith('install-') for part in path.parts) or str(path).startswith('/opt/ros/'):
        raise ValueError('Select the source YAML config instead of an installed copy')
    temporary = None
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        fcntl.flock(directory, fcntl.LOCK_EX)
        original = path.read_text()
        if hashlib.sha256(original.encode()).hexdigest() != revision:
            raise ValueError('Config changed on disk. Reload tuning before saving')
        if any(isinstance(event, yaml.AliasEvent) for event in yaml.parse(original)):
            raise ValueError('Saving tuning requires a source config without YAML aliases')
        tree = yaml.compose(original)
        replacements = []
        for key, bindings in BINDINGS.items():
            for node, name, index, multiplier in bindings:
                scalar = config_node(tree, node, name, index)
                start, end = scalar.start_mark.index, scalar.end_mark.index
                value = values[key]*multiplier
                if not matches(yaml.safe_load(original[start:end]), value):
                    replacements.append((start, end, repr(value)))
        if not replacements:
            return read_config(path)
        updated = original
        for start, end, value in sorted(replacements, reverse=True):
            updated = updated[:start]+value+updated[end:]
        yaml.safe_load(updated)
        descriptor, temporary = tempfile.mkstemp(prefix='.'+path.name+'-', dir=path.parent)
        with os.fdopen(descriptor, 'w') as output:
            os.fchmod(output.fileno(), stat.S_IMODE(path.stat().st_mode))
            output.write(updated)
            output.flush()
            os.fsync(output.fileno())
        if path.read_text() != original:
            raise ValueError('Config changed while saving. Reload tuning and retry')
        os.replace(temporary, path)
        temporary = None
        os.fsync(directory)
    finally:
        if temporary is not None:
            os.unlink(temporary)
        os.close(directory)
    return read_config(path)
