"""Recovery policy and full-resolution stored-costmap checks without hardware."""
from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import traction_recovery as recovery_module
from traction_recovery import TractionRecovery, corridor_clear


def grid():
    message = OccupancyGrid()
    message.info.width = message.info.height = 60
    message.info.resolution = .1
    message.info.origin.position.x = message.info.origin.position.y = -3.
    message.info.origin.orientation.w = 1.
    message.data = [0]*3600
    return message


@pytest.mark.parametrize('direction', [-1, 1])
def test_swept_clearance_obstacles_unknown_and_boundaries(direction):
    message = grid()
    assert corridor_clear(message, (0, 0, 0), direction)
    cell = 30*60 + (36 if direction == 1 else 23)
    for value in (100, 65, -1):
        message.data[cell] = value
        assert not corridor_clear(message, (0, 0, 0), direction)
        assert corridor_clear(message, (0, 0, 0), -direction)
    assert not corridor_clear(message, (2.9, 0, 0), direction)
    message.data = []
    assert not corridor_clear(message, (0, 0, 0), direction)


def test_swept_clearance_respects_rotated_grid():
    import math
    message = grid()
    message.info.origin.position.x = 3.
    message.info.origin.orientation.z = math.sin(math.pi/4)
    message.info.origin.orientation.w = math.cos(math.pi/4)
    assert corridor_clear(message, (0, 0, 0), 1)
    message.data[23*60+30] = 100
    assert not corridor_clear(message, (0, 0, 0), 1)
    assert corridor_clear(message, (0, 0, 0), -1)


@pytest.fixture
def policy():
    clock = [10.]
    enabled = [True]
    node = Mock()
    node.get_parameter.side_effect = lambda name: SimpleNamespace(
        value=enabled[0] if name == 'traction_recovery_enabled' else 2.)
    node.create_client.side_effect = lambda *args: Mock(service_is_ready=lambda: True)
    clearance = Mock(return_value=[-1, 1])
    canceled = Mock()
    with patch.object(recovery_module, 'ActionClient') as client, patch.object(
            recovery_module, 'time', SimpleNamespace(monotonic=lambda: clock[0])):
        client.side_effect = lambda *args: Mock(server_is_ready=lambda: True)
        recovery = TractionRecovery(node, clearance, canceled)
        recovery.pause.call_async.return_value = Future()
        recovery.resume.call_async.return_value = Future()
        for action in recovery.actions.values():
            action.send_goal_async.return_value = Future()
        yield recovery, clock, enabled, clearance, canceled


def traction(state='suspected_forward_slip', stopped=False):
    return dict(state=state, metrics=dict(wheel_usable=True, gyro_usable=True,
                wheel_vx=0. if stopped else .4, wheel_wz=0., gyro_wz=0.,
                command_vx=0. if stopped else .4, command_wz=0.))


def trigger(policy):
    recovery, clock, *_ = policy
    recovery.tick(traction(), True)
    clock[0] += 1.9
    recovery.tick(traction('suspected_turn_slip'), True)
    recovery.pause.call_async.assert_not_called()
    clock[0] += .2
    recovery.tick(traction(), True)


def settle(policy):
    recovery, clock, *_ = policy
    trigger(policy)
    assert recovery.phase == 'pausing'
    recovery.pause.call_async.return_value.set_result(Trigger.Response(success=True))
    recovery.follow_path_active = False
    recovery.follow_path_received = clock[0]
    recovery.tick(traction(stopped=True), True)
    clock[0] += .4
    recovery.tick(traction(stopped=True), True)


def moving(policy):
    recovery, *_ = policy
    settle(policy)
    assert recovery.phase == 'requesting'
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    recovery.actions[recovery.direction].send_goal_async.return_value.set_result(handle)
    assert recovery.phase == 'moving'
    return handle


def test_delay_pause_stop_collision_checks_resume_and_cooldown(policy):
    recovery, clock, *_ = policy
    handle = moving(policy)
    goal = recovery.actions[-1].send_goal_async.call_args.args[0]
    assert goal.target.x == pytest.approx(-.3)
    assert goal.speed == pytest.approx(.4)
    assert goal.disable_collision_checks is False
    handle.get_result_async.return_value.set_result(SimpleNamespace(
        status=4, result=SimpleNamespace(error_code=0, error_msg='')))
    assert recovery.phase == 'resuming'
    recovery.resume.call_async.return_value.set_result(Trigger.Response(success=True))
    assert recovery.phase == 'idle'
    recovery.tick(traction(), True)
    assert recovery.confirmed_since is None
    assert recovery.cooldown_until > clock[0]


def test_forward_fallback_and_blocked_corridors(policy):
    recovery, _, _, clearance, canceled = policy
    clearance.return_value = [1]
    settle(policy)
    assert recovery.direction == 1
    assert recovery.actions[1].send_goal_async.call_args.args[0].target.x > 0
    recovery.actions[1].send_goal_async.return_value.set_result(Mock(accepted=False))
    assert canceled.called
    assert recovery.phase == 'failed'


def test_no_clear_corridor_stops_navigation_without_motion(policy):
    recovery, _, _, clearance, canceled = policy
    clearance.return_value = []
    settle(policy)
    assert canceled.called
    for client in recovery.actions.values():
        client.send_goal_async.assert_not_called()


def test_disable_cancels_motion_before_resuming(policy):
    recovery, _, enabled, _, _ = policy
    handle = moving(policy)
    enabled[0] = False
    recovery.tick(traction(), True)
    handle.cancel_goal_async.assert_called_once()
    recovery.resume.call_async.assert_not_called()
    handle.get_result_async.return_value.set_result(SimpleNamespace(status=5))
    recovery.resume.call_async.assert_called_once()


def test_user_cancel_does_not_resume_navigation(policy):
    recovery, *_ = policy
    handle = moving(policy)
    recovery.stop(False)
    handle.get_result_async.return_value.set_result(SimpleNamespace(status=5))
    recovery.resume.call_async.assert_not_called()


def test_late_goal_acceptance_after_timeout_is_canceled(policy):
    recovery, clock, _, _, canceled = policy
    settle(policy)
    clock[0] += 4
    recovery.tick(traction(), True)
    assert canceled.called and recovery.busy
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    recovery.actions[-1].send_goal_async.return_value.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    recovery.resume.call_async.assert_not_called()


def test_obstacle_or_stale_clearance_during_motion_cancels(policy):
    recovery, _, _, clearance, canceled = policy
    handle = moving(policy)
    clearance.return_value = []
    recovery.tick(traction(), True)
    assert canceled.called
    handle.cancel_goal_async.assert_called_once()
    assert recovery.resume_allowed is False


@pytest.mark.parametrize('active,enabled', [(False, True), (True, False)])
def test_never_moves_without_enabled_autonomous_navigation(policy, active, enabled):
    recovery, clock, setting, *_ = policy
    setting[0] = enabled
    for _ in range(20):
        clock[0] += .5
        recovery.tick(traction(), active)
    recovery.pause.call_async.assert_not_called()


def test_attempt_limit_stops_persistent_failure(policy):
    recovery, _, _, _, canceled = policy
    recovery.attempts = 2
    trigger(policy)
    assert canceled.called
    recovery.pause.call_async.assert_not_called()


def test_recovery_speed_matches_cruise_and_reverse_smoother_limit(policy):
    import yaml
    from ui_tuning import NavigationTuning
    path = Path(__file__).resolve().parents[1]/'config/nav2_minimal.yaml'
    config = yaml.safe_load(path.read_text())
    recovery, *_ = policy
    node = Mock(recovery=recovery)
    tuning = NavigationTuning(node, lambda: '', path)
    controller = config['controller_server']['ros__parameters']['FollowPath']
    smoother = config['velocity_smoother']['ros__parameters']
    assert recovery.speed == controller['max_vel_x']
    assert recovery.speed <= smoother['max_velocity'][0]
    assert recovery.speed <= -smoother['min_velocity'][0]
    tuning.close()


def test_progress_resets_consecutive_attempts_and_failure_keeps_count(policy):
    recovery, clock, *_ = policy
    recovery.node.components = {'gps_rover': dict(level='ok', received=clock[0], accuracy=.02,
                                                latitude=-27., longitude=153.)}
    recovery.attempts = 2
    recovery.track_progress()
    assert recovery.attempts == 2
    recovery.node.components['gps_rover']['latitude'] += .6/111320
    recovery.track_progress()
    assert recovery.attempts == 0
    recovery.attempts = 2
    recovery.fail('Test fault.')
    recovery.tick(traction(), False)
    assert recovery.attempts == 2
    assert recovery.phase == 'failed'


def test_unlimited_attempt_option(policy):
    recovery, _, _, _, canceled = policy
    recovery.node.get_parameter.side_effect = lambda name: SimpleNamespace(
        value=True if name == 'traction_recovery_enabled' else 0 if name == 'traction_recovery_max_attempts' else 2.)
    recovery.attempts = 10
    trigger(policy)
    assert recovery.phase == 'pausing'
    assert not canceled.called
