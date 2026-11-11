"""Traction evidence is assessed without motion-control side effects."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from traction_monitor import TractionMonitor


def feed(monitor, now, command=(0.0, 0.4), wheel=(0.0, 0.4), gyro=0.0,
         gps=(0.0, 0.01), gps_valid=True):
    for source, values in [('command', command), ('wheel', wheel), ('gyro', (gyro,))]:
        monitor.update(source, values, now, now)
    monitor.update('gps', gps, now if gps_valid else None, now)
    return monitor.evaluate(now)


@pytest.mark.parametrize('expected,command,wheel,gyro,gps', [
    ('turn_slip', (0, .4), (0, .4), 0, (0, .01)),
    ('forward_slip', (.2, 0), (.2, 0), 0, (0, .01)),
    ('forward_slip', (-.2, 0), (-.2, 0), 0, (0, .01)),
    ('stall', (.2, 0), (0, 0), 0, (0, .01)),
    ('stall', (0, .4), (0, 0), 0, (0, .01)),
])
def test_sustained_mismatch_and_recovery(expected, command, wheel, gyro, gps):
    monitor = TractionMonitor()
    for i in range(21):
        result = feed(monitor, 10 + i / 10, command, wheel, gyro, gps)
        assert result['state'] == ('suspected_' if i == 20 else 'checking_') + expected
    assert feed(monitor, 12.1, command=(0, 0))['state'] == 'idle'
    assert feed(monitor, 12.2, command, wheel, gyro, gps)['state'] == 'suspected_' + expected
    for i in range(15):
        feed(monitor, 12.3 + i / 10, command=(0, 0), wheel=wheel, gyro=gyro, gps=gps)
    assert feed(monitor, 13.8, command, wheel, gyro, gps)['state'] == 'checking_' + expected


@pytest.mark.parametrize('command,wheel,gyro,gps', [
    ((0, .4), (0, .4), .4, (.08, .01)),
    ((.2, 0), (.2, 0), 0, (.18, .02)),
    ((-.2, 0), (-.2, 0), 0, (.18, .02)),
    ((.2, .4), (.2, .4), .4, (.01, .01)),
    ((.2, 0), (.2, 0), 0, (0, .10)),
])
def test_normal_motion_pivots_and_uncertainty_do_not_flag(command, wheel, gyro, gps):
    monitor = TractionMonitor()
    for i in range(40):
        result = feed(monitor, 10 + i / 10, command, wheel, gyro, gps)
        assert result['state'] == 'monitoring'


def test_bad_gnss_limits_translation_but_gyro_can_detect_turning_slip():
    monitor = TractionMonitor()
    assert feed(monitor, 10, command=(.2, 0), wheel=(.2, 0), gps_valid=False)['state'] == 'limited_data'
    for i in range(21):
        result = feed(monitor, 11 + i / 10, gps_valid=False)
    assert result['state'] == 'suspected_turn_slip'
    assert result['metrics']['gps_usable'] is False
    assert result['metrics']['gps_accuracy'] == .01


def test_stale_inputs_and_commands_reset_confirmation():
    monitor = TractionMonitor()
    feed(monitor, 10)
    feed(monitor, 11.0)
    result = monitor.evaluate(12)
    assert result['state'] == 'idle'
    assert feed(monitor, 12.1)['state'] == 'checking_turn_slip'
    monitor.update('gyro', (0,), None, 12.2)
    assert monitor.evaluate(12.2)['state'] == 'insufficient_data'
    assert feed(monitor, 12.3)['state'] == 'checking_turn_slip'


def test_corrupt_telemetry_is_reported_without_becoming_slip():
    monitor = TractionMonitor()
    result = feed(monitor, 10, wheel=(572, 3180))
    assert result['state'] == 'telemetry_fault'
    assert '572' in result['message']
    assert result['metrics']['wheel_usable'] is False
    assert feed(monitor, 10.5)['state'] == 'telemetry_fault'
    assert feed(monitor, 11.1)['state'] == 'checking_turn_slip'
    monitor.update('gyro', (math.nan,), 11.2, 11.2)
    result = monitor.evaluate(11.2)
    assert result['state'] == 'insufficient_data'
    assert result['metrics']['gyro_wz'] is None


def test_short_transients_and_out_of_order_samples():
    monitor = TractionMonitor()
    for i in range(10):
        assert feed(monitor, 10 + i / 10)['state'] == 'checking_turn_slip'
    for i in range(10):
        result = feed(monitor, 11 + i / 10, gyro=.4)
    assert result['state'] == 'monitoring'
    monitor.update('gyro', (0,), 10, 11.9)
    assert monitor.evaluate(11.9)['state'] == 'monitoring'


def test_bogged_forward_motion_with_steering_commands_and_real_gnss_uncertainty():
    monitor = TractionMonitor()
    for i in range(51):
        result = feed(monitor, 10+i/10, command=(.4, .38 if i % 2 else -.03),
                      wheel=(.39, .18 if i % 2 else -.05), gyro=.004, gps=(.012, .065))
    assert result['state'] == 'suspected_forward_slip'


def test_pivot_command_reversals_do_not_cancel_wheel_motion_evidence():
    monitor = TractionMonitor()
    for i in range(51):
        direction = -1 if i % 2 else 1
        result = feed(monitor, 10+i/10, command=(0, direction*.3), wheel=(0, direction*.2))
    assert result['state'] == 'suspected_turn_slip'


def test_mismatch_type_changes_and_brief_stop_commands_retain_evidence():
    monitor = TractionMonitor()
    confirmed = []
    for i in range(101):
        # Wheelspin and a stalled pivot alternate while body motion stays zero.
        turning = (i // 8) % 2 == 0
        command = (0, .4) if turning else (.4, 0)
        wheel = (0, .4) if turning else (.4, 0)
        if i % 10 == 9:
            command = (0, 0)
        result = feed(monitor, 10+i/10, command=command, wheel=wheel)
        if i >= 60 and command != (0, 0):
            confirmed.append(result['state'].startswith('suspected_'))
    assert confirmed and all(confirmed)


def test_sparse_transients_do_not_accumulate_into_slip():
    monitor = TractionMonitor()
    for i in range(151):
        gyro = 0 if i % 20 < 3 else .4
        result = feed(monitor, 10+i/10, gyro=gyro)
        assert not result['state'].startswith('suspected_')


def test_confirmed_slip_clears_after_real_motion_agrees():
    monitor = TractionMonitor()
    for i in range(31):
        result = feed(monitor, 10+i/10)
    assert result['state'] == 'suspected_turn_slip'
    for i in range(1, 26):
        result = feed(monitor, 13+i/10, gyro=.4)
    assert result['state'] == 'monitoring'
    assert result['duration'] == 0
