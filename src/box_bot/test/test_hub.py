"""Hub packet validation and hardware-free serial/BatteryState integration."""
import math
import os
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from hub_protocol import StreamDecoder, battery_fields, encode


def packet(seq=1):
    return dict(v=1, hub='boxbot-hub', boot='1234abcd', seq=seq, t_ms=2000,
                type='battery', device='bms0', voltage_mv=39780, current_ma=-200,
                soc_pct=92, cells_mv=[3315] * 12, mos_temp_c=30, probe_temp_c=[29, 28])


def test_stream_recovers_after_noise_corruption_and_partial_reads():
    decoder = StreamDecoder()
    good = encode(packet())
    broken = good.replace(b'39780', b'39781')
    assert decoder.feed(b'boot noise\n' + broken + good[:20]) == []
    assert decoder.feed(good[20:] + good) == [packet(), packet()]
    assert decoder.errors == 1
    decoder.feed(b'@' + b'x' * 4096)
    assert len(decoder.buffer) <= 2048
    assert decoder.feed(good) == [packet()]


@pytest.mark.parametrize('field,value', [
    ('soc_pct', True), ('soc_pct', 101), ('cells_mv', []),
    ('cells_mv', [70000]), ('probe_temp_c', [29]), ('current_ma', float('nan')),
])
def test_invalid_battery_fields(field, value):
    message = packet()
    message[field] = value
    with pytest.raises(ValueError):
        battery_fields(message)


def test_serial_to_battery_and_watchdog():
    import rclpy
    from sensor_msgs.msg import BatteryState
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from rclpy.qos import qos_profile_sensor_data
    from esp32_hub import Esp32Hub

    master, slave = os.openpty()
    rclpy.init(domain_id=159, args=[
        '--ros-args', '-p', f'port:={os.ttyname(slave)}',
        '-p', 'battery_timeout_s:=0.3', '-p', 'hub_timeout_s:=2.0'])
    node = Esp32Hub()
    battery, diagnostics = [], []
    battery_sub = node.create_subscription(BatteryState, '/battery/state', battery.append, qos_profile_sensor_data)
    diag_sub = node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 10)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        spin(0.3)
        assert node.fd is not None
        good = encode(packet())
        os.write(master, good[:17])
        spin(0.1)
        assert not battery
        os.write(master, good[17:])
        spin(0.2)
        assert len(battery) == 1
        msg = battery[0]
        assert msg.voltage == pytest.approx(39.78)
        assert msg.current == pytest.approx(-0.2)
        assert msg.percentage == pytest.approx(0.92)
        assert msg.temperature == 29.0 and len(msg.cell_voltage) == 12
        assert math.isnan(msg.capacity) and all(math.isnan(v) for v in msg.cell_temperature)
        os.write(master, good)  # Duplicate must not reset the watchdog or republish.
        spin(0.2)
        assert len(battery) == 1
        node.publish_diagnostics()
        spin(0.1)
        bms = next(s for s in diagnostics[-1].status if s.name == 'boxbot_hub/bms0')
        assert bms.level == DiagnosticStatus.ERROR
        reboot = packet(0)
        reboot['boot'] = '87654321'
        os.write(master, encode(reboot))
        spin(0.2)
        assert len(battery) == 2
    finally:
        node.destroy_subscription(battery_sub)
        node.destroy_subscription(diag_sub)
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


def test_hoverboard_status_validation_freshness_and_reboot():
    import rclpy
    from std_msgs.msg import Int8
    from esp32_hub import Esp32Hub

    master, slave = os.openpty()
    rclpy.init(domain_id=159, args=['--ros-args', '-p', f'port:={os.ttyname(slave)}'])
    node = Esp32Hub()
    states = []
    node.create_subscription(Int8, '/hoverboard/power_state', lambda msg: states.append(msg.data), 10)

    def receive_status(seq, **changes):
        message = dict(packet(seq), type='status', state='unavailable', hoverboard_on=1)
        message.update(changes)
        node.receive(message, time.monotonic())

    try:
        node.read_serial()
        assert node.power_state() == -1
        receive_status(1)
        assert node.power_state() == 1  # BMS unavailable does not hide power sense.
        for invalid in (True, False, '1', 2, None):
            with pytest.raises(ValueError):
                receive_status(2, hoverboard_on=invalid)
        assert node.power_state() == 1
        receive_status(2, hoverboard_on=0)
        assert node.power_state() == 0
        receive_status(1)  # An out-of-order on packet cannot overwrite off.
        assert node.power_state() == 0
        node.last_status -= 6
        node.receive(packet(3), time.monotonic())
        assert node.power_state() == -1  # Battery traffic cannot refresh sense.
        receive_status(4, boot='87654321')
        assert node.power_state() == 1
        legacy = dict(packet(5), type='status', state='ok')
        node.receive(legacy, time.monotonic())
        assert node.power_state() == -1
        node.publish_power()
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert states and states[-1] == -1
        node.close_serial()
        assert node.power_state() == -1
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


@pytest.mark.parametrize('initial,expected', [(-1, 0), (0, 1), (1, 0)])
def test_latch_startup_never_retries_or_toggles_an_on_board(initial, expected):
    from hub_power import HubPower
    pulses = []
    power = HubPower(lambda: pulses.append(True))
    power.startup(initial, 1.)
    power.startup(initial, 2.)
    assert len(pulses) == expected


@pytest.mark.parametrize('initial,confirm,pulses_expected,success', [
    (1, True, 1, True), (0, True, 0, True), (-1, True, 0, False),
    (1, False, 1, False),
])
def test_latch_shutdown_uses_fresh_feedback(monkeypatch, initial, confirm, pulses_expected, success):
    import hub_power
    from types import SimpleNamespace
    clock, state, updated = [10.], [initial], [0.]
    pulses, warnings = [], []
    monkeypatch.setattr(hub_power, 'time', SimpleNamespace(
        monotonic=lambda: clock[0], sleep=lambda delay: clock.__setitem__(0, clock[0] + delay)))

    def read():
        clock[0] += .1
        updated[0] = clock[0]
        if pulses and confirm:
            state[0] = 0

    power = hub_power.HubPower(lambda: pulses.append(True), timeout=.5)
    assert power.shutdown(read, lambda: state[0], lambda: updated[0], warnings.append) == success
    assert len(pulses) == pulses_expected
    assert bool(warnings) != success
    power.startup(0, clock[0])
    assert len(pulses) == pulses_expected


@pytest.mark.parametrize('signal_name', ['SIGINT', 'SIGTERM'])
def test_hub_signal_shutdown_with_simulated_latch(tmp_path, signal_name):
    import signal
    import subprocess

    # Substitute only Jetson.GPIO. USB framing, ROS, signal handling and power
    # sequencing all run through the normal executable against a pseudo-terminal.
    package = tmp_path / 'Jetson'
    package.mkdir()
    (package / '__init__.py').write_text('')
    audit = tmp_path / 'gpio.log'
    (package / 'GPIO.py').write_text('''import os
BOARD, OUT, LOW, HIGH = 1, 2, 0, 1
def record(text):
    with open(os.environ['GPIO_AUDIT'], 'a') as log:
        log.write(text + '\\n')
def setwarnings(value): pass
def setmode(value): pass
def setup(pin, mode, initial): record('setup ' + str(pin))
def output(pin, value): record('output ' + str(pin) + ' ' + str(value))
def cleanup(pin): record('cleanup ' + str(pin))
''')
    master, slave = os.openpty()
    environment = dict(os.environ, ROS_DOMAIN_ID='161', GPIO_AUDIT=str(audit),
                       PYTHONPATH=str(tmp_path) + os.pathsep + os.environ.get('PYTHONPATH', ''))
    script = Path(__file__).resolve().parents[1] / 'scripts' / 'esp32_hub.py'
    with (tmp_path / 'hub.log').open('w') as log:
        process = subprocess.Popen([
            sys.executable, str(script), '--ros-args', '-p', f'port:={os.ttyname(slave)}',
            '-p', 'manage_hoverboard_power:=true',
        ], env=environment, stdout=log, stderr=subprocess.STDOUT)
        seq, is_on, high_count, signaled = 0, 0, 0, False
        deadline = time.monotonic() + 10
        try:
            while process.poll() is None and time.monotonic() < deadline:
                events = audit.read_text().splitlines() if audit.exists() else []
                count = events.count('output 11 1')
                if count > high_count:
                    is_on = 1 - is_on
                    high_count = count
                message = dict(packet(seq), type='status', state='unavailable', hoverboard_on=is_on)
                os.write(master, encode(message))
                seq += 1
                if is_on and 'output 11 0' in events and not signaled:
                    process.send_signal(getattr(signal, signal_name))
                    signaled = True
                time.sleep(.05)
            assert process.poll() == 0, (tmp_path / 'hub.log').read_text()
            assert signaled and high_count == 2 and is_on == 0
            assert audit.read_text().splitlines()[-1] == 'cleanup 11'
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)


@pytest.mark.parametrize('state,model_ready,expected', [(-1, True, 1), (0, True, 1), (1, True, 0), (1, False, 1)])
def test_controller_readiness_gate(tmp_path, state, model_ready, expected):
    from launch import LaunchContext
    from launch_ros.substitutions import ExecutableInPackage
    import subprocess
    import rclpy
    from std_msgs.msg import Int8

    rclpy.init(domain_id=160)
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    node = rclpy.create_node('fake_power_sense')
    model = node.create_publisher(String, '/robot_description',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_footprint', 'gps'
    transform.transform.rotation.w = 1.
    if model_ready:
        model.publish(String(data='<robot name="test"/>'))
        broadcaster.sendTransform(transform)
    publisher = node.create_publisher(Int8, '/hoverboard/power_state', 10)
    script = ExecutableInPackage(package='box_bot', executable='hoverboard_wait_ready.py').perform(LaunchContext())
    with (tmp_path / 'ready.log').open('w') as log:
        process = subprocess.Popen([
            script, '--ros-args', '-p', 'timeout_s:=3.0'],
            env=dict(os.environ, ROS_DOMAIN_ID='160'), stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 8
        try:
            while process.poll() is None and time.monotonic() < deadline:
                publisher.publish(Int8(data=state))
                rclpy.spin_once(node, timeout_sec=.05)
            assert process.poll() == expected, (tmp_path / 'ready.log').read_text()
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
