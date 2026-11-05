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
