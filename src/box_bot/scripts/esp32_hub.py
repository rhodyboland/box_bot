#!/usr/bin/env python3
"""Receive Boxbot Hub v1 telemetry over USB; never transmit to the hub."""
import errno
import math
import os
import termios
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from hub_protocol import StreamDecoder, battery_fields


class Esp32Hub(Node):
    def __init__(self):
        super().__init__('esp32_hub')
        defaults = {
            'port': '/dev/boxbot/hub',
            'hub_id': 'boxbot-hub', 'device_id': 'bms0', 'frame_id': 'base_link',
            'battery_timeout_s': 5.0, 'hub_timeout_s': 5.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        if min(self.settings['battery_timeout_s'], self.settings['hub_timeout_s']) <= 0:
            raise ValueError('Hub timeouts must be positive')
        self.fd = None
        self.attributes = None
        self.retry_at = 0.0
        self.last_error = 'Waiting for USB telemetry'
        self.last_packet = self.last_battery = None
        self.boot = self.seq = None
        self.sequence_gaps = self.schema_errors = 0
        self.decoder = StreamDecoder()
        self.telemetry = {}
        self.hub_status = {}
        self.battery_pub = self.create_publisher(BatteryState, '/battery/state', qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(0.05, self.read_serial)
        self.create_timer(1.0, self.publish_diagnostics)

    def close_serial(self):
        if self.fd is not None:
            try:
                if self.attributes is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.attributes)
            except (OSError, termios.error):
                pass
            os.close(self.fd)
            self.fd = None
            self.attributes = None

    def read_serial(self):
        now = time.monotonic()
        try:
            if self.fd is None:
                if now < self.retry_at:
                    return
                port = self.settings['port']
                self.fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
                self.attributes = termios.tcgetattr(self.fd)
                attrs = termios.tcgetattr(self.fd)
                attrs[0] = attrs[1] = attrs[3] = 0
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
                attrs[4] = attrs[5] = termios.B115200
                attrs[6][termios.VMIN] = 1
                attrs[6][termios.VTIME] = 0
                termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
                self.decoder.buffer.clear()
                self.boot = self.seq = None
                self.last_packet = self.last_battery = None
                self.hub_status.clear()
                self.telemetry.clear()
                self.get_logger().info(f'Receiving hub telemetry from {port}')
            # Bounded nonblocking work; never wait for a complete serial packet.
            for _ in range(4):
                try:
                    data = os.read(self.fd, 4096)
                except BlockingIOError:
                    break
                if not data:
                    raise OSError(errno.EIO, 'Hub USB disconnected')
                for message in self.decoder.feed(data):
                    try:
                        self.receive(message, time.monotonic())
                    except ValueError:
                        self.schema_errors += 1
        except (OSError, termios.error) as exc:
            self.last_error = str(exc)
            self.close_serial()
            self.retry_at = now + 2.0

    def receive(self, message, now):
        if (message['hub'] != self.settings['hub_id'] or
                message['device'] != self.settings['device_id']):
            return
        kind = message['type']
        if kind not in ('battery', 'status'):
            return
        fields = battery_fields(message) if kind == 'battery' else None
        if kind == 'status' and message.get('state') not in ('ok', 'stale', 'unavailable'):
            raise ValueError('invalid status')
        if self.boot != message['boot']:
            self.boot, self.seq = message['boot'], None
            self.last_battery = None
            self.telemetry.clear()
            self.hub_status.clear()
        if self.seq is not None:
            delta = (message['seq'] - self.seq) & 0xffffffff
            if delta == 0 or delta >= 0x80000000:
                return  # Duplicate or out-of-order packet is not a fresh sample.
            self.sequence_gaps += delta - 1
        self.seq, self.last_packet = message['seq'], now
        if kind == 'status':
            self.hub_status = message
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.settings['frame_id']
        for key in ('voltage', 'current', 'percentage', 'cell_voltage', 'temperature'):
            setattr(msg, key, fields[key])
        msg.charge = msg.capacity = msg.design_capacity = math.nan
        msg.cell_temperature = [math.nan] * len(msg.cell_voltage)
        msg.present = True
        # Chemistry, health and charging state are not provided by this protocol.
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self.last_battery, self.telemetry = now, fields
        self.battery_pub.publish(msg)

    def publish_diagnostics(self):
        now = time.monotonic()
        link_ok = (self.fd is not None and self.last_packet is not None and
                   now - self.last_packet <= self.settings['hub_timeout_s'])
        battery_ok = (link_ok and self.last_battery is not None and
                      now - self.last_battery <= self.settings['battery_timeout_s'] and
                      self.hub_status.get('state', 'ok') == 'ok')
        values = {
            'port': self.settings['port'], 'boot': self.boot,
            'packet_age_s': None if self.last_packet is None else round(now - self.last_packet, 2),
            'battery_age_s': None if self.last_battery is None else round(now - self.last_battery, 2),
            'framing_crc_errors': self.decoder.errors, 'schema_errors': self.schema_errors,
            'sequence_gaps': self.sequence_gaps, **self.telemetry,
            **{f'hub_{k}': v for k, v in self.hub_status.items()
               if k in ('state', 'sample_age_ms', 'good', 'invalid', 'timeouts', 'tx_dropped')},
        }
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        for name, ok, detail in (
                ('link', link_ok, 'Receiving hub packets' if link_ok else
                 ('No recent valid hub packets' if self.fd is not None else self.last_error)),
                ('bms0', battery_ok, 'Fresh battery telemetry' if battery_ok else 'Battery telemetry unavailable or stale')):
            status = DiagnosticStatus()
            status.name = f'boxbot_hub/{name}'
            status.hardware_id = self.settings['hub_id']
            status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
            status.message = detail
            status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
            array.status.append(status)
        self.diagnostics_pub.publish(array)


def main():
    rclpy.init()
    node = Esp32Hub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
