#!/usr/bin/env python3

import base64
from collections import Counter
import select
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message as RtcmMessage
from sensor_msgs.msg import NavSatFix


def _nmea_checksum(sentence):
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return checksum


def _format_lat_lon(value, is_latitude):
    direction = ('N', 'S') if is_latitude else ('E', 'W')
    hemi = direction[0] if value >= 0.0 else direction[1]
    value = abs(value)
    degrees_width = 2 if is_latitude else 3
    degrees = int(value)
    minutes = (value - degrees) * 60.0
    return f'{degrees:0{degrees_width}d}{minutes:09.6f}', hemi


def _gga_from_fix(fix):
    lat, lat_hemi = _format_lat_lon(fix.latitude, True)
    lon, lon_hemi = _format_lat_lon(fix.longitude, False)
    timestamp = time.strftime('%H%M%S', time.gmtime())
    altitude = fix.altitude if fix.altitude == fix.altitude else 0.0
    body = (
        f'GPGGA,{timestamp}.00,{lat},{lat_hemi},{lon},{lon_hemi},'
        f'1,12,1.0,{altitude:.1f},M,0.0,M,,'
    )
    return f'${body}*{_nmea_checksum(body):02X}\r\n'


def _crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc


def _rtcm_crc_ok(frame):
    expected = int.from_bytes(frame[-3:], byteorder='big')
    return _crc24q(frame[:-3]) == expected


def _rtcm_message_type(frame):
    if len(frame) < 5:
        return 0
    return ((frame[3] << 4) | (frame[4] >> 4)) & 0x0FFF


class Rtk2GoNtripClient(Node):
    def __init__(self):
        super().__init__('ntrip_client')

        self.declare_parameter('host', 'rtk2go.com')
        self.declare_parameter('port', 2101)
        self.declare_parameter('mode', 'ntrip')
        self.declare_parameter('mountpoint', '')
        self.declare_parameter('ntrip_version', '')
        self.declare_parameter('user_agent', 'NTRIP curl_test')
        self.declare_parameter('authenticate', True)
        self.declare_parameter('username', '')
        self.declare_parameter('password', 'none')
        self.declare_parameter('rtcm_frame_id', 'gps')
        self.declare_parameter('reconnect_attempt_wait_seconds', 5)
        self.declare_parameter('rtcm_timeout_seconds', 4)
        self.declare_parameter('send_gga', False)
        self.declare_parameter('gga_interval_seconds', 5)
        self.declare_parameter('ssl', False)
        self.declare_parameter('cert', '')
        self.declare_parameter('key', '')
        self.declare_parameter('ca_cert', '')
        self.declare_parameter('rtcm_message_package', 'rtcm_msgs')
        self.declare_parameter('nmea_max_length', 128)
        self.declare_parameter('nmea_min_length', 3)
        self.declare_parameter('log_rtcm_interval', 50)
        self.declare_parameter('validate_rtcm_crc', True)

        self._host = self.get_parameter('host').value
        self._port = int(self.get_parameter('port').value)
        self._mode = str(self.get_parameter('mode').value).lower()
        self._mountpoint = str(self.get_parameter('mountpoint').value).lstrip('/')
        self._ntrip_version = str(self.get_parameter('ntrip_version').value)
        self._user_agent = str(self.get_parameter('user_agent').value)
        self._authenticate = bool(self.get_parameter('authenticate').value)
        self._username = str(self.get_parameter('username').value)
        self._password = str(self.get_parameter('password').value)
        self._frame_id = str(self.get_parameter('rtcm_frame_id').value)
        self._reconnect_wait = float(
            self.get_parameter('reconnect_attempt_wait_seconds').value)
        self._rtcm_timeout = float(self.get_parameter('rtcm_timeout_seconds').value)
        self._send_gga = bool(self.get_parameter('send_gga').value)
        self._gga_interval = float(self.get_parameter('gga_interval_seconds').value)
        self._log_rtcm_interval = int(self.get_parameter('log_rtcm_interval').value)
        self._validate_rtcm_crc = bool(
            self.get_parameter('validate_rtcm_crc').value)

        self._last_fix = None
        self._last_gga_time = 0.0
        self._rtcm_count = 0
        self._rtcm_bad_crc_count = 0
        self._rtcm_type_counts = Counter()
        self._running = True
        self._pub = self.create_publisher(RtcmMessage, 'rtcm', 10)
        self.create_subscription(NavSatFix, 'fix', self._fix_callback, 10)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()

    def _fix_callback(self, msg):
        self._last_fix = msg

    def _request_bytes(self):
        request = [
            f'GET /{self._mountpoint} HTTP/1.0',
            f'Host: {self._host}:{self._port}',
        ]
        if self._authenticate:
            raw = f'{self._username}:{self._password}'.encode('utf-8')
            request.append(
                'Authorization: Basic '
                + base64.b64encode(raw).decode('ascii')
            )
        request.append(f'User-Agent: {self._user_agent}')
        if self._ntrip_version:
            request.append(f'Ntrip-Version: {self._ntrip_version}')
        request.append('Accept: */*')
        request.append('')
        request.append('')
        return '\r\n'.join(request).encode('ascii')

    def _log_request_summary(self):
        version = self._ntrip_version if self._ntrip_version else '<none>'
        auth = 'on' if self._authenticate else 'off'
        self.get_logger().info(
            'NTRIP request: '
            f'GET /{self._mountpoint} HTTP/1.0, '
            f'Host={self._host}:{self._port}, '
            f'User-Agent={self._user_agent}, '
            f'Ntrip-Version={version}, '
            f'auth={auth}'
        )

    def _connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(8.0)
        sock.connect((self._host, self._port))
        if self._mode == 'tcp':
            self.get_logger().info(
                f'Connected to raw RTCM TCP stream {self._host}:{self._port}')
        else:
            self._log_request_summary()
            sock.sendall(self._request_bytes())
            self.get_logger().info(
                f'Connecting to http://{self._host}:{self._port}/{self._mountpoint}')
        return sock

    def _send_gga_if_needed(self, sock):
        if not self._send_gga or self._last_fix is None:
            return
        now = time.monotonic()
        if now - self._last_gga_time < self._gga_interval:
            return
        sock.sendall(_gga_from_fix(self._last_fix).encode('ascii'))
        self._last_gga_time = now

    def _publish_rtcm_frames(self, buffer):
        published = 0
        while True:
            start = buffer.find(b'\xd3')
            if start < 0:
                return b'', published
            if start > 0:
                buffer = buffer[start:]
            if len(buffer) < 6:
                return buffer, published
            length = ((buffer[1] & 0x03) << 8) | buffer[2]
            frame_length = 3 + length + 3
            if len(buffer) < frame_length:
                return buffer, published
            frame = buffer[:frame_length]
            msg_type = _rtcm_message_type(frame)
            if self._validate_rtcm_crc and not _rtcm_crc_ok(frame):
                self._rtcm_bad_crc_count += 1
                if self._rtcm_bad_crc_count == 1 or self._rtcm_bad_crc_count % 10 == 0:
                    self.get_logger().warn(
                        'Dropped RTCM frame with bad CRC '
                        f'(type={msg_type}, bad_crc={self._rtcm_bad_crc_count})')
                buffer = buffer[frame_length:]
                continue

            msg = RtcmMessage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.message = bytes(frame)
            self._pub.publish(msg)
            published += 1
            self._rtcm_count += 1
            self._rtcm_type_counts[msg_type] += 1
            if (
                self._log_rtcm_interval > 0
                and self._rtcm_count % self._log_rtcm_interval == 0
            ):
                type_summary = ', '.join(
                    f'{msg_type}:{count}'
                    for msg_type, count in self._rtcm_type_counts.most_common(8)
                )
                self.get_logger().info(
                    f'Published {self._rtcm_count} valid RTCM frames on rtcm '
                    f'(types {type_summary}; bad_crc={self._rtcm_bad_crc_count})')
            buffer = buffer[frame_length:]

    def _strip_response_header(self, buffer):
        if buffer.startswith(b'\xd3'):
            return buffer, True
        if buffer.startswith(b'ICY 200 OK\r\n'):
            self.get_logger().info('NTRIP stream connected')
            return buffer[len(b'ICY 200 OK\r\n'):], True
        header_end = buffer.find(b'\r\n\r\n')
        if header_end < 0:
            if (
                buffer.startswith(b'HTTP/')
                or buffer.startswith(b'SOURCETABLE')
                or buffer.startswith(b'ICY')
            ):
                return buffer, False
            start = buffer.find(b'\xd3')
            if start >= 0:
                self.get_logger().info('NTRIP stream connected')
                return buffer[start:], True
            return buffer, False
        header = buffer[:header_end].decode('ISO-8859-1', errors='replace')
        first_line = header.splitlines()[0] if header else '<empty response>'
        self.get_logger().info(f'NTRIP response: {first_line}')
        if 'SOURCETABLE 200 OK' in header:
            raise RuntimeError('caster returned sourcetable; mountpoint/auth rejected')
        if '401' in header:
            raise RuntimeError('caster rejected NTRIP credentials')
        if (
            'ICY 200 OK' not in header
            and 'HTTP/1.0 200 OK' not in header
            and 'HTTP/1.1 200 OK' not in header
        ):
            self.get_logger().warn(f'Unexpected NTRIP response: {header[:120]}')
        self.get_logger().info('NTRIP stream connected')
        return buffer[header_end + 4:], True

    def _run(self):
        while self._running:
            sock = None
            try:
                sock = self._connect()
                buffer = b''
                connected_at = time.monotonic()
                last_rtcm = time.monotonic()
                header_checked = self._mode == 'tcp'
                while self._running:
                    self._send_gga_if_needed(sock)
                    readable, _, _ = select.select([sock], [], [], 0.2)
                    if not readable:
                        if time.monotonic() - connected_at > self._rtcm_timeout:
                            if time.monotonic() - last_rtcm > self._rtcm_timeout:
                                raise TimeoutError('RTCM timeout')
                        continue
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise ConnectionError('NTRIP socket closed')
                    buffer += chunk
                    if not header_checked:
                        buffer, header_checked = self._strip_response_header(buffer)
                        if not header_checked:
                            continue
                    buffer, published = self._publish_rtcm_frames(buffer)
                    if published:
                        last_rtcm = time.monotonic()
            except Exception as exc:
                self.get_logger().error(f'NTRIP disconnected: {exc}')
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            if self._running:
                time.sleep(self._reconnect_wait)


def main():
    rclpy.init()
    node = Rtk2GoNtripClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
