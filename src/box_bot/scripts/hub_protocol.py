"""Dependency-free Boxbot Hub v1 framing; independent of ROS and pyserial."""
import binascii
import json

MAX_LINE = 2048  # Includes LF.


def battery_fields(message):
    """Validate the battery schema and return values in BatteryState SI units."""
    def integer(value, low, high):
        if type(value) is not int or not low <= value <= high:
            raise ValueError('battery value outside schema')
        return value

    cells = message.get('cells_mv')
    probes = message.get('probe_temp_c')
    if not isinstance(cells, list) or not 1 <= len(cells) <= 32:
        raise ValueError('cell count')
    if not isinstance(probes, list) or len(probes) != 2:
        raise ValueError('probe count')
    return {
        'voltage': integer(message.get('voltage_mv'), 0, 655350) / 1000.0,
        'current': integer(message.get('current_ma'), -327670, 327670) / 1000.0,
        'percentage': integer(message.get('soc_pct'), 0, 100) / 100.0,
        'cell_voltage': [integer(v, 0, 65535) / 1000.0 for v in cells],
        'temperature': float(max(integer(v, -100, 100) for v in probes)),
        'mos_temperature': integer(message.get('mos_temp_c'), -100, 100),
        'probe_temperature': probes,
    }


def encode(message):
    payload = json.dumps(message, separators=(',', ':'), allow_nan=False).encode('ascii')
    if b'*' in payload or b'@' in payload:
        raise ValueError('reserved character')
    line = b'@' + payload + b'*' + f'{binascii.crc_hqx(payload, 0xffff):04X}'.encode() + b'\n'
    if len(line) > MAX_LINE:
        raise ValueError('oversize')
    return line


def decode(line):
    if len(line) > MAX_LINE or not line.startswith(b'@') or not line.endswith(b'\n'):
        raise ValueError('framing')
    payload, separator, checksum = line[1:-1].rpartition(b'*')
    if not separator or len(checksum) != 4 or any(c not in b'0123456789ABCDEF' for c in checksum):
        raise ValueError('checksum format')
    if b'@' in payload or b'*' in payload or b'\r' in payload or b'\n' in payload:
        raise ValueError('reserved character')
    if binascii.crc_hqx(payload, 0xffff) != int(checksum, 16):
        raise ValueError('checksum mismatch')
    def reject_constant(value):
        raise ValueError('non-finite number')
    def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate key')
            result[key] = value
        return result
    try:
        message = json.loads(payload.decode('ascii'), parse_constant=reject_constant,
                             object_pairs_hook=unique_pairs)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError('JSON') from exc
    if not isinstance(message, dict) or type(message.get('v')) is not int or message['v'] != 1:
        raise ValueError('unsupported protocol version')
    for key in ('seq', 't_ms'):
        if type(message.get(key)) is not int or not 0 <= message[key] <= 0xffffffff:
            raise ValueError(key)
    for key in ('hub', 'boot', 'type', 'device'):
        if not isinstance(message.get(key), str) or not message[key]:
            raise ValueError(key)
    if len(message['boot']) != 8 or any(c not in '0123456789abcdef' for c in message['boot']):
        raise ValueError('boot')
    return message  # Per-message schema validation is the device adapter's responsibility.


class StreamDecoder:
    """Resync at @; bounded storage even for noise and missing newlines."""
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        messages = []
        for byte in data:
            if byte == ord('@'):
                if self.buffer:
                    self.errors += 1
                self.buffer.clear()
                self.buffer.append(byte)
            elif self.buffer:
                self.buffer.append(byte)
                if len(self.buffer) > MAX_LINE:
                    self.errors += 1
                    self.buffer.clear()
                elif byte == 10:
                    try:
                        messages.append(decode(bytes(self.buffer)))
                    except ValueError:
                        self.errors += 1
                    self.buffer.clear()
        return messages
