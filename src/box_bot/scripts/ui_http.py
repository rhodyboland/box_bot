"""Bounded HTTP transport for BOXBOT UI. ROS work stays in the ROS executor."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import secrets
import socket
import threading
import time
from urllib.parse import urlsplit

from tuning_settings import validate_tuning


def validate_goal(payload):
    if not isinstance(payload, dict):
        raise ValueError('Expected a target pose object')
    values = {}
    for key, low, high in [('latitude', -90, 90), ('longitude', -180, 180),
                           ('heading', 0, 360)]:
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{key} must be a number')
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{key} is outside its valid range')
        values[key] = float(value)
    return values


def validate_waypoints(payload):
    points = payload.get('waypoints') if isinstance(payload, dict) else None
    if not isinstance(points, list) or not 1 <= len(points) <= 50:
        raise ValueError('A route must contain 1–50 waypoints')
    return [validate_goal(point) for point in points]


def validate_filter(payload):
    value = payload.get('min_obstacle_height') if isinstance(payload, dict) else None
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or not 0.05 <= value <= 0.50):
        raise ValueError('Minimum obstacle height must be between 0.05 and 0.50 m')
    return {'min_obstacle_height': float(value)}


class UiHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, web_directory):
        self.web_directory = Path(web_directory)
        self.token = secrets.token_urlsafe(32)
        self.commands = queue.Queue(maxsize=8)
        self.connections = threading.BoundedSemaphore(16)
        self.snapshot_lock = threading.Lock()
        self.snapshot = b'{}'
        self.snapshot_updated = time.monotonic()
        self.preview_requested = 0.0
        self.preview_image = None
        self.preview_updated = 0.0
        self.hostnames = {'localhost', socket.gethostname().lower(), socket.getfqdn().lower()}
        super().__init__(address, UiHandler)

    def process_request(self, request, client_address):
        if not self.connections.acquire(blocking=False):
            self.close_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.connections.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.connections.release()

    def preview_active(self):
        with self.snapshot_lock:
            return time.monotonic() - self.preview_requested < 3.0

    def update_preview(self, jpeg):
        with self.snapshot_lock:
            self.preview_image = jpeg
            self.preview_updated = time.monotonic()

    def update(self, state):
        encoded = json.dumps(dict(state, csrf=self.token), allow_nan=False,
                             separators=(',', ':')).encode()
        with self.snapshot_lock:
            self.snapshot = encoded
            self.snapshot_updated = time.monotonic()


class UiHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(4)

    def log_message(self, *args):
        pass

    def reply(self, code, body, content_type='application/json', snapshot_age=None):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store' if content_type in ('application/json', 'image/jpeg') else 'no-cache')
        if snapshot_age is not None:
            self.send_header('X-Boxbot-Snapshot-Age', f'{snapshot_age:.3f}')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
                         "img-src 'self' data: blob: http://mt0.google.com https://mt0.google.com; "
                         "connect-src 'self'; frame-ancestors 'none'; base-uri 'self'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def error(self, code, message):
        self.reply(code, json.dumps({'error': message}).encode())

    def trusted_host(self):
        try:
            hostname = urlsplit('http://' + self.headers.get('Host', '')).hostname
        except ValueError:
            return False
        return hostname in self.server.hostnames | {self.connection.getsockname()[0]}

    def do_GET(self):
        if not self.trusted_host():
            self.error(403, 'Use the robot IP address or hostname')
            return
        path = urlsplit(self.path).path
        if path == '/api/camera.jpg':
            with self.server.snapshot_lock:
                self.server.preview_requested = time.monotonic()
                body = self.server.preview_image
                age = time.monotonic() - self.server.preview_updated
            if body is None or age > 2.0:
                self.error(503, 'Waiting for a fresh RGB frame')
            else:
                self.reply(200, body, 'image/jpeg', snapshot_age=age)
            return
        if path == '/api/state':
            with self.server.snapshot_lock:
                body = self.server.snapshot
                age = time.monotonic() - self.server.snapshot_updated
            self.reply(200, body, snapshot_age=age)
            return
        assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                  '/favicon.svg': ('favicon.svg', 'image/svg+xml'),
                  '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                  '/style.css': ('style.css', 'text/css; charset=utf-8'),
                  '/vendor/leaflet.js': ('vendor/leaflet.js', 'text/javascript'),
                  '/vendor/leaflet.css': ('vendor/leaflet.css', 'text/css')}
        if path not in assets:
            self.error(404, 'Not found')
            return
        filename, mime = assets[path]
        try:
            body = (self.server.web_directory / filename).read_bytes()
        except OSError:
            self.error(503, 'UI assets unavailable. Rebuild box_bot.')
            return
        self.reply(200, body, mime)

    def do_POST(self):
        if not self.trusted_host():
            self.error(403, 'Invalid host')
            return
        origin = self.headers.get('Origin')
        try:
            origin_valid = not origin or urlsplit(origin).netloc.lower() == self.headers.get('Host', '').lower()
        except ValueError:
            origin_valid = False
        if not origin_valid:
            self.error(403, 'Cross-origin control is disabled')
            return
        token = self.headers.get('X-Boxbot-Token', '')
        if not hmac.compare_digest(token.encode(), self.server.token.encode()):
            self.error(403, 'Refresh BOXBOT UI before controlling the robot')
            return
        path = urlsplit(self.path).path
        if path not in ('/api/goal', '/api/cancel', '/api/waypoints', '/api/clear_trail', '/api/filter', '/api/recovery', '/api/tuning'):
            self.error(404, 'Not found')
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 16384:
                raise ValueError('Request body must be 1–16384 bytes')
            payload = json.loads(self.rfile.read(size))
            if path == '/api/goal':
                payload = validate_goal(payload)
            elif path == '/api/waypoints':
                payload = validate_waypoints(payload)
            elif path == '/api/tuning':
                payload = validate_tuning(payload)
            elif path == '/api/recovery':
                if not isinstance(payload, dict) or not payload or set(payload)-{'enabled', 'max_attempts'}:
                    raise ValueError('Expected recovery enabled or max_attempts')
                if 'enabled' in payload and type(payload['enabled']) is not bool:
                    raise ValueError('Recovery enabled must be a boolean')
                if 'max_attempts' in payload and (type(payload['max_attempts']) is not int or
                                                not 0 <= payload['max_attempts'] <= 20):
                    raise ValueError('Recovery max_attempts must be an integer from 0 to 20')
            elif path == '/api/filter':
                payload = validate_filter(payload)
        except (ValueError, UnicodeError) as exc:
            self.error(400, str(exc))
            return
        command = {'action': path.rsplit('/', 1)[1], 'payload': payload,
                   'deadline': time.monotonic() + 2, 'done': threading.Event()}
        try:
            self.server.commands.put_nowait(command)
        except queue.Full:
            self.error(503, 'Robot control queue is busy')
            return
        if not command['done'].wait(2):
            command['deadline'] = 0
            self.error(503, 'ROS bridge is not responding')
            return
        code, response = command['response']
        self.reply(code, json.dumps(response).encode())
