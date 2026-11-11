"""Bounded Nav2 slip recovery with mission pause and stored-costmap clearance."""
import math
import time
import uuid

from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID

from nav2_msgs.action import BackUp, DriveOnHeading
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


def corridor_clear(grid, pose, direction, distance=.3, radius=.45):
    """Check the swept footprint against every occupied/unknown grid cell."""
    info = grid.info
    resolution = info.resolution
    geometry = (info.origin.position.x, info.origin.position.y, info.origin.orientation.x,
                info.origin.orientation.y, info.origin.orientation.z, info.origin.orientation.w)
    if not all(math.isfinite(value) for value in geometry):
        return False
    if (not math.isfinite(resolution) or resolution <= 0 or info.width <= 0 or info.height <= 0 or
            len(grid.data) != info.width * info.height or not all(math.isfinite(v) for v in pose)):
        return False
    q = info.origin.orientation
    angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = pose[0]-info.origin.position.x, pose[1]-info.origin.position.y
    x, y = c*dx+s*dy, -s*dx+c*dy
    heading = pose[2]-angle
    ex = x+direction*distance*math.cos(heading)
    ey = y+direction*distance*math.sin(heading)
    if (min(x, ex)-radius < 0 or min(y, ey)-radius < 0 or
            max(x, ex)+radius > info.width*resolution or max(y, ey)+radius > info.height*resolution):
        return False
    padded = radius+resolution*math.sqrt(2)/2
    length_squared = distance*distance
    for index, value in enumerate(grid.data):
        if 0 <= value < 65:
            continue
        px = (index % info.width+.5)*resolution
        py = (index // info.width+.5)*resolution
        fraction = max(0., min(1., ((px-x)*(ex-x)+(py-y)*(ey-y))/length_squared))
        if math.hypot(px-x-fraction*(ex-x), py-y-fraction*(ey-y)) <= padded:
            return False
    return True


class TractionRecovery:
    distance = .3
    speed = .4

    def __init__(self, node, clearance, cancel_navigation):
        self.node = node
        self.clearance = clearance
        self.cancel_navigation = cancel_navigation
        self.pause = node.create_client(Trigger, '/traction_recovery/pause_navigation')
        self.resume = node.create_client(Trigger, '/traction_recovery/resume_navigation')
        self.actions = {-1: ActionClient(node, BackUp, '/backup'),
                        1: ActionClient(node, DriveOnHeading, '/drive_on_heading')}
        self.cancel_clients = {direction: node.create_client(
            CancelGoal, name+'/_action/cancel_goal')
            for direction, name in [(-1, '/backup'), (1, '/drive_on_heading')]}
        self.goal_id = None
        self.phase = 'idle'
        self.message = 'Waiting for confirmed slip during navigation.'
        self.attempts = 0
        self.confirmed_since = None
        self.last_confirmed = 0.
        self.cooldown_until = 0.
        self.deadline = 0.
        self.future = None
        self.request_pending = False
        self.pause_started = 0.
        self.follow_path_active = True
        self.follow_path_received = 0.
        self.handle = None
        self.stop_requested = False
        self.resume_allowed = True
        self.direction = -1
        self.stopped_since = None
        self.progress_anchor = None

    @property
    def max_attempts(self):
        return self.node.get_parameter('traction_recovery_max_attempts').value

    def track_progress(self):
        components = getattr(self.node, 'components', {})
        if not isinstance(components, dict):
            return
        gps = components.get('gps_rover', {})
        if (gps.get('level') != 'ok' or time.monotonic()-gps.get('received', 0) > 1 or
                gps.get('accuracy') is None or gps['accuracy'] > .1 or
                gps.get('latitude') is None or gps.get('longitude') is None):
            self.progress_anchor = None
            return
        position = (gps['latitude'], gps['longitude'])
        if self.progress_anchor is not None:
            north = (position[0]-self.progress_anchor[0])*111320
            east = (position[1]-self.progress_anchor[1])*111320*math.cos(math.radians(position[0]))
            if math.hypot(north, east) < .5:
                return
            self.attempts = 0
        self.progress_anchor = position

    @property
    def busy(self):
        pending_fault = self.phase == 'failed' and self.future is not None and not self.future.done()
        return pending_fault or self.request_pending or self.handle is not None or self.phase in ('pausing', 'settling', 'requesting', 'moving', 'canceling', 'resuming')

    def new_mission(self):
        if not self.busy:
            self.phase = 'idle'
            self.message = 'Monitoring navigation for confirmed slip.'
            self.attempts = 0
            self.cooldown_until = 0.
            self.confirmed_since = None
            self.progress_anchor = None

    def follow_path_status(self, message):
        self.follow_path_active = any(item.status in (1, 2, 3) for item in message.status_list)
        self.follow_path_received = time.monotonic()

    def transition(self, phase, message):
        self.phase, self.message = phase, message
        self.node.event('Traction recovery '+message)
        self.node.get_logger().info('Traction recovery '+message)

    def fail(self, message):
        self.resume_allowed = False
        self.stop(False)
        self.transition('failed', message+' Navigation stopped.')
        self.cancel_navigation()

    def stop(self, resume=True):
        self.stop_requested = True
        self.resume_allowed = self.resume_allowed and resume
        if self.request_pending and self.goal_id is not None:
            client = self.cancel_clients[self.direction]
            if client.service_is_ready():
                request = CancelGoal.Request()
                request.goal_info.goal_id = self.goal_id
                client.call_async(request)
        if self.handle is not None and self.phase != 'canceling':
            self.handle.cancel_goal_async()
            if self.phase != 'failed':
                self.transition('canceling', 'Canceling recovery motion.')
            self.deadline = time.monotonic()+3

    def call(self, client, phase, callback):
        if not client.service_is_ready():
            self.fail('Navigation pause/resume service unavailable.')
            return
        self.phase = phase
        self.deadline = time.monotonic()+3
        try:
            self.future = client.call_async(Trigger.Request())
            self.future.add_done_callback(callback)
        except Exception as exc:
            self.fail('Navigation service request failed. '+str(exc))

    def paused(self, future):
        if self.phase != 'pausing':
            return
        try:
            if not future.result().success:
                self.fail('Navigation rejected pause.')
                return
            self.transition('settling', 'Navigation paused. Waiting for wheels to stop.')
            self.deadline = time.monotonic()+5
            self.stopped_since = None
        except Exception as exc:
            self.fail('Pause failed. '+str(exc))

    def start_motion(self):
        try:
            directions = self.clearance()
        except Exception as exc:
            self.fail('Clearance check failed. '+str(exc))
            return
        choices = [self.direction, -self.direction]
        direction = next((value for value in choices if value in directions), None)
        if direction is None:
            self.fail('No clear recovery corridor or fresh obstacle data.')
            return
        client = self.actions[direction]
        if not client.server_is_ready() or not self.cancel_clients[direction].service_is_ready():
            self.fail('Recovery action server unavailable.')
            return
        self.direction = direction
        goal = BackUp.Goal() if direction == -1 else DriveOnHeading.Goal()
        goal.target.x = direction*self.distance
        goal.speed = self.speed
        goal.time_allowance.sec = 6
        goal.disable_collision_checks = False
        self.request_pending = True
        self.attempts += 1
        self.transition('requesting', 'Requesting '+('reverse' if direction == -1 else 'forward')+' recovery.')
        self.deadline = time.monotonic()+3
        try:
            self.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
            self.future = client.send_goal_async(goal, goal_uuid=self.goal_id)
            self.future.add_done_callback(self.accepted)
        except Exception as exc:
            self.request_pending = False
            self.fail('Recovery request failed. '+str(exc))

    def accepted(self, future):
        self.request_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                if self.phase == 'requesting':
                    self.fail('Recovery goal rejected.')
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(self.finished)
            if self.stop_requested or self.phase != 'requesting':
                handle.cancel_goal_async()
                return
            self.transition('moving', 'Moving '+('backward' if self.direction == -1 else 'forward')+' 0.3 m.')
            self.deadline = time.monotonic()+8
        except Exception as exc:
            self.fail('Recovery request failed. '+str(exc))

    def finished(self, future):
        self.handle = None
        if self.phase not in ('moving', 'canceling', 'requesting'):
            return
        try:
            result = future.result()
            if not self.resume_allowed:
                self.transition('idle', 'Recovery canceled with navigation.')
            elif self.stop_requested or (result.status == 4 and result.result.error_code == 0):
                self.call(self.resume, 'resuming', self.resumed)
            else:
                self.fail('Recovery motion failed. '+result.result.error_msg)
        except Exception as exc:
            self.fail('Recovery result failed. '+str(exc))

    def resumed(self, future):
        if self.phase != 'resuming':
            return
        try:
            if not future.result().success:
                self.fail('Navigation rejected resume.')
                return
            self.cooldown_until = time.monotonic()+10
            self.confirmed_since = None
            self.transition('idle', 'Navigation resumed. Recovery cooldown 10 s.')
        except Exception as exc:
            self.fail('Resume failed. '+str(exc))

    def tick(self, traction, navigation_active):
        now = time.monotonic()
        enabled = self.node.get_parameter('traction_recovery_enabled').value
        delay = self.node.get_parameter('traction_recovery_delay_seconds').value
        if navigation_active:
            self.track_progress()
        if self.busy:
            if not navigation_active:
                self.stop(False)
            elif not enabled:
                self.stop()
            if self.phase == 'settling':
                metrics = traction['metrics']
                stopped = (not self.follow_path_active and self.follow_path_received >= self.pause_started and
                           metrics['wheel_usable'] and metrics['gyro_usable'] and
                           abs(metrics['wheel_vx']) < .03 and abs(metrics['wheel_wz']) < .08 and
                           abs(metrics['gyro_wz']) < .05 and
                           (metrics['command_vx'] is None or abs(metrics['command_vx']) < .02) and
                           (metrics['command_wz'] is None or abs(metrics['command_wz']) < .05))
                if not stopped:
                    self.stopped_since = None
                elif self.stopped_since is None:
                    self.stopped_since = now
                if self.stopped_since is not None and now-self.stopped_since >= .3:
                    if self.stop_requested:
                        if self.resume_allowed:
                            self.call(self.resume, 'resuming', self.resumed)
                        else:
                            self.transition('idle', 'Recovery canceled with navigation.')
                    else:
                        self.start_motion()
            if self.phase == 'moving':
                try:
                    clear = self.direction in self.clearance()
                except Exception:
                    clear = False
                if not clear:
                    self.fail('Recovery corridor or sensor freshness changed.')
            if self.phase != 'failed' and self.busy and now > self.deadline:
                self.fail('Recovery '+self.phase+' timed out.')
        else:
            if not navigation_active:
                self.confirmed_since = None
            if not enabled or not navigation_active or now < self.cooldown_until or self.phase == 'failed':
                self.confirmed_since = None
            elif traction['state'].startswith('suspected_'):
                self.last_confirmed = now
                if self.confirmed_since is None:
                    self.confirmed_since = now
                    self.message = 'Confirmed slip. Waiting before recovery.'
                if now-self.confirmed_since >= delay:
                    if self.max_attempts > 0 and self.attempts >= self.max_attempts:
                        self.fail('Recovery attempt limit reached.')
                    else:
                        self.stop_requested = False
                        self.resume_allowed = True
                        vx = traction['metrics']['command_vx'] or 0.
                        self.direction = -self.direction if self.attempts else (1 if vx < -.05 else -1)
                        self.confirmed_since = None
                        self.pause_started = now
                        self.transition('pausing', 'Confirmed slip persisted. Pausing navigation.')
                        self.call(self.pause, 'pausing', self.paused)
            elif now-self.last_confirmed > 1:
                self.confirmed_since = None
        remaining = max(0., delay-(now-self.confirmed_since)) if self.confirmed_since is not None else None
        return dict(enabled=enabled, state=self.phase, message=self.message, busy=self.busy,
                    delay=delay, countdown=remaining, attempts=self.attempts,
                    max_attempts=self.max_attempts, distance=self.distance, speed=self.speed)
