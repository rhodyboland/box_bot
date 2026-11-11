"""Read-only traction evidence from independent wheel, gyro and GNSS measurements."""
from collections import deque
import math


class TractionMonitor:
    def __init__(self, dwell=2.0, timeout=1.0):
        self.dwell = dwell
        self.timeout = timeout
        self.samples = {name: deque(maxlen=256) for name in ('wheel', 'gyro', 'gps', 'command')}
        self.reported = {}
        self.fault_message = ''
        self.candidate = None
        self.evidence = 0.0
        self.confirmed = False
        self.evaluated_at = None
        self.fault_until = 0.0

    def update(self, source, values, stamp, now):
        history = self.samples[source]
        if history and stamp is not None and stamp <= history[-1][0]:
            return
        self.reported[source] = (now, tuple(v if math.isfinite(v) else None for v in values))
        if stamp is None or not all(math.isfinite(v) for v in values):
            history.clear()
            return
        if stamp > now + 0.1 or now - stamp > self.timeout:
            history.clear()
            return
        if source == 'wheel' and (abs(values[0]) > 3.0 or abs(values[1]) > 8.0):
            history.clear()
            self.fault_until = now + 1.0
            self.fault_message = f'Implausible wheel odometry ({values[0]:.2f} m/s, {values[1]:.2f} rad/s). Traction assessment suspended.'
            return
        history.append((stamp, values))

    def mean(self, source, now):
        history = self.samples[source]
        while history and now - history[0][0] > 0.6:
            history.popleft()
        if not history:
            return None
        if source == 'command':
            return history[-1][1]
        return tuple(sum(sample[1][i] for sample in history) / len(history)
                     for i in range(len(history[0][1])))

    def evaluate(self, now):
        elapsed = 0.0 if self.evaluated_at is None else max(0.0, now - self.evaluated_at)
        if elapsed > self.timeout:
            self.evidence = 0.0
            self.confirmed = False
            self.candidate = None
        self.evaluated_at = now
        elapsed = min(elapsed, 0.5)
        wheel = self.mean('wheel', now)
        gyro = self.mean('gyro', now)
        gps = self.mean('gps', now)
        command = self.mean('command', now)
        readings = {}
        for source, mean in [('wheel', wheel), ('gyro', gyro), ('gps', gps), ('command', command)]:
            readings[source] = mean
            raw = self.reported.get(source)
            if mean is None and raw and now - raw[0] <= self.timeout:
                readings[source] = raw[1]
        w, g, p, c = [readings[name] for name in ('wheel', 'gyro', 'gps', 'command')]
        metrics = dict(command_vx=c[0] if c else None, command_wz=c[1] if c else None,
                       wheel_vx=w[0] if w else None, wheel_wz=w[1] if w else None,
                       gyro_wz=g[0] if g else None, gps_speed=p[0] if p else None,
                       gps_accuracy=p[1] if p else None, wheel_usable=wheel is not None,
                       gyro_usable=gyro is not None, gps_usable=gps is not None)
        state, message = 'monitoring', 'Monitoring wheel and body motion.'
        candidate = None
        if now < self.fault_until:
            state, message = 'telemetry_fault', self.fault_message
        elif not command or (abs(command[0]) < 0.08 and abs(command[1]) < 0.15):
            state, message = 'idle', 'Waiting for a sustained motion command.'
        elif wheel is None or gyro is None:
            state, message = 'insufficient_data', 'Waiting for fresh wheel odometry and body-frame gyro data.'
        else:
            # Magnitudes retain evidence when a stuck controller reverses its
            # pivot command within the averaging window.
            wheel_turn = sum(abs(sample[1][1]) for sample in self.samples['wheel']) / len(self.samples['wheel'])
            body_turn = sum(abs(sample[1][0]) for sample in self.samples['gyro']) / len(self.samples['gyro'])
            turn_error = max(abs(wheel[1] - gyro[0]), wheel_turn - body_turn)
            turn_slip = (wheel_turn >= 0.15 and turn_error >= 0.15 and
                         turn_error > 0.6 * max(wheel_turn, body_turn))
            # Straight-motion checks avoid GNSS antenna lever-arm motion during
            # pivots. Include three speed-accuracy sigmas before declaring no motion.
            gps_upper = gps[0] + 3 * gps[1] if gps else None
            straight = body_turn < 0.15
            if turn_slip:
                candidate = 'turn_slip'
                message = 'Wheel turning disagrees with measured body rotation.'
            elif gps and straight and abs(wheel[0]) >= 0.10 and gps_upper < 0.65 * abs(wheel[0]):
                candidate = 'forward_slip'
                message = 'Wheel travel exceeds GNSS-measured movement.'
            elif (abs(wheel[0]) < 0.03 and abs(wheel[1]) < 0.08 and abs(gyro[0]) < 0.05 and
                  ((abs(command[1]) >= 0.15 and abs(command[0]) < 0.08) or
                   (gps and gps_upper < 0.08))):
                candidate = 'stall'
                message = 'Motion is commanded but wheels and body show little movement.'
            elif gps is None:
                state, message = 'limited_data', 'Turning assessment available. GNSS velocity quality is insufficient for forward-slip or stall assessment.'
        # Accumulate across command and mismatch-category changes. Agreement
        # removes evidence twice as fast as disagreement adds it, clearing a
        # confirmed condition after one second of sustained agreement by default.
        reset = (state in ('telemetry_fault', 'insufficient_data') or command is None or
                 (gps is None and self.candidate == 'forward_slip' and candidate is None))
        if reset:
            self.evidence = 0.0
        else:
            change = elapsed if candidate else -2.0 * elapsed
            self.evidence = round(max(0.0, min(self.dwell, self.evidence + change)), 6)
        if candidate:
            self.candidate = candidate
        if self.evidence == 0.0:
            self.confirmed = False
            if candidate is None:
                self.candidate = None
        elif self.evidence >= self.dwell:
            self.confirmed = True
        if candidate or (self.evidence > 0.0 and state not in ('idle', 'limited_data')):
            state = ('suspected_' if self.confirmed else 'checking_') + self.candidate
            if candidate is None:
                message = 'Recent traction mismatch. Waiting for sustained agreement before clearing.'
        duration = self.evidence
        level = 'warn' if state.startswith(('checking_', 'suspected_')) or state in ('limited_data', 'telemetry_fault', 'insufficient_data') else 'ok'
        if state == 'telemetry_fault':
            level = 'error'
        return dict(state=state, level=level, message=message, duration=round(duration, 2),
                    dwell=self.dwell, metrics=metrics)
