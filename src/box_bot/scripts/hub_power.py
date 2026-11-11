"""Jetson latch control using fresh ESP32 sense feedback."""
import time


class HubPower:
    def __init__(self, pulse, timeout=5.0):
        self.pulse = pulse
        self.timeout = timeout
        self.requested = None
        self.stopping = False

    def startup(self, state, now):
        if self.stopping or self.requested is not None or state == -1:
            return
        self.requested = now
        if state == 0:
            self.pulse()

    def shutdown(self, read, state, updated, warn):
        self.stopping = True

        def wait_for(predicate):
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                read()
                if predicate():
                    return True
                time.sleep(.02)
            return False

        requested = time.monotonic()
        if not wait_for(lambda: updated() >= requested and state() != -1):
            warn('Cannot confirm hoverboard power. No shutdown pulse sent.')
            return False
        if state() == 0:
            return True
        requested = time.monotonic()
        self.pulse()
        if not wait_for(lambda: updated() >= requested and state() == 0):
            warn('Hoverboard shutdown pulse sent but off state was not confirmed.')
            return False
        return True
