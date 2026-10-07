"""Single SDK command buffer; the wrist and fingers never overwrite each other."""
import math
import threading
import uuid


class WristMux:
    def __init__(self, lower=-900, upper=900, timeout=0.5):
        if not all(math.isfinite(x) for x in (lower, upper, timeout)) or lower >= upper or timeout <= 0:
            raise ValueError('Invalid wrist raw limits or watchdog timeout')
        self.lock = threading.RLock()
        self.lower, self.upper, self.timeout = lower, upper, timeout
        self.target = None
        self.measured = None
        self.received = None
        self.commanded = None
        self.token = ''
        self.fault = ''

    def feedback(self, positions, now):
        with self.lock:
            if len(positions) != 17 or not all(math.isfinite(x) for x in positions) or not self.lower <= positions[16] <= self.upper:
                return False
            self.measured, self.received = list(positions), now
            if self.target is None:
                self.target = list(positions)
            return True

    def fresh(self, now):
        return self.received is not None and 0 <= now - self.received <= self.timeout

    def fingers(self, positions):
        with self.lock:
            if (self.target is None or len(positions) not in (16, 17)
                    or not all(math.isfinite(x) for x in positions[:16])):
                return False
            self.target[:16] = positions[:16]
            return True

    def arm(self, now):
        with self.lock:
            if self.token or self.fault or not self.fresh(now):
                return ''
            self.target[16] = self.measured[16]
            self.token = uuid.uuid4().hex
            self.commanded = now
            return self.token

    def wrist(self, token, position, now):
        with self.lock:
            self.check(now)
            if (not token or token != self.token or self.fault
                    or not math.isfinite(position) or not self.lower <= position <= self.upper):
                return False
            self.target[16], self.commanded = position, now
            return True

    def stop(self):
        with self.lock:
            self.token = ''  # Fence even delayed commands from the previous execution.
            if self.measured is not None:
                self.target[16] = self.measured[16]

    def check(self, now):
        with self.lock:
            if self.token and (not self.fresh(now) or now - self.commanded > self.timeout):
                self.stop()
                self.fault = 'Wrist feedback or command timeout; explicit reset required'

    def reset(self, now):
        with self.lock:
            if not self.fresh(now):
                return False
            self.stop()
            self.fault = ''
            return True

    def snapshot(self, now):
        with self.lock:
            self.check(now)
            return None if self.target is None else list(self.target)
