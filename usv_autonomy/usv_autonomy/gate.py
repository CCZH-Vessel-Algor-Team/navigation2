"""Final autonomous-output contract. No ROS dependency; monotonic deadlines."""
import math

DISABLED, NEUTRAL, VELOCITY_CONTROL = 0, 1, 2


class Gate:
    def __init__(self, clock, input_timeout=1.0, command_timeout=0.3):
        self.clock = clock
        self.input_timeout, self.command_timeout = input_timeout, command_timeout
        self.authority = (False, float('-inf'))
        self.safety = (False, 0.0, float('-inf'))
        self.heartbeat = float('-inf')
        self.permit = None
        self.permit_received = float('-inf')
        self.permit_deadline = float('-inf')
        self.command = None
        self.command_deadline = float('-inf')
        self.last_sequence = -1
        self.sessions = set()
        self.boot = None
        self.revision = -1
        self.reason = 'no inputs'
        self.retired_epochs = set()

    def clear_command(self, reset_sequence=False):
        self.command = None
        self.command_deadline = float('-inf')
        if reset_sequence:
            self.last_sequence = -1

    def update_permit(self, boot, execution, attempt, epoch, revision, active, ttl):
        if not boot or not math.isfinite(ttl) or ttl <= 0 or ttl > self.input_timeout:
            return False
        if self.permit and self.permit[4] and self.clock() > self.permit_deadline:
            self.retired_epochs.add(self.permit[:4])
        if active and (boot, execution, attempt, epoch) in self.retired_epochs:
            return False
        if boot != self.boot:
            if boot in self.sessions:
                return False
            if self.boot is not None:
                self.sessions.add(self.boot)
            self.boot, self.revision = boot, -1
            self.clear_command(True)
        if revision < self.revision:
            return False
        new = (boot, execution, attempt, epoch, bool(active))
        if revision == self.revision and self.permit is not None and new != self.permit:
            return False  # One revision cannot both revoke and renew.
        if new != self.permit:
            self.clear_command(True)
        if not active:
            self.retired_epochs.add(new[:4])
        self.permit, self.revision = new, revision
        self.permit_received = self.clock()
        self.permit_deadline = self.clock() + ttl
        return True

    def accept(self, boot, execution, attempt, epoch, sequence, velocity, age, ttl):
        if not self.permit or (boot, execution, attempt, epoch, True) != self.permit:
            return False
        if not self._allowed() or self.clock() > self.permit_deadline:
            return False
        if not all(math.isfinite(v) for v in (*velocity, age, ttl)):
            return False
        if sequence <= self.last_sequence or age < -0.05 or ttl <= 0:
            return False
        lifetime = min(ttl, self.command_timeout)
        if age >= lifetime:
            return False
        self.last_sequence = sequence
        self.command = tuple(velocity)
        self.command_deadline = self.clock() + lifetime - max(age, 0.0)
        return True

    def _allowed(self):
        now = self.clock()
        return (self.authority[0] and now - self.authority[1] <= self.input_timeout
                and self.safety[0] and self.safety[1] > 0
                and now - self.safety[2] <= self.input_timeout
                and now - self.heartbeat <= self.input_timeout)

    def output(self):
        now = self.clock()
        if not self.authority[0] or now - self.authority[1] > self.input_timeout:
            self.clear_command()
            return DISABLED, (0.0, 0.0), 'external_or_unknown_authority'
        if not self._allowed():
            self.clear_command()
            return NEUTRAL, (0.0, 0.0), 'safety_or_execution_heartbeat'
        if not self.permit or not self.permit[4] or now > self.permit_deadline:
            if self.permit and self.permit[4]:
                self.retired_epochs.add(self.permit[:4])
            self.clear_command()
            return NEUTRAL, (0.0, 0.0), 'revoked_or_expired_permit'
        if self.command is None or now > self.command_deadline:
            self.clear_command()
            return NEUTRAL, (0.0, 0.0), 'expired_command'
        speed = max(0.0, self.safety[1])
        v, w = self.command
        scale = min(1.0, speed / abs(v)) if v else 1.0
        return VELOCITY_CONTROL, (v * scale, w * scale), 'accepted'
