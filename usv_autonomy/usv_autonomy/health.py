"""Regular safety policy, separate from independent emergency sensing."""
import math


class HealthPolicy:
    def __init__(self, clock, timeout=1., stable_time=2., degraded_variance=4.,
                 inhibited_variance=9., max_speed=1., degraded_speed=.3,
                 required_sources=()):
        if not (0 <= degraded_variance < inhibited_variance and
                0 < degraded_speed <= max_speed and timeout > 0 and stable_time >= 0):
            raise ValueError('invalid regular safety policy')
        self.clock, self.timeout, self.stable_time = clock, timeout, stable_time
        self.degraded_variance, self.inhibited_variance = degraded_variance, inhibited_variance
        self.max_speed, self.degraded_speed = max_speed, degraded_speed
        self.required_sources = set(required_sources)
        self.variance, self.pose_valid, self.pose_received = 0., False, float('-inf')
        self.sources = {}
        self.clear_since = None

    def localization(self, variance, valid, age=0.):
        self.variance, self.pose_valid = variance, valid
        self.pose_received = self.clock() - age

    def source(self, name, normal, age=0.):
        self.sources[name] = (normal, self.clock() - age)

    def evaluate(self):
        now = self.clock()
        reasons = []
        if (not self.pose_valid or now - self.pose_received > self.timeout
                or not math.isfinite(self.variance) or self.variance < 0
                or self.variance > self.inhibited_variance):
            reasons.append('localization_quality_or_age')
        for source in sorted(self.required_sources):
            state = self.sources.get(source)
            if not state or not state[0] or now - state[1] > self.timeout:
                reasons.append('health:' + source)
        if reasons:
            self.clear_since = None
            return False, 0., reasons
        if self.clear_since is None:
            self.clear_since = now
        if now - self.clear_since < self.stable_time:
            return False, 0., ['regular_recovery_stabilizing']
        if self.variance > self.degraded_variance:
            return True, self.degraded_speed, ['localization_degraded']
        return True, self.max_speed, []
