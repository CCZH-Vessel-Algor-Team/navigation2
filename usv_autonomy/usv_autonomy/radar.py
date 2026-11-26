"""Conservative near-field radar geometry and short sweep memory."""
import math
from collections import deque


def sector_distance(msg, threshold, transform=None, max_samples=50000):
    geometry = (msg.angle_start, msg.angle_increment, msg.range_min, msg.range_max, threshold)
    if (not all(math.isfinite(v) for v in geometry) or msg.range_min < 0
            or msg.range_max <= msg.range_min or not msg.intensities):
        raise ValueError('invalid radar sector geometry')
    samples = sum(len(echo.echoes) for echo in msg.intensities)
    if not 0 < samples <= max_samples:
        raise ValueError('missing or oversized radar samples')
    if transform:
        q, t = transform.rotation, transform.translation
        if (not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, t.x, t.y, t.z))
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .02):
            raise ValueError('invalid radar transform')
    nearest = float('inf')
    for ray, echo in enumerate(msg.intensities):
        count = len(echo.echoes)
        if not count:
            raise ValueError('incomplete radar ray')
        angle = msg.angle_start + ray * msg.angle_increment
        for sample, intensity in enumerate(echo.echoes):
            if not math.isfinite(intensity):
                raise ValueError('nonfinite radar intensity')
            if intensity <= threshold:
                continue
            # RadarSector defines the first and LAST sample ranges.
            distance = msg.range_min + (msg.range_max - msg.range_min) * sample / max(1, count - 1)
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            if transform:
                tx, ty, tz = -2*q.z*y, 2*q.z*x, 2*(q.x*y-q.y*x)
                x, y = (x + q.w*tx + q.y*tz - q.z*ty + t.x,
                        y + q.w*ty + q.z*tx - q.x*tz + t.y)
            nearest = min(nearest, math.hypot(x, y))
    return nearest


class RadarWindow:
    """A clear beam cannot erase a close return from the same recent sweep."""
    def __init__(self, timeout):
        self.timeout = timeout
        self.samples = deque()

    def observe(self, distance, measured_at, now):
        self.samples.append((measured_at, distance))
        return self.closest(now)

    def closest(self, now):
        while self.samples and now - self.samples[0][0] > self.timeout:
            self.samples.popleft()
        return min((distance for _, distance in self.samples), default=float('inf'))
