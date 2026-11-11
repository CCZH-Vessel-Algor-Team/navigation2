import math
from geometry_msgs.msg import PoseStamped


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def duration(value, seconds_value):
    value.sec = int(seconds_value)
    value.nanosec = int((seconds_value - int(seconds_value)) * 1e9)
    return value


def pose_dict(pose):
    p, q = pose.pose.position, pose.pose.orientation
    return dict(frame=pose.header.frame_id, position=[p.x, p.y, p.z],
                orientation=[q.x, q.y, q.z, q.w])


def pose_message(data):
    pose = PoseStamped()
    pose.header.frame_id = data['frame']
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = data['position']
    q = pose.pose.orientation
    q.x, q.y, q.z, q.w = data['orientation']
    return pose


def fresh_stamp(node, header, maximum=1.0):
    age = node.get_clock().now().nanoseconds * 1e-9 - seconds(header.stamp)
    return math.isfinite(age) and -0.05 <= age <= maximum


def observation_age(node, key, header, maximum=1.0):
    """Accept only a fresh, advancing source stamp; replay cannot feed a watchdog."""
    stamp = seconds(header.stamp)
    if not fresh_stamp(node, header, maximum):
        return None
    cache = getattr(node, '_observation_stamps', {})
    if stamp <= cache.get(key, float('-inf')):
        return None
    cache[key] = stamp
    node._observation_stamps = cache
    return max(0.0, node.get_clock().now().nanoseconds * 1e-9 - stamp)


async def bounded_call(node, client, request, timeout=3.0):
    """Await without blocking the executor; clients need a separate callback group."""
    from rclpy.clock import Clock, ClockType
    if not client.service_is_ready():
        raise RuntimeError('service unavailable')
    future = client.call_async(request)
    timer = node.create_timer(timeout, lambda: future.cancel() if not future.done() else None,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        result = await future
        if result is None:
            raise RuntimeError('service timed out')
        return result
    finally:
        node.destroy_timer(timer)
        client.remove_pending_request(future)
