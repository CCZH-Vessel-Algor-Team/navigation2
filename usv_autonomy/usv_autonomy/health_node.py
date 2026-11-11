"""Nonblocking regular-health monitor. Task degradation remains in BT XML."""
import math
from time import monotonic
import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from usv_interfaces.msg import NavigationInputState, SafetyConstraintState
from .health import HealthPolicy
from .ros_helpers import duration, observation_age


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('usv_regular_safety_v2')
        def param(name, value):
            return self.declare_parameter(name, value).value
        sources = [s.strip() for s in param('required_health_sources', '').split(',') if s.strip()]
        self.policy = HealthPolicy(monotonic, param('regular_timeout', 1.),
            param('regular_clear_time', 2.), param('degraded_xy_variance', 4.),
            param('inhibited_xy_variance', 9.), param('max_speed', 1.),
            param('slow_speed', .3), sources)
        self.revision, self.last_facts = 0, None
        self.pub = self.create_publisher(SafetyConstraintState, 'autonomy/regular_safety', 1)
        self.create_subscription(Odometry, 'odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(NavigationInputState, 'health/input', self.health, 10)
        if self.declare_parameter('enable_simulation_tests', False,
                descriptor=ParameterDescriptor(read_only=True)).value:
            self.policy.required_sources.add('simulation_test')
            self.create_subscription(Bool, 'autonomy/test_localization_failure',
                                     lambda msg: self.policy.source('simulation_test', not msg.data), 1)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def odom(self, msg):
        age = observation_age(self, 'pose', msg.header, self.policy.timeout)
        if age is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w] + list(msg.pose.covariance)
        valid = all(math.isfinite(v) for v in values) and abs(
            q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) <= .02
        valid = valid and msg.pose.covariance[0] >= 0 and msg.pose.covariance[7] >= 0
        self.policy.localization(max(msg.pose.covariance[0], msg.pose.covariance[7]), valid, age)

    def health(self, msg):
        if msg.source not in self.policy.required_sources:
            return
        age = observation_age(self, 'health:' + msg.source, msg.header, self.policy.timeout)
        if age is not None:
            self.policy.source(msg.source, msg.monitor_enabled and msg.status == msg.NORMAL, age)

    def tick(self):
        allowed, cap, reasons = self.policy.evaluate()
        facts = allowed, cap, tuple(reasons)
        if facts != self.last_facts:
            self.revision += 1
            self.last_facts = facts
        state = SafetyConstraintState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.revision = self.revision
        state.level = state.WAIT_REQUIRED if not allowed else (
            state.LIMITED if cap < self.policy.max_speed else state.NORMAL)
        state.max_speed, state.task_allowed, state.hold_allowed = cap, allowed, allowed
        state.reasons = reasons
        duration(state.valid_for, .5)
        self.pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
