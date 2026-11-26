"""Read-only cc/state authority adapter; never arms or changes FCU mode.

Input stamps belong to the configured source clock; output stamps belong to the
BT's ROS clock. A steady-time watchdog remains active when simulation pauses.
"""
import math
from time import monotonic
import rclpy
from cc_msgs.msg import USVState
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from usv_interfaces.msg import ControlAuthorityState


class AuthorityAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('usv_authority_v2', **kwargs)
        self.modes = self.declare_parameter('autonomous_modes', ['GUIDED', 'OFFBOARD']).value
        self.timeout = self.declare_parameter('state_timeout', 1.0).value
        self.topic = self.declare_parameter('fcu_state_topic', '/usv1/cc/state').value
        clock_name = self.declare_parameter('fcu_state_clock', 'system').value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('state_timeout must be finite and positive')
        if clock_name not in ('system', 'ros'):
            raise ValueError('fcu_state_clock must be system or ros')
        if not self.topic:
            raise ValueError('fcu_state_topic must not be empty')
        self.input_clock = (Clock(clock_type=ClockType.SYSTEM_TIME) if clock_name == 'system'
                            else self.get_clock())
        self.source_stamp = None
        self.previous_input_now = self.previous_output_now = None
        self.reason = 'waiting for ' + self.topic
        self.state, self.received = None, float('-inf')
        self.pub = self.create_publisher(ControlAuthorityState, 'control/authority', 1)
        self.create_subscription(USVState, self.topic, self.receive, 1)
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Observing {self.topic} (cc_msgs/USVState, {clock_name} stamps); '
                               'read-only, no arming or mode commands')

    def check_clocks(self):
        source_now = self.input_clock.now().nanoseconds
        output_now = self.get_clock().now().nanoseconds
        if ((self.previous_input_now is not None and source_now < self.previous_input_now)
                or (self.previous_output_now is not None and output_now < self.previous_output_now)):
            self.state, self.received, self.source_stamp = None, float('-inf'), None
            self.reason = 'clock rewound; waiting for a new FCU observation'
        self.previous_input_now, self.previous_output_now = source_now, output_now
        return source_now, output_now

    def receive(self, state):
        now, _ = self.check_clocks()
        stamp = state.header.stamp.sec * 1_000_000_000 + state.header.stamp.nanosec
        age = (now - stamp) * 1e-9
        if stamp <= 0 or not -0.05 <= age <= self.timeout:
            self.received = float('-inf')
            self.reason = 'FCU source stamp stale/invalid or clock mismatch'
            return
        if self.source_stamp is not None and stamp <= self.source_stamp:
            # Replays cannot renew the watchdog, including a repeated false->true transition.
            return
        self.source_stamp = stamp
        self.state, self.received = state, monotonic() - max(age, 0.0)
        self.reason = 'FCU state observation only: ' + self.topic

    def tick(self):
        source_now, output_now = self.check_clocks()
        msg = ControlAuthorityState()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(output_now, 1_000_000_000)
        source_age = float('inf') if self.source_stamp is None else (source_now - self.source_stamp) * 1e-9
        msg.valid = (self.state is not None and monotonic() - self.received <= self.timeout
                     and -0.05 <= source_age <= self.timeout)
        if self.state:
            msg.connected, msg.armed, msg.mode = self.state.connected, self.state.armed, self.state.mode
        msg.authority = ((msg.AUTONOMOUS if msg.mode in self.modes else msg.EXTERNAL)
                         if msg.valid and msg.connected else msg.UNKNOWN)
        msg.motion_allowed = msg.valid and msg.connected and msg.armed and msg.authority == msg.AUTONOMOUS
        if not msg.valid:
            msg.reason = 'FCU unavailable/stale: ' + self.reason
        elif not msg.connected:
            msg.reason = 'FCU disconnected: ' + self.topic
        elif msg.authority == msg.EXTERNAL:
            msg.reason = f'FCU external mode {msg.mode}: ' + self.topic
        elif not msg.armed:
            msg.reason = 'FCU disarmed: ' + self.topic
        else:
            msg.reason = self.reason
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AuthorityAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
