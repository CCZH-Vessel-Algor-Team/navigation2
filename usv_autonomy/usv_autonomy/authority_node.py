"""Read-only FCU authority adapter; never arms or changes FCU mode."""
from time import monotonic
import rclpy
from mavros_msgs.msg import State
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from usv_interfaces.msg import ControlAuthorityState
from .ros_helpers import observation_age


class AuthorityAdapter(Node):
    def __init__(self):
        super().__init__('usv_authority_v2')
        self.modes = self.declare_parameter('autonomous_modes', ['GUIDED', 'OFFBOARD']).value
        self.timeout = self.declare_parameter('state_timeout', 1.0).value
        self.state, self.received = None, float('-inf')
        self.pub = self.create_publisher(ControlAuthorityState, 'control/authority', 1)
        self.create_subscription(State, 'mavros/state', self.receive, 1)
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, state):
        age = observation_age(self, 'FCU', state.header, self.timeout)
        if age is not None:
            self.state, self.received = state, monotonic() - age

    def tick(self):
        msg = ControlAuthorityState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.valid = self.state is not None and monotonic() - self.received <= self.timeout
        if self.state:
            msg.connected, msg.armed, msg.mode = self.state.connected, self.state.armed, self.state.mode
        msg.authority = (msg.AUTONOMOUS if msg.mode in self.modes else msg.EXTERNAL) if msg.valid else msg.UNKNOWN
        msg.motion_allowed = msg.valid and msg.connected and msg.armed and msg.authority == msg.AUTONOMOUS
        msg.reason = 'FCU state observation only'
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
