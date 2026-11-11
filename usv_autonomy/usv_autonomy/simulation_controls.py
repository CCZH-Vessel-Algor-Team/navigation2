"""Explicit CCS-only test inputs; never publish over real rule/health decisions."""
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from usv_interfaces.msg import ControlAuthorityState


class SimulationControls(Node):
    def __init__(self, **kwargs):
        super().__init__('usv_simulation_controls_v2', **kwargs)
        enabled = self.declare_parameter('simulation_mode', False,
            descriptor=ParameterDescriptor(read_only=True)).value
        if not enabled:
            raise ValueError('simulation_controls requires explicit simulation_mode=true')
        self.flags = dict(authority=False, maritime_wait=False, emergency=False,
                          localization_failure=False)
        self.authority_pub = self.create_publisher(ControlAuthorityState, 'control/authority', 1)
        self.test_publishers = {
            'maritime_wait': self.create_publisher(Bool, 'autonomy/test_maritime_wait', 1),
            'emergency': self.create_publisher(Bool, 'safety/estop', 1),
            'localization_failure': self.create_publisher(Bool, 'autonomy/test_localization_failure', 1),
        }
        for key in self.flags:
            self.create_service(SetBool, 'autonomy/test/' + key, self.handler(key))
        self.create_service(SetBool, 'control/request_manual', self.request_manual)
        self.create_timer(.1, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def handler(self, key):
        def callback(request, response):
            self.flags[key] = request.data
            self.publish()
            response.success = True
            response.message = f'simulation {key}={request.data}; observe task/gate state for completion'
            return response
        return callback

    def request_manual(self, request, response):
        if not request.data:
            response.success, response.message = False, 'this endpoint only requests manual takeover'
            return response
        self.flags['authority'] = False
        self.publish()
        response.success, response.message = True, 'SIM_MANUAL requested; observe control/authority'
        return response

    def publish(self):
        authority = ControlAuthorityState()
        authority.header.stamp = self.get_clock().now().to_msg()
        authority.valid = authority.connected = authority.armed = True
        autonomous = self.flags['authority']
        authority.authority = authority.AUTONOMOUS if autonomous else authority.EXTERNAL
        authority.motion_allowed = autonomous
        authority.mode = 'SIM_AUTONOMOUS' if autonomous else 'SIM_MANUAL'
        authority.reason = 'CCS explicit simulation controls'
        self.authority_pub.publish(authority)
        for key, publisher in self.test_publishers.items():
            publisher.publish(Bool(data=self.flags[key]))


def main(args=None):
    rclpy.init(args=args)
    node = SimulationControls()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
