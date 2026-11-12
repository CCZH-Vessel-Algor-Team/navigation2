"""Independent safety and final autonomous command gate (separate process)."""
import math
from time import monotonic

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from marine_sensor_msgs.msg import RadarSector
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException
from usv_interfaces.msg import (
    ActuatorCommand, ControlAuthorityState, GateState, ManagedMotion,
    MotionPermit, SafetyConstraintState, MaritimeRuleDecision,
)
from usv_interfaces.srv import ResetSafetyLatch
from .gate import Gate, VELOCITY_CONTROL
from .lifecycle import identity
from .ros_helpers import duration, fresh_stamp, observation_age, seconds, bounded_call
from .radar import RadarWindow, sector_distance


class MotionGate(Node):
    def __init__(self):
        super().__init__('usv_motion_gate_v2')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.boot = identity()
        self.gate = Gate(monotonic, param('input_timeout', 1.0), param('command_timeout', 0.3))
        self.require_radar = param('require_radar', True)
        self.radar_frame = param('radar_frame', 'usv_1/base_link')
        self.max_radar_points = param('max_radar_points', 50000)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.stop_distance = param('emergency_distance', 5.0)
        self.slow_distance = param('slow_distance', 15.0)
        self.max_speed = param('max_speed', 1.0)
        self.slow_speed = param('slow_speed', 0.3)
        self.odom_timeout = param('odom_timeout', 0.5)
        self.sensor_timeout = param('sensor_timeout', 1.0)
        self.sector_window = RadarWindow(self.sensor_timeout)
        self.manual_reset = param('emergency_requires_reset', True)
        self.clear_time = param('emergency_clear_time', 3.0)
        if not (0 < self.stop_distance < self.slow_distance and 0 < self.slow_speed <= self.max_speed):
            raise ValueError('invalid independent safety thresholds')
        self.odom_time = self.radar_time = float('-inf')
        self.sector_time = self.points_time = float('-inf')
        self.sector_nearest = float('inf')
        self.points_nearest = float('inf')
        self.radar_sector_min_intensity = param('radar_sector_min_intensity', 2.0)
        self.closest = float('inf')
        self.estop = False
        self.estop_time = float('-inf')
        self.require_estop = param('require_estop_heartbeat', False)
        self.latched = False
        self.clear_since = None
        self.permit_cap = 0.0
        self.sequence, self.safety_revision = 0, 1
        self.previous_safety = None
        self.regular_constraint = (False, 0., float('-inf'))
        self.test_maritime_wait = False
        self.test_emergency = False
        self.stand_on_enabled = param('left_crossing_stand_on', False)
        self.maritime_time = float('-inf')
        self.maritime_event = ''
        self.maritime_blocked = True
        self.maritime_reset_client = self.create_client(ResetSafetyLatch,
            'autonomy/internal/reset_maritime', callback_group=ReentrantCallbackGroup())
        self.reset_in_progress = False
        if self.declare_parameter('enable_simulation_tests', False,
                descriptor=ParameterDescriptor(read_only=True)).value:
            self.create_subscription(Bool, 'autonomy/test_maritime_wait', self.test_wait, 1)
            self.create_subscription(Bool, 'autonomy/test_emergency',
                                     lambda msg: setattr(self, 'test_emergency', msg.data), 1)
        self.create_subscription(ControlAuthorityState, 'control/authority', self.authority, 1)
        self.create_subscription(MotionPermit, 'autonomy/permit', self.permit, 1)
        self.create_subscription(ManagedMotion, 'autonomy/motion_raw', self.command, 1)
        self.create_subscription(SafetyConstraintState, 'autonomy/regular_safety', self.regular, 1)
        self.create_subscription(Bool, 'autonomy/bt_heartbeat', self.heartbeat, 1)
        self.create_subscription(Odometry, 'odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'safety/points', self.radar, qos_profile_sensor_data)
        # Raw radar echoes arrive continuously even when nothing is detected,
        # unlike the filtered point cloud which publishes only on returns.
        self.create_subscription(RadarSector, 'safety/radar_sector', self.sector,
                                 rclpy.qos.QoSProfile(depth=5,
                                     reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Bool, 'safety/estop', self.emergency, 1)
        self.create_subscription(MaritimeRuleDecision, 'autonomy/maritime', self.maritime, 1)
        self.create_service(ResetSafetyLatch, 'autonomy/reset_safety', self.reset,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.output = self.create_publisher(ActuatorCommand, 'control/actuator_command', 1)
        self.status = self.create_publisher(GateState, 'autonomy/gate_state', 1)
        self.safety_pub = self.create_publisher(SafetyConstraintState, 'autonomy/safety', 1)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def authority(self, msg):
        age = observation_age(self, 'authority', msg.header)
        if age is None:
            return
        valid = msg.valid and msg.connected
        allowed = valid and msg.authority == msg.AUTONOMOUS and msg.motion_allowed and msg.armed
        self.gate.authority = (allowed, monotonic() - age if valid else float('-inf'))

    def maritime(self, msg):
        age = observation_age(self, 'maritime', msg.header)
        if age is not None:
            self.maritime_time = monotonic() - age
            self.maritime_event = msg.event_id if msg.handover_required else ''
            self.maritime_blocked = not msg.valid or msg.intervention_required or msg.handover_required

    def heartbeat(self, msg):
        if msg.data:
            self.gate.heartbeat = monotonic()

    def regular(self, msg):
        age = observation_age(self, 'regular_safety', msg.header, min(seconds(msg.valid_for), 1.))
        if age is not None:
            cap = msg.max_speed if math.isfinite(msg.max_speed) and msg.max_speed >= 0 else 0.
            self.regular_constraint = (msg.task_allowed and not msg.emergency_latched,
                                       cap, monotonic() - age)

    def test_wait(self, msg):
        self.test_maritime_wait = msg.data

    def permit(self, msg):
        if msg.gate_boot_id != self.boot or not fresh_stamp(self, msg.header, 0.5):
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - seconds(msg.header.stamp)
        if self.gate.update_permit(msg.boot_id, msg.execution_id, msg.attempt_id, msg.epoch,
                                   msg.revision, msg.active, seconds(msg.valid_for) - max(0.0, age)):
            self.permit_cap = msg.max_speed if math.isfinite(msg.max_speed) and msg.max_speed >= 0 else 0.0

    def command(self, msg):
        v = msg.velocity
        if not all(math.isfinite(x) for x in (v.linear.x, v.linear.y, v.linear.z,
                                              v.angular.x, v.angular.y, v.angular.z)):
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - seconds(msg.header.stamp)
        self.gate.accept(msg.boot_id, msg.execution_id, msg.attempt_id, msg.epoch, msg.sequence,
                         (v.linear.x, v.angular.z), age, seconds(msg.valid_for))

    def odom(self, msg):
        v = msg.twist.twist
        age = observation_age(self, 'odometry', msg.header, self.odom_timeout)
        if age is not None and all(
                math.isfinite(x) for x in (v.linear.x, v.linear.y, v.angular.z)):
            self.odom_time = monotonic() - age

    def radar(self, msg):
        # Transform directly from the observation frame, without depending on BT.
        age = observation_age(self, 'radar', msg.header, self.sensor_timeout)
        if age is None:
            return
        if msg.width * msg.height > self.max_radar_points:
            self.points_time = float('-inf')
            return
        try:
            transform = None
            if msg.header.frame_id != self.radar_frame:
                transform = self.tf.lookup_transform(self.radar_frame, msg.header.frame_id,
                                                     rclpy.time.Time.from_msg(msg.header.stamp)).transform
            points = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
            closest = float('inf')
            for point in points:
                x, y, z = float(point[0]), float(point[1]), float(point[2])
                if not all(math.isfinite(v) for v in (x, y, z)):
                    self.points_time = float('-inf')
                    return
                if transform:
                    q, t = transform.rotation, transform.translation
                    # q * vector * inverse(q), then translate.
                    tx, ty, tz = 2*(q.y*z-q.z*y), 2*(q.z*x-q.x*z), 2*(q.x*y-q.y*x)
                    x, y = (x + q.w*tx + q.y*tz - q.z*ty + t.x,
                            y + q.w*ty + q.z*tx - q.x*tz + t.y)
                closest = min(closest, math.hypot(x, y))
            self.points_nearest = closest
            self.points_time = monotonic() - age
            self.radar_time = max(self.points_time, self.sector_time)
            if self.require_radar and closest < self.stop_distance:
                self.latched = True
        except (ValueError, AssertionError, KeyError, TransformException):
            self.points_time = float('-inf')
            return

    def sector(self, msg):
        """Check valid raw samples; retain close returns across clear beams."""
        age = observation_age(self, 'radar_sector', msg.header, self.sensor_timeout)
        if age is None:
            return
        try:
            transform = None
            if msg.header.frame_id != self.radar_frame:
                transform = self.tf.lookup_transform(self.radar_frame, msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp)).transform
            distance = sector_distance(msg, self.radar_sector_min_intensity, transform, self.max_radar_points)
            now = monotonic()
            self.sector_time = now - age
            self.sector_nearest = self.sector_window.observe(distance, self.sector_time, now)
            self.radar_time = max(self.points_time, self.sector_time)
            if self.require_radar and distance < self.stop_distance:
                self.latched = True
        except (ValueError, AssertionError, KeyError, TransformException, ZeroDivisionError):
            self.sector_time = float('-inf')

    def current_closest(self, now):
        """Nearest fresh observation; stale sources never claim an empty scene."""
        values = []
        if now - self.points_time <= self.sensor_timeout:
            values.append(self.points_nearest)
        if now - self.sector_time <= self.sensor_timeout:
            values.append(self.sector_window.closest(now))
        return min(values) if values else float('inf')

    def emergency(self, msg):
        self.estop, self.estop_time = msg.data, monotonic()

    def hazards(self):
        now = monotonic()
        reasons = []
        self.radar_time = max(self.points_time, self.sector_time)
        self.closest = self.current_closest(now)
        if now - self.odom_time > self.odom_timeout:
            reasons.append('odometry_invalid')
        if self.require_radar and now - self.radar_time > self.sensor_timeout:
            reasons.append('emergency_observation_invalid')
        if self.require_estop and now - self.estop_time > self.sensor_timeout:
            reasons.append('estop_input_stale')
        emergency = self.estop or self.test_emergency or (self.require_radar and self.closest < self.stop_distance)
        if self.estop:
            reasons.append('estop')
        if self.test_emergency:
            reasons.append('simulation_estop')
        if self.require_radar and self.closest < self.stop_distance:
            reasons.append('emergency_distance')
        return reasons, emergency

    async def reset(self, request, response):
        reasons, emergency = self.hazards()
        if (self.reset_in_progress or not request.confirmed or emergency or reasons
                or self.clear_since is None or monotonic() - self.clear_since < self.clear_time):
            response.accepted, response.reason = False, 'conditions not stably clear or confirmation missing'
            return response
        if (self.stand_on_enabled or self.maritime_event) and monotonic() - self.maritime_time > 1.0:
            response.accepted, response.reason = False, 'maritime reset state unavailable'
            return response
        self.reset_in_progress = True
        try:
            if self.maritime_event:
                result = await bounded_call(self, self.maritime_reset_client, request, 3.0)
                if not result.accepted:
                    response.accepted, response.reason = False, result.reason
                    return response
            # A real emergency may have arrived while the distributed reset ran.
            reasons, emergency = self.hazards()
            if emergency or reasons or self.clear_since is None or monotonic() - self.clear_since < self.clear_time:
                response.accepted, response.reason = False, 'hazards changed during reset'
                return response
            self.latched = False
            response.accepted, response.reason = True, 'protection reset; task reentry still requires admission'
        except Exception as exc:
            response.accepted, response.reason = False, str(exc)
        finally:
            self.reset_in_progress = False
        return response

    def tick(self):
        reasons, emergency = self.hazards()
        now = monotonic()
        if emergency:
            self.latched = True
        if not emergency and not reasons:
            if self.clear_since is None:
                self.clear_since = now
            if not self.manual_reset and now - self.clear_since >= self.clear_time:
                self.latched = False
        else:
            self.clear_since = None
        allowed = not reasons and not self.latched
        cap = self.slow_speed if self.require_radar and self.closest < self.slow_distance else self.max_speed
        regular_allowed, regular_cap, regular_received = self.regular_constraint
        regular_allowed = regular_allowed and now - regular_received <= .5
        maritime_allowed = (not self.stand_on_enabled or
                            (now - self.maritime_time <= 1.0 and not self.maritime_blocked))
        maritime_allowed = maritime_allowed and not self.maritime_event
        self.gate.safety = (allowed and regular_allowed and not self.test_maritime_wait and maritime_allowed,
                            min(cap, self.permit_cap, regular_cap), now)
        intent, velocity, reason = self.gate.output()
        command = ActuatorCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.boot_id, command.sequence, command.intent = self.boot, self.sequence, intent
        if self.gate.permit:
            command.execution_id, command.attempt_id, command.epoch = self.gate.permit[1:4]
        self.sequence += 1
        command.velocity.linear.x, command.velocity.angular.z = velocity
        duration(command.valid_for, 0.2)
        command.reason = reason
        self.output.publish(command)
        state = GateState()
        state.header = command.header
        state.boot_id, state.permit_boot_id = self.boot, self.gate.boot or ''
        state.permit_revision = max(0, self.gate.revision)
        state.epoch = self.gate.permit[3] if self.gate.permit else 0
        state.permitted = bool(self.gate.permit and self.gate.permit[4]
                               and now <= self.gate.permit_deadline and self.gate._allowed())
        state.intent, state.reason = intent, reason
        self.status.publish(state)
        safety = SafetyConstraintState()
        safety.header = command.header
        facts = (allowed, cap, self.latched, tuple(reasons))
        if facts != self.previous_safety:
            self.safety_revision += 1
            self.previous_safety = facts
        safety.revision = self.safety_revision
        safety.level = safety.EMERGENCY if self.latched else (safety.NORMAL if allowed else safety.WAIT_REQUIRED)
        if allowed and cap < self.max_speed:
            safety.level = safety.LIMITED
        safety.max_speed, safety.task_allowed, safety.hold_allowed = cap, allowed, allowed
        safety.emergency_latched, safety.reasons = self.latched, reasons
        duration(safety.valid_for, 0.5)
        self.safety_pub.publish(safety)


def main(args=None):
    rclpy.init(args=args)
    node = MotionGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
