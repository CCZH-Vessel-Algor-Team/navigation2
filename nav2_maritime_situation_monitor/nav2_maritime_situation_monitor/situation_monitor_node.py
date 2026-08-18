# Copyright 2026 vectorwang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS boundary for deterministic all-target maritime situation reports."""

import math
import threading

from geometry_msgs.msg import Point
from nav2_colregs_msgs.msg import TrackedShipList
from nav2_maritime_situation_msgs.msg import SituationReport, SituationReportArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .enu_algorithms import (
    EncounterConfig,
    EncounterType,
    RiskLevel,
    propagate_motion,
    RiskThreshold,
    RiskThresholds,
    validate_risk_thresholds,
    Vector2,
    VesselMotion,
)
from .report_builder import AssessmentConfig, build_reports, TrackedMotion


ENCOUNTER_NAMES = {
    int(EncounterType.UNKNOWN): 'UNKNOWN',
    int(EncounterType.HEAD_ON): 'HEAD_ON',
    int(EncounterType.OVERTAKING): 'OVERTAKING',
    int(EncounterType.CROSSING_LEFT): 'CROSSING_LEFT',
    int(EncounterType.CROSSING_RIGHT): 'CROSSING_RIGHT',
}

RISK_NAMES = {
    int(RiskLevel.SAFE): 'SAFE',
    int(RiskLevel.INFO): 'INFO',
    int(RiskLevel.WARNING): 'WARNING',
    int(RiskLevel.CRITICAL): 'CRITICAL',
}


def yaw_from_quaternion(quaternion) -> float:
    """Return planar yaw from a finite quaternion."""
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion values must be finite')
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def transform_point(x: float, y: float, transform) -> Vector2:
    """Rotate and translate a planar point using a TransformStamped."""
    translation = transform.transform.translation
    values = (x, y, translation.x, translation.y)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('point and transform values must be finite')
    yaw = yaw_from_quaternion(transform.transform.rotation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    point = Vector2(
        cosine * x - sine * y + translation.x,
        sine * x + cosine * y + translation.y,
    )
    if not all(math.isfinite(value) for value in (point.x, point.y)):
        raise ValueError('transformed point must be finite')
    return point


def rotate_vector(x: float, y: float, transform) -> Vector2:
    """Rotate a planar vector without applying transform translation."""
    translation = transform.transform.translation
    values = (x, y, translation.x, translation.y)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('vector and transform values must be finite')
    yaw = yaw_from_quaternion(transform.transform.rotation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    vector = Vector2(cosine * x - sine * y, sine * x + cosine * y)
    if not all(math.isfinite(value) for value in (vector.x, vector.y)):
        raise ValueError('rotated vector must be finite')
    return vector


def effective_source_time_ns(message, receipt_time):
    """Return the source stamp, falling back to receipt time for zero stamps."""
    stamp = message.header.stamp
    if stamp.sec != 0 or stamp.nanosec != 0:
        return stamp.sec * 1_000_000_000 + stamp.nanosec
    return receipt_time


def snapshot_is_stale(snapshot, now_ns, timeout):
    """Reject expired snapshots and timestamps ahead of the current clock."""
    message, receipt_time = snapshot
    source_time = effective_source_time_ns(message, receipt_time)
    if source_time > now_ns or receipt_time > now_ns:
        return True
    return now_ns - source_time > timeout * 1_000_000_000


class MaritimeSituationMonitor(Node):
    """Assess fresh ownship and target snapshots at a fixed frequency."""

    def __init__(self):
        super().__init__('maritime_situation_monitor')

        self.declare_parameter('publish_frequency', 0.5)
        self.declare_parameter('tracked_ship_topic', '/tracked_ship')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/maritime_situation')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_timeout', 3.0)
        self.declare_parameter('ownship_timeout', 1.0)
        self.declare_parameter('transform_timeout', 0.2)
        self.declare_parameter('relative_speed_epsilon', 1.0e-6)
        self.declare_parameter('course_speed_epsilon', 0.05)
        self.declare_parameter('info_dcpa_threshold', 30.0)
        self.declare_parameter('info_tcpa_threshold', 120.0)
        self.declare_parameter('warning_dcpa_threshold', 20.0)
        self.declare_parameter('warning_tcpa_threshold', 30.0)
        self.declare_parameter('critical_dcpa_threshold', 10.0)
        self.declare_parameter('critical_tcpa_threshold', 10.0)
        self.declare_parameter('head_on_bearing_threshold_deg', 6.0)
        self.declare_parameter('reciprocal_heading_tolerance_deg', 15.0)
        self.declare_parameter('overtaking_stern_sector_deg', 112.5)
        self.declare_parameter('enable_markers', True)
        self.declare_parameter('marker_topic', '/maritime_situation_markers')
        self.declare_parameter('marker_scale', 3.0)
        self.declare_parameter('marker_label_scale', 2.0)
        self.declare_parameter('marker_velocity_scale', 2.0)
        self.declare_parameter('marker_line_width', 0.2)
        self.declare_parameter('marker_z_offset', 0.5)

        self._publish_frequency = self._parameter('publish_frequency')
        self._tracked_ship_topic = self._parameter('tracked_ship_topic')
        self._odom_topic = self._parameter('odom_topic')
        self._output_topic = self._parameter('output_topic')
        self._global_frame = self._parameter('global_frame')
        self._base_frame = self._parameter('base_frame')
        self._target_timeout = self._parameter('target_timeout')
        self._ownship_timeout = self._parameter('ownship_timeout')
        self._transform_timeout = self._parameter('transform_timeout')
        self._enable_markers = self._parameter('enable_markers')
        self._marker_topic = self._parameter('marker_topic')
        self._marker_scale = self._parameter('marker_scale')
        self._marker_label_scale = self._parameter('marker_label_scale')
        self._marker_velocity_scale = self._parameter('marker_velocity_scale')
        self._marker_line_width = self._parameter('marker_line_width')
        self._marker_z_offset = self._parameter('marker_z_offset')

        risk_thresholds = RiskThresholds(
            info=RiskThreshold(
                self._parameter('info_dcpa_threshold'),
                self._parameter('info_tcpa_threshold'),
            ),
            warning=RiskThreshold(
                self._parameter('warning_dcpa_threshold'),
                self._parameter('warning_tcpa_threshold'),
            ),
            critical=RiskThreshold(
                self._parameter('critical_dcpa_threshold'),
                self._parameter('critical_tcpa_threshold'),
            ),
        )
        encounter_config = EncounterConfig(
            head_on_bearing_rad=math.radians(
                self._parameter('head_on_bearing_threshold_deg')
            ),
            reciprocal_course_tolerance_rad=math.radians(
                self._parameter('reciprocal_heading_tolerance_deg')
            ),
            stern_sector_rad=math.radians(
                self._parameter('overtaking_stern_sector_deg')
            ),
            course_epsilon=self._parameter('course_speed_epsilon'),
        )
        self._assessment_config = AssessmentConfig(
            risk_thresholds=risk_thresholds,
            relative_speed_epsilon=self._parameter('relative_speed_epsilon'),
            encounter_config=encounter_config,
        )
        self._validate_configuration()

        self._lock = threading.Lock()
        self._latest_odom = None
        self._latest_targets = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        if not isinstance(self._tf_listener.group, ReentrantCallbackGroup):
            raise RuntimeError('TF listener callbacks must be reentrant')
        self._timer_callback_group = MutuallyExclusiveCallbackGroup()

        output_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._publisher = self.create_publisher(
            SituationReportArray, self._output_topic, output_qos
        )
        if self._enable_markers:
            self._marker_publisher = self.create_publisher(
                MarkerArray, self._marker_topic, output_qos
            )
        else:
            self._marker_publisher = None
        self._target_subscription = self.create_subscription(
            TrackedShipList,
            self._tracked_ship_topic,
            self._targets_callback,
            qos_profile_sensor_data,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(
            1.0 / self._publish_frequency,
            self._assess,
            callback_group=self._timer_callback_group,
        )

    def _parameter(self, name):
        return self.get_parameter(name).value

    def _validate_configuration(self):
        timing_values = (
            self._publish_frequency,
            self._target_timeout,
            self._ownship_timeout,
            self._transform_timeout,
        )
        if not all(
            isinstance(value, (int, float))
            and math.isfinite(value)
            and value > 0.0
            for value in timing_values
        ):
            raise ValueError('timing parameters must be finite and positive')

        for name in (
            self._tracked_ship_topic,
            self._odom_topic,
            self._output_topic,
            self._global_frame,
            self._base_frame,
        ):
            if not isinstance(name, str) or not name:
                raise ValueError('topic and frame parameters must be non-empty')

        validate_risk_thresholds(self._assessment_config.risk_thresholds)
        config = self._assessment_config
        encounter = config.encounter_config
        epsilon_values = (config.relative_speed_epsilon, encounter.course_epsilon)
        if not all(
            math.isfinite(value) and value >= 0.0 for value in epsilon_values
        ):
            raise ValueError('epsilon parameters must be finite and non-negative')
        angles = (
            encounter.head_on_bearing_rad,
            encounter.reciprocal_course_tolerance_rad,
            encounter.stern_sector_rad,
        )
        if not all(math.isfinite(value) and 0.0 <= value <= math.pi for value in angles):
            raise ValueError('angle parameters must be finite and within [0, 180] degrees')

        if not isinstance(self._enable_markers, bool):
            raise ValueError('enable_markers must be a boolean')
        if self._enable_markers:
            if not isinstance(self._marker_topic, str) or not self._marker_topic:
                raise ValueError('marker_topic must be non-empty when markers are enabled')
            marker_geometry = (
                self._marker_scale,
                self._marker_label_scale,
                self._marker_velocity_scale,
                self._marker_line_width,
                self._marker_z_offset,
            )
            if not all(
                isinstance(value, (int, float))
                and math.isfinite(value)
                and value > 0.0
                for value in marker_geometry
            ):
                raise ValueError(
                    'marker geometry parameters must be finite and positive'
                )

    def _targets_callback(self, message):
        receipt_time = self.get_clock().now().nanoseconds
        with self._lock:
            self._latest_targets = (message, receipt_time)

    def _odom_callback(self, message):
        receipt_time = self.get_clock().now().nanoseconds
        with self._lock:
            self._latest_odom = (message, receipt_time)

    def _snapshot_inputs(self):
        with self._lock:
            return self._latest_odom, self._latest_targets

    def _empty_output(self, now):
        output = SituationReportArray()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self._global_frame
        return output

    def _lookup_transform(self, source_frame, source_time_ns):
        if not source_frame:
            raise ValueError('source frame must be non-empty')
        return self._tf_buffer.lookup_transform(
            self._global_frame,
            source_frame,
            Time(
                nanoseconds=source_time_ns,
                clock_type=self.get_clock().clock_type,
            ),
            timeout=Duration(seconds=self._transform_timeout),
        )

    def _ownship_motion(self, odom, source_time_ns):
        pose_transform = self._lookup_transform(odom.header.frame_id, source_time_ns)
        twist_frame = odom.child_frame_id or self._base_frame
        twist_transform = self._lookup_transform(twist_frame, source_time_ns)
        return VesselMotion(
            position=transform_point(
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                pose_transform,
            ),
            velocity=rotate_vector(
                odom.twist.twist.linear.x,
                odom.twist.twist.linear.y,
                twist_transform,
            ),
        )

    def _target_motion(self, ship, transform):
        return TrackedMotion(
            target_id=bytes(ship.target_id.uuid),
            motion=VesselMotion(
                position=transform_point(
                    ship.pose.position.x, ship.pose.position.y, transform
                ),
                velocity=rotate_vector(
                    ship.twist.linear.x, ship.twist.linear.y, transform
                ),
            ),
        )

    @staticmethod
    def _risk_color(risk_level):
        if risk_level == RiskLevel.CRITICAL:
            return 1.0, 0.0, 0.0
        if risk_level == RiskLevel.WARNING:
            return 1.0, 0.5, 0.0
        if risk_level == RiskLevel.INFO:
            return 1.0, 1.0, 0.0
        return 0.0, 1.0, 0.0

    def _encounter_yaw(self, encounter, target, ownship):
        target_position = target.motion.position
        if encounter == EncounterType.HEAD_ON:
            return math.atan2(
                ownship.position.y - target_position.y,
                ownship.position.x - target_position.x,
            )
        if encounter == EncounterType.OVERTAKING:
            return math.atan2(
                target_position.y - ownship.position.y,
                target_position.x - ownship.position.x,
            )
        ownship_course = 0.0
        if ownship.velocity.norm() >= (
            self._assessment_config.encounter_config.course_epsilon
        ):
            ownship_course = math.atan2(
                ownship.velocity.y, ownship.velocity.x
            )
        if encounter == EncounterType.CROSSING_LEFT:
            return ownship_course + math.pi / 2.0
        if encounter == EncounterType.CROSSING_RIGHT:
            return ownship_course - math.pi / 2.0
        return 0.0

    def _encounter_base_marker(self, target, now, marker_id, yaw, risk):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self._global_frame
        marker.ns = 'maritime_' + target.target_id.hex()
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.position.x = target.motion.position.x
        marker.pose.position.y = target.motion.position.y
        marker.pose.position.z = self._marker_z_offset
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        red, green, blue = self._risk_color(risk)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.95
        return marker

    def _pentagon_encounter_marker(self, target, yaw, risk, now):
        marker = self._encounter_base_marker(target, now, 0, yaw, risk)
        marker.type = Marker.TRIANGLE_LIST
        radius = self._marker_scale * 0.5
        center = Point(x=0.0, y=0.0, z=0.0)
        marker.points = []
        for index in range(5):
            angle = 2.0 * math.pi * index / 5.0
            next_angle = 2.0 * math.pi * (index + 1) / 5.0
            marker.points.extend([
                center,
                Point(
                    x=radius * math.cos(angle),
                    y=radius * math.sin(angle),
                    z=0.0,
                ),
                Point(
                    x=radius * math.cos(next_angle),
                    y=radius * math.sin(next_angle),
                    z=0.0,
                ),
            ])
        return marker

    def _cylinder_encounter_marker(self, target, yaw, risk, now):
        marker = self._encounter_base_marker(target, now, 0, yaw, risk)
        marker.type = Marker.CYLINDER
        marker.scale.x = self._marker_scale * 0.8
        marker.scale.y = self._marker_scale * 0.8
        marker.scale.z = self._marker_scale * 0.5
        return marker

    def _sphere_encounter_marker(self, target, yaw, risk, now):
        marker = self._encounter_base_marker(target, now, 0, yaw, risk)
        marker.type = Marker.SPHERE
        marker.scale.x = self._marker_scale * 0.8
        marker.scale.y = self._marker_scale * 0.8
        marker.scale.z = self._marker_scale * 0.8
        return marker

    def _square_encounter_marker(self, target, yaw, risk, now):
        marker = self._encounter_base_marker(target, now, 0, yaw, risk)
        marker.type = Marker.TRIANGLE_LIST
        half = self._marker_scale * 0.3
        marker.points = [
            Point(x=-half, y=-half, z=0.0),
            Point(x=half, y=-half, z=0.0),
            Point(x=half, y=half, z=0.0),
            Point(x=-half, y=-half, z=0.0),
            Point(x=half, y=half, z=0.0),
            Point(x=-half, y=half, z=0.0),
        ]
        return marker

    def _combo_triangle_marker(self, target, yaw, risk, now):
        marker = self._encounter_base_marker(target, now, 3, yaw, risk)
        marker.type = Marker.TRIANGLE_LIST
        half = self._marker_scale * 0.3
        length = self._marker_scale * 0.35
        apex_x = half + length
        marker.points = [
            Point(x=apex_x, y=0.0, z=0.0),
            Point(x=half, y=-half, z=0.0),
            Point(x=half, y=half, z=0.0),
        ]
        return marker

    def _triangle_encounter_marker(self, target, yaw, risk, now):
        marker = self._encounter_base_marker(target, now, 0, yaw, risk)
        marker.type = Marker.TRIANGLE_LIST
        half = self._marker_scale * 0.5
        spread = self._marker_scale * 0.433
        marker.points = [
            Point(x=half, y=0.0, z=0.0),
            Point(x=-half * 0.5, y=spread, z=0.0),
            Point(x=-half * 0.5, y=-spread, z=0.0),
        ]
        return marker

    def _encounter_markers(self, target, ownship, encounter, risk, now):
        yaw = self._encounter_yaw(encounter, target, ownship)
        if encounter == EncounterType.HEAD_ON:
            return [self._pentagon_encounter_marker(target, yaw, risk, now)]
        if encounter == EncounterType.OVERTAKING:
            return [self._cylinder_encounter_marker(target, yaw, risk, now)]
        if encounter == EncounterType.CROSSING_LEFT:
            return [
                self._square_encounter_marker(target, yaw, risk, now),
                self._combo_triangle_marker(target, yaw, risk, now),
            ]
        if encounter == EncounterType.CROSSING_RIGHT:
            return [self._triangle_encounter_marker(target, yaw, risk, now)]
        return [self._sphere_encounter_marker(target, yaw, risk, now)]

    def _velocity_line_marker(self, target, ownship, risk, now):
        relative_velocity = target.motion.velocity - ownship.velocity
        speed = relative_velocity.norm()
        if speed <= 0.0:
            return None
        direction = relative_velocity * (1.0 / speed)
        line_length = speed * self._marker_velocity_scale
        start = target.motion.position
        end = start + direction * line_length

        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self._global_frame
        marker.ns = 'maritime_' + target.target_id.hex()
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        z = self._marker_z_offset
        marker.points = [
            Point(x=start.x, y=start.y, z=z),
            Point(x=end.x, y=end.y, z=z),
        ]
        marker.scale.x = self._marker_line_width
        red, green, blue = self._risk_color(risk)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.9
        return marker

    def _label_marker(self, target, result, risk, now):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self._global_frame
        marker.ns = 'maritime_' + target.target_id.hex()
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = target.motion.position.x
        marker.pose.position.y = target.motion.position.y
        marker.pose.position.z = (
            self._marker_z_offset + self._marker_label_scale + 1.0
        )
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._marker_label_scale
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = (
            f'ID {target.target_id.hex()[:8]}\n'
            f'DCPA {result.dcpa:.1f} m\n'
            f'TCPA {result.tcpa:.1f} s\n'
            f'{ENCOUNTER_NAMES.get(int(result.encounter), "UNKNOWN")} '
            f'{RISK_NAMES.get(risk, "SAFE")}'
        )
        return marker

    def _publish_markers(self, now, ownship, targets_by_id, results):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for result in results:
            target = targets_by_id.get(result.target_id)
            if target is None:
                continue
            risk = int(result.risk)
            markers.markers.extend(
                self._encounter_markers(
                    target, ownship, result.encounter, risk, now
                )
            )
            if result.cpa_valid:
                line = self._velocity_line_marker(
                    target, ownship, risk, now
                )
                if line is not None:
                    markers.markers.append(line)
            markers.markers.append(
                self._label_marker(target, result, risk, now)
            )

        self._marker_publisher.publish(markers)

    def _assess(self):
        odom_snapshot, target_snapshot = self._snapshot_inputs()
        now = self.get_clock().now()
        output = self._empty_output(now)

        if odom_snapshot is None:
            self.get_logger().warning(
                'Ownship odometry is unavailable', throttle_duration_sec=5.0
            )
            self._publisher.publish(output)
            return
        if target_snapshot is None:
            self.get_logger().warning(
                'Target ship list is unavailable', throttle_duration_sec=5.0
            )
            self._publisher.publish(output)
            return
        if snapshot_is_stale(odom_snapshot, now.nanoseconds, self._ownship_timeout):
            self.get_logger().warning(
                'Ownship odometry is stale', throttle_duration_sec=5.0
            )
            self._publisher.publish(output)
            return
        if snapshot_is_stale(target_snapshot, now.nanoseconds, self._target_timeout):
            self.get_logger().warning(
                'Target ship list is stale', throttle_duration_sec=5.0
            )
            self._publisher.publish(output)
            return

        odom = odom_snapshot[0]
        target_list = target_snapshot[0]
        odom_source_time_ns = effective_source_time_ns(*odom_snapshot)
        target_source_time_ns = effective_source_time_ns(*target_snapshot)
        try:
            ownship = self._ownship_motion(odom, odom_source_time_ns)
            ownship = propagate_motion(
                ownship,
                (now.nanoseconds - odom_source_time_ns) / 1_000_000_000.0,
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(
                f'Cannot transform ownship state: {error}',
                throttle_duration_sec=5.0,
            )
            self._publisher.publish(output)
            return

        targets = []
        seen_ids = set()
        if target_list.ships:
            try:
                target_transform = self._lookup_transform(
                    target_list.header.frame_id,
                    target_source_time_ns,
                )
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f'Cannot transform target snapshot: {error}',
                    throttle_duration_sec=5.0,
                )
                self._publisher.publish(output)
                return
        for ship in target_list.ships:
            try:
                target = self._target_motion(ship, target_transform)
                target = TrackedMotion(
                    target.target_id,
                    propagate_motion(
                        target.motion,
                        (now.nanoseconds - target_source_time_ns)
                        / 1_000_000_000.0,
                    ),
                )
                if target.target_id in seen_ids:
                    raise ValueError('duplicate target UUID')
                seen_ids.add(target.target_id)
                targets.append(target)
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f'Skipping target with invalid state or transform: {error}',
                    throttle_duration_sec=5.0,
                )

        results = build_reports(ownship, targets, self._assessment_config)
        targets_by_id = {target.target_id: target for target in targets}
        for result in results:
            report = SituationReport()
            report.target_id.uuid = list(result.target_id)
            report.cpa_valid = result.cpa_valid
            report.dcpa = result.dcpa
            report.tcpa = result.tcpa
            report.encounter_type = int(result.encounter)
            report.risk_level = int(result.risk)
            output.targets.append(report)
        self._publisher.publish(output)
        if self._enable_markers and self._marker_publisher is not None:
            self._publish_markers(now, ownship, targets_by_id, results)


def main(args=None):
    """Run the standalone, non-lifecycle monitor node."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = MaritimeSituationMonitor()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None:
            executor.shutdown()
            if node is not None:
                executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
