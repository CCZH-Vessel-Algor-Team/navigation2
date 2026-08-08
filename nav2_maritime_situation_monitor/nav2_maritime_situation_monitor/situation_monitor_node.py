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

from .enu_algorithms import (
    EncounterConfig,
    propagate_motion,
    RiskThreshold,
    RiskThresholds,
    validate_risk_thresholds,
    Vector2,
    VesselMotion,
)
from .report_builder import AssessmentConfig, build_reports, TrackedMotion


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

        self._publish_frequency = self._parameter('publish_frequency')
        self._tracked_ship_topic = self._parameter('tracked_ship_topic')
        self._odom_topic = self._parameter('odom_topic')
        self._output_topic = self._parameter('output_topic')
        self._global_frame = self._parameter('global_frame')
        self._base_frame = self._parameter('base_frame')
        self._target_timeout = self._parameter('target_timeout')
        self._ownship_timeout = self._parameter('ownship_timeout')
        self._transform_timeout = self._parameter('transform_timeout')

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
