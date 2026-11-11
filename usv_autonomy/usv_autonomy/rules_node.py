import math
import json
import fcntl
import os
from pathlib import Path
from time import monotonic

import rclpy
from nav2_maritime_situation_msgs.msg import SituationReportArray
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from usv_interfaces.msg import MaritimeRuleDecision, AutonomyState
from usv_interfaces.srv import ResetMaritimeEvent
from .ros_helpers import fresh_stamp, observation_age
from .rules import MaritimeRules


class RulesNode(Node):
    def __init__(self):
        super().__init__('usv_maritime_rules_v2')
        def param(name, value):
            return self.declare_parameter(name, value).value
        self.rules = MaritimeRules(monotonic, param('observe_time', 5.0),
            param('clear_time', 5.0), param('observation_timeout', 1.5),
            param('dcpa_improvement', 10.0), param('release_dcpa', 40.0), param('wait_risk', 2),
            missing_target_timeout=param('missing_target_timeout', 5.0),
            stand_on_enabled=param('left_crossing_stand_on', False),
            stand_on_confirm_time=param('stand_on_confirm_time', 1.0),
            wait_tcpa=param('maritime_wait_tcpa', 30.0),
            wait_dcpa=param('maritime_wait_dcpa', 20.0),
            critical_range=param('maritime_critical_range', 10.0),
            critical_tcpa=param('maritime_critical_tcpa', 5.0),
            fault_timeout=param('maritime_fault_timeout', 5.0),
            encounter_confirm_time=param('encounter_confirm_time', 1.0),
            encounter_end_time=param('encounter_end_time', 5.0),
            encounter_release_range=param('encounter_release_range', 40.0))
        self.frame = param('global_frame', 'map')
        default_checkpoint = '~/.ros/usv_autonomy/' + self.get_namespace().strip('/') + '/maritime.json'
        self.checkpoint = Path(os.path.expanduser(param('rules_checkpoint', default_checkpoint)))
        self.checkpoint.parent.mkdir(parents=True, exist_ok=True)
        self.checkpoint_lock = self.checkpoint.with_suffix('.lock').open('a')
        fcntl.flock(self.checkpoint_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.saved_checkpoint = None
        if self.checkpoint.exists():
            data = json.loads(self.checkpoint.read_text())
            targets = data['waiting_targets']
            if not isinstance(targets, list) or not all(isinstance(t, str) for t in targets):
                raise ValueError('invalid maritime checkpoint')
            self.rules.restore_waiting(targets)
            event = data.get('event_id', '')
            reason = data.get('handover_reason', '')
            event_target = data.get('event_target', targets[0] if event and targets else '')
            if not all(isinstance(v, str) for v in (event, reason, event_target)):
                raise ValueError('invalid maritime event checkpoint')
            self.rules.event_id, self.rules.handover_reason = event, reason
            self.rules.event_target = event_target
            self.rules.restore_responsibilities(data.get('responsibilities', {}))
        self.current_encounters = {}
        self.last_policy_facts = None
        self.pub = self.create_publisher(MaritimeRuleDecision, 'autonomy/maritime', 1)
        self.task_mode, self.task_time = '', float('-inf')
        self.create_subscription(AutonomyState, 'autonomy/state', self.task_state, 1)
        self.create_service(ResetMaritimeEvent, 'autonomy/internal/reset_maritime_event', self.reset)
        self.create_subscription(SituationReportArray, 'maritime/situations', self.report, 10)
        self.timer = self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def task_state(self, msg):
        age = observation_age(self, 'task_state', msg.header)
        if age is not None:
            self.task_mode, self.task_time = msg.mode, monotonic() - age

    def report(self, msg):
        reports = []
        seen = set()
        age = observation_age(self, 'situation', msg.header, self.rules.stale)
        valid = msg.observations_valid and msg.header.frame_id == self.frame and age is not None
        for target in msg.targets:
            key = bytes(target.target_id.uuid).hex()
            if (key in seen or target.dcpa < 0 or target.current_range < 0 or
                    not all(math.isfinite(v) for v in (target.dcpa, target.tcpa, target.current_range))):
                valid = False
                break
            seen.add(key)
            reports.append((key, target.encounter_type == 3,
                            target.cpa_valid, target.dcpa, target.tcpa, target.risk_level,
                            target.current_range, target.encounter_type))
        self.rules.update(reports, valid)
        self.current_encounters = {report[0]: report[7] for report in reports} if valid else {}
        self.log_policy(valid)

    def log_policy(self, valid):
        roles = self.rules.responsibilities()
        pending = tuple(sorted(target for target, e in self.rules.encounters.items()
                               if e.head_on_since is not None and not e.responsibility))
        facts = (valid, tuple(sorted(self.current_encounters.items())), tuple(roles.items()),
                 pending, self.rules.stand_on_target, tuple(self.rules.waiting_ids()), self.rules.event_id)
        if facts != self.last_policy_facts:
            self.get_logger().info('maritime policy: valid=%s geometry=%s responsibilities=%s '
                'pending_head_on=%s stand_on=%s waiting=%s event=%s' % (valid, self.current_encounters, roles, pending,
                self.rules.stand_on_target, self.rules.waiting_ids(), self.rules.event_id))
            self.last_policy_facts = facts

    def tick(self):
        waiting, valid, clear = self.rules.decision()
        data = self.checkpoint_data()
        persisted = True
        if data != self.saved_checkpoint:
            try:
                self.persist(data)
            except OSError as exc:
                self.get_logger().error('maritime checkpoint failed: ' + str(exc))
                valid, clear = False, False
                persisted = False
        msg = MaritimeRuleDecision()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.rule_id = 'left_crossing_scene_v2'
        msg.revision = self.rules.revision
        msg.intervention_required, msg.valid, msg.clear_confirmed = waiting, valid, clear
        msg.handover_required = bool(self.rules.event_id)
        msg.event_id = self.rules.event_id
        msg.stand_on = bool(persisted and valid and self.rules.stand_on_target and not waiting
                            and self.task_mode == 'NAVIGATION' and monotonic() - self.task_time <= 1.0)
        head_on = next((target for target in sorted(self.current_encounters)
                        if self.rules.responsibilities().get(target) == 1), '')
        target = (self.rules.event_target or self.rules.stand_on_target or
                  next(iter(self.rules.waiting_ids()), '') or head_on)
        if len(target) == 32 and all(c in '0123456789abcdefABCDEF' for c in target):
            msg.target_id.uuid = list(bytes.fromhex(target))
        msg.reason = (self.rules.handover_reason if msg.handover_required else
            ('neutral_drift_wait' if waiting else ('stand_on' if msg.stand_on else
             ('head_on_responsibility' if clear and head_on else
              ('clear' if clear else 'observation_unknown')))))
        self.pub.publish(msg)

    def checkpoint_data(self):
        return dict(version=3, waiting_targets=self.rules.waiting_ids(),
                    event_id=self.rules.event_id, event_target=self.rules.event_target,
                    handover_reason=self.rules.handover_reason,
                    responsibilities=self.rules.responsibilities())

    def persist(self, data):
        temporary = self.checkpoint.with_suffix('.tmp')
        with temporary.open('w') as stream:
            json.dump(data, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.checkpoint)
        directory = os.open(str(self.checkpoint.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        self.saved_checkpoint = data

    def reset(self, request, response):
        if not request.confirmed or not request.event_id or request.event_id != self.rules.event_id:
            response.accepted, response.reason = False, 'confirmation/event mismatch'
            return response
        try:
            # Commit the durable clear before making it visible in memory.
            self.persist(dict(version=3, waiting_targets=[], event_id='', event_target='',
                              handover_reason='', responsibilities={}))
            self.rules.reset_event(request.event_id)
            response.accepted, response.reason = True, 'maritime event reset'
        except OSError as exc:
            response.accepted, response.reason = False, 'checkpoint failure: ' + str(exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RulesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
