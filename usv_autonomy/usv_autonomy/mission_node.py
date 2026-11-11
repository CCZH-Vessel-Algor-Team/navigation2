"""Single-writer lifecycle service and BT primitives. No business workflow loop."""
import json
import fcntl
import math
import os
from pathlib import Path
from time import monotonic

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from usv_interfaces.action import ManageTask, ExecuteAutoTask
from usv_interfaces.msg import (
    AutonomyState, ControlAuthorityState, ExecutionEvidence, GateState,
    MaritimeRuleDecision, MotionPermit, SafetyConstraintState, TaskProgress,
)
from usv_interfaces.srv import AutonomyStep, ManageMission, NavigationControl, ResetSafetyLatch, ResetMaritimeEvent

from .lifecycle import Coordinator, FAILURE, SUCCESS, RUNNING, identity, validate_pose
from .ros_helpers import duration, fresh_stamp, observation_age, pose_dict, pose_message, seconds, bounded_call
from .store import Store
from .child_results import ChildResults
from .hold_visualization import hold_markers
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy


class Mission(Node):
    def __init__(self):
        super().__init__('usv_mission_v2')
        def param(name, default):
            return self.declare_parameter(name, default).value
        database = os.path.expanduser(param('database', '~/.ros/usv_autonomy/tasks.sqlite3'))
        Path(database).parent.mkdir(parents=True, exist_ok=True)
        self.database_lock = open(database + '.lock', 'a')
        fcntl.flock(self.database_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.store = Store(database)
        self.core = Coordinator(
            self.store, monotonic, param('cancel_timeout', 3.0),
            param('leg_retries', 3), param('task_retries', 12),
            param('recovery_timeout', 120.0), param('automatic_authority_return', False))
        self.frame = param('global_frame', 'map')
        self.base = param('base_frame', 'usv_1/base_link')
        self.pose_timeout = param('pose_timeout', 1.0)
        self.goal_tolerance = param('goal_tolerance', 1.5)
        self.tree = param('navigation_tree', '')
        self.through_tree = param('navigation_through_tree', '')
        self.stand_on_enabled = param('left_crossing_stand_on', False)
        self.stand_on_tree = param('stand_on_navigation_tree', '')
        self.stand_on_through_tree = param('stand_on_navigation_through_tree', '')
        self.desired_stand_on = self.active_stand_on = False
        self.policy_changed_at = monotonic()
        self.planner_settle_time = param('planner_switch_settle_time', 0.5)
        if not math.isfinite(self.planner_settle_time) or self.planner_settle_time < 0:
            raise ValueError('planner_switch_settle_time must be finite and nonnegative')
        self.handover_event = ''
        self.manual_authority_time = float('-inf')
        self.manual_modes = param('manual_takeover_modes', ['MANUAL', 'SIM_MANUAL'])
        self.handover_future = None
        self.handover_sent_at = float('-inf')
        self.handover_timeout = param('handover_timeout', 5.0)
        self.handover_retry_interval = param('handover_retry_interval', 5.0)
        for value in (self.handover_timeout, self.handover_retry_interval):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('handover timing must be finite and positive')
        client_group = ReentrantCallbackGroup()
        self.manual_client = self.create_client(SetBool, 'control/request_manual', callback_group=client_group)
        self.event_reset_client = self.create_client(ResetMaritimeEvent,
            'autonomy/internal/reset_maritime_event', callback_group=client_group)
        self.create_service(ResetSafetyLatch, 'autonomy/internal/reset_maritime', self.reset_maritime,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.via_radius = param('via_radius', 8.0)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_many = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.child_results = ChildResults(self, self.core)
        self.backend_client = self.create_client(GetState, 'bt_navigator/get_state')
        self.backend_future = None
        self.backend_state = None
        self.backend_received = float('-inf')
        self.backend_query_at = 0.
        self.nav_handle = None
        self.cancel_sent = False
        self.nav_result = None
        self.goal_data = None
        self.navigation_started = None
        self.hold_clock_task = None
        self.hold_last_tick = monotonic()
        self.hold_last_save = monotonic()
        self.pose = None
        self.pose_time = float('-inf')
        self.authority_time = self.safety_time = self.rules_time = float('-inf')
        self.bt_time = float('-inf')
        self.safe_speed = 0.0
        self.regular_speed = 0.0
        self.regular_time = float('-inf')
        self.safety_allowed = False
        self.rules_wait, self.rules_valid = False, False
        self.gate_boot = ''
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.state_pub = self.create_publisher(AutonomyState, 'autonomy/state', 10)
        self.hold_marker_pub = self.create_publisher(MarkerArray, 'autonomy/hold_region',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.hold_marker_time = float('-inf')
        self.permit_pub = self.create_publisher(MotionPermit, 'autonomy/permit', 1)
        self.progress_pub = self.create_publisher(TaskProgress, 'task_progress', 10)
        self.create_service(ManageMission, 'autonomy/manage', self.manage)
        self.create_service(AutonomyStep, 'autonomy/step', self.step)
        self.create_service(NavigationControl, 'autonomy/navigation_control', self.navigation_control)
        self.create_subscription(ControlAuthorityState, 'control/authority', self.authority, 1)
        self.create_subscription(SafetyConstraintState, 'autonomy/safety', self.safety, 1)
        self.create_subscription(SafetyConstraintState, 'autonomy/regular_safety', self.regular, 1)
        self.create_subscription(MaritimeRuleDecision, 'autonomy/maritime', self.rules, 1)
        self.create_subscription(ExecutionEvidence, 'autonomy/producer', self.producer, 10)
        self.create_subscription(GateState, 'autonomy/gate_state', self.gate, 1)
        self.create_subscription(Bool, 'autonomy/bt_heartbeat', self.bt_heartbeat, 1)
        if self.declare_parameter('enable_simulation_tests', False,
                descriptor=ParameterDescriptor(read_only=True)).value:
            self.create_subscription(Bool, 'autonomy/test_maritime_wait',
                lambda msg: self.core.set_blocker('maritime_test', msg.data), 1)
        self.manage_action = ActionServer(self, ManageTask, 'manage_task', self.manage_action_execute)
        self.legacy_action = ActionServer(
            self, ExecuteAutoTask, 'execute_auto_task', self.legacy_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('v2 lifecycle boot=' + self.core.boot + ' database=' + database)

    def authority(self, msg):
        age = observation_age(self, 'authority', msg.header)
        if age is None:
            return
        valid = msg.valid and msg.connected
        self.manual_authority_time = (monotonic() - age if valid and msg.authority == msg.EXTERNAL
                                      and msg.mode in self.manual_modes
                                      and not msg.motion_allowed else float('-inf'))
        self.authority_time = monotonic() - age if valid else float('-inf')
        self.core.set_authority(valid and msg.authority == msg.AUTONOMOUS
                                and msg.motion_allowed and msg.armed)

    def safety(self, msg):
        age = observation_age(self, 'safety', msg.header, min(seconds(msg.valid_for), 1.0))
        if age is None:
            return
        self.safety_time = monotonic() - age
        self.safety_allowed = msg.task_allowed
        self.safe_speed = msg.max_speed if math.isfinite(msg.max_speed) else 0.0
        self.core.set_blocker('emergency', msg.level == msg.EMERGENCY or msg.emergency_latched)
        self.core.set_blocker('safety', not msg.task_allowed)

    def rules(self, msg):
        age = observation_age(self, 'rules', msg.header)
        if age is None:
            return
        self.rules_time = monotonic() - age
        self.rules_valid = msg.valid
        if msg.handover_required:
            if msg.event_id != self.handover_event:
                self.handover_sent_at = float('-inf')
                self.handover_future = None
            self.handover_event = msg.event_id
            # Explicit Continue remains required even if authority was already
            # external when this event arrived, or auto authority return is on.
            if self.core.task and not self.core.task.get('return_confirmation'):
                self.core.task['return_confirmation'] = True
                self.core._persist('maritime_handover_latched')
        elif msg.valid and not msg.intervention_required and msg.clear_confirmed:
            self.handover_event = ''
        self.core.set_blocker('maritime_handover', bool(self.handover_event))
        desired = bool(self.stand_on_enabled and msg.valid and msg.stand_on
                       and not msg.intervention_required and not self.handover_event)
        if desired != self.desired_stand_on:
            self.policy_changed_at = monotonic()
        self.desired_stand_on = desired
        if self.desired_stand_on != self.active_stand_on:
            self.core.set_blocker('planner_switch', True)
        # A non-intervention sample is not a clear acknowledgement.
        if msg.intervention_required:
            self.rules_wait = True
        elif msg.valid and msg.clear_confirmed:
            self.rules_wait = False
        self.core.set_blocker('maritime', self.rules_wait)
        self.core.set_blocker('rules_unknown', not msg.valid)

    async def reset_maritime(self, request, response):
        event = self.handover_event
        if monotonic() - self.rules_time > 1.0:
            response.accepted, response.reason = False, 'fresh maritime decision required'
            return response
        if not event:
            response.accepted, response.reason = True, 'no maritime handover event'
            return response
        if (not request.confirmed or monotonic() - self.rules_time > 1.0
                or monotonic() - self.manual_authority_time > 1.0 or not self.core.quiet()):
            response.accepted, response.reason = False, 'fresh manual authority and quiescence required'
            return response
        try:
            # Persist the resume barrier BEFORE clearing the rules checkpoint.
            if self.core.task:
                self.core.task['return_confirmation'] = True
                self.core.task['intent'] = 'PAUSE'
                self.core._persist('maritime_manual_reset')
            result = await bounded_call(self, self.event_reset_client,
                ResetMaritimeEvent.Request(confirmed=True, event_id=event), 2.0)
            if not result.accepted:
                response.accepted, response.reason = False, result.reason
                return response
            if (self.handover_event != event or not self.core.quiet()
                    or monotonic() - self.manual_authority_time > 1.0):
                response.accepted, response.reason = False, 'conditions changed; autonomous resume remains blocked'
                return response
            self.handover_event = ''
            self.rules_wait = False
            self.core.set_blocker('maritime_handover', False)
            self.core.set_blocker('maritime', False)
            self.core.revision += 1
            response.accepted, response.reason = True, 'maritime reset; explicit Continue required'
        except Exception as exc:
            response.accepted, response.reason = False, str(exc)
        return response

    def regular(self, msg):
        age = observation_age(self, 'regular', msg.header, min(seconds(msg.valid_for), 1.))
        if age is None:
            return
        self.regular_time = monotonic() - age
        self.regular_speed = msg.max_speed if math.isfinite(msg.max_speed) else 0.
        self.core.set_blocker('regular_safety', not msg.task_allowed or msg.emergency_latched)

    def producer(self, msg):
        age = observation_age(self, 'producer', msg.header)
        if msg.role == 'controller' and age is not None:
            facts = (msg.execution_id, msg.attempt_id, msg.epoch, msg.active, msg.boot_id)
            if self.core.producer is None or facts != self.core.producer[:5]:
                self.core.audit('producer_state', dict(execution=msg.execution_id,
                    attempt=msg.attempt_id, epoch=msg.epoch, active=msg.active, boot=msg.boot_id))
            self.core.producer_update(msg.execution_id, msg.attempt_id, msg.epoch,
                                      msg.active, msg.boot_id)
            self.core.producer = self.core.producer[:5] + (monotonic() - age,)

    def gate(self, msg):
        if fresh_stamp(self, msg.header):
            if self.gate_boot and self.gate_boot != msg.boot_id:
                self.core.stop(cause='gate_restarted')
                self.core.revision += 1
            self.gate_boot = msg.boot_id
            self.core.acknowledge_gate(msg.permit_boot_id, msg.permit_revision, msg.permitted)
            if (self.core.permitted and msg.permit_boot_id == self.core.boot
                    and msg.permit_revision >= self.core.permit_revision and not msg.permitted):
                self.core.stop(cause='gate_permission_lost')
                self.core.revision += 1

    def bt_heartbeat(self, msg):
        if msg.data:
            self.bt_time = monotonic()

    def specification(self, mode, goals, hold):
        mode = mode.upper()
        data = [pose_dict(p) for p in goals]
        for goal in data:
            validate_pose(goal)
            if goal['frame'] != self.frame:
                raise ValueError('goals must be in configured global_frame')
        h = dict(inner=hold.inner_radius, outer=hold.outer_radius,
                 speed=hold.return_speed, timeout=hold.return_timeout_s,
                  retries=hold.return_retries, retry_delay=hold.retry_delay_s,
                  duration=hold.duration_s, anchor=None, use_current_pose=hold.use_current_pose)
        if mode == 'HOLD':
            if not all(math.isfinite(h[k]) for k in ('inner', 'outer', 'speed', 'timeout', 'duration')):
                raise ValueError('nonfinite HOLD parameters')
            if not (0 < h['inner'] < h['outer'] and h['speed'] > 0 and h['timeout'] > 0):
                raise ValueError('invalid HOLD radius/speed/timeout')
            if h['inner'] < self.goal_tolerance:
                raise ValueError('HOLD inner radius must cover the configured navigation arrival tolerance')
            if not hold.use_current_pose:
                h['anchor'] = pose_dict(hold.anchor)
                validate_pose(h['anchor'])
                if h['anchor']['frame'] != self.frame:
                    raise ValueError('HOLD anchor frame mismatch')
            else:
                if self.pose is None or monotonic() - self.pose_time > self.pose_timeout:
                    raise ValueError('no fresh current pose for HOLD preflight')
                # Capture at mode selection, not at Continue / first BT tick.
                h['anchor'] = dict(frame=self.frame, position=list(self.pose),
                                   orientation=[0.0, 0.0, 0.0, 1.0])
        return dict(mode=mode, goals=data, hold=h)

    def manage(self, request, response):
        try:
            if request.expected_boot_id != self.core.boot:
                response.accepted, response.code, response.reason = False, 409, 'coordinator boot mismatch'
                response.revision = self.core.revision
                return response
            spec = None
            if request.command in ('start', 'switch'):
                spec = self.specification(request.mode, request.goals, request.hold)
                spec['loop'] = request.loop_execution
                spec['policy'] = request.waypoint_policy.upper() or 'REQUIRED'
                if spec['policy'] not in ('REQUIRED', 'VIA'):
                    raise ValueError('unknown waypoint_policy')
                spec['speeds'] = list(request.speeds)
                if spec['speeds'] and len(spec['speeds']) != len(spec['goals']):
                    raise ValueError('speeds/goals size mismatch')
                if any(not math.isfinite(v) or v < 0 for v in spec['speeds']):
                    raise ValueError('invalid speed')
            response.accepted, response.code, response.reason = self.core.manage(
                request.command, request.task_id, spec, request.request_id, request.expected_revision)
        except (ValueError, KeyError) as exc:
            response.accepted, response.code, response.reason = False, 400, str(exc)
        except Exception as exc:
            self.fail_storage(exc)
            response.accepted, response.code, response.reason = False, 500, str(exc)
        response.revision = self.core.revision
        return response

    async def manage_action_execute(self, handle):
        req = handle.request
        expected = self.core.revision
        result = ManageTask.Result()
        if req.expected_boot_id != self.core.boot:
            result.code, result.message = 409, 'coordinator boot mismatch'
            handle.abort()
            return result
        if req.expected_task_id and (not self.core.task or req.expected_task_id != self.core.task['id']):
            result.code, result.message = 409, 'active task conflict'
            handle.abort()
            return result
        try:
            goals = await self.geographic_goals(req.task.waypoints)
            spec = self.specification(req.task.mode, goals, req.task.hold) if req.command in ('start', 'switch') else None
            if spec:
                spec['loop'] = req.task.loop_execution
                spec['policy'] = req.task.waypoint_policy.upper()
                if spec['policy'] not in ('REQUIRED', 'VIA'):
                    raise ValueError('unknown waypoint_policy')
                spec['speeds'] = [w.speed_target for w in req.task.waypoints]
            result.success, result.code, result.message = self.core.manage(
                req.command, req.target_task_id or req.task.task_id, spec, req.request_id,
                req.expected_revision or expected)
        except Exception as exc:
            result.success, result.code, result.message = False, 400, str(exc)
        result.active_task_id = self.core.task['id'] if self.core.task else ''
        result.active_mode = self.core.task['spec']['mode'] if self.core.task else ''
        result.phase = ('TRANSITIONING' if self.core.pending_task or self.core.cancel_since is not None
                        else (self.core.task['state'] if self.core.task else 'IDLE'))
        handle.succeed() if result.success else handle.abort()
        return result

    async def geographic_goals(self, waypoints):
        if not waypoints:
            return []
        # Use the localization node's actual georeference, never a second local origin.
        from robot_localization.srv import FromLL
        client = self.create_client(FromLL, 'fromLL')
        if not client.service_is_ready():
            self.destroy_client(client)
            raise ValueError('fromLL georeference service unavailable')
        goals = []
        try:
            for waypoint in waypoints:
                if (not all(math.isfinite(v) for v in (waypoint.latitude, waypoint.longitude,
                                                       waypoint.heading_target, waypoint.speed_target))
                        or not -90 <= waypoint.latitude <= 90
                        or not -180 <= waypoint.longitude <= 180 or waypoint.speed_target < 0):
                    raise ValueError('invalid geographic waypoint')
                request = FromLL.Request()
                request.ll_point.latitude, request.ll_point.longitude = waypoint.latitude, waypoint.longitude
                future = client.call_async(request)
                # Bound conversion with a steady timer; delayed answers cannot activate a task.
                timer = self.create_timer(2.0, lambda f=future: f.cancel() if not f.done() else None,
                                          clock=Clock(clock_type=ClockType.STEADY_TIME))
                try:
                    converted = await future
                    if converted is None:
                        raise ValueError('fromLL conversion timed out')
                finally:
                    self.destroy_timer(timer)
                pose = pose_message(dict(frame=self.frame,
                    position=[converted.map_point.x, converted.map_point.y, converted.map_point.z],
                    orientation=[0.0, 0.0, math.sin(waypoint.heading_target / 2),
                                 math.cos(waypoint.heading_target / 2)]))
                goals.append(pose)
        finally:
            self.destroy_client(client)
        return goals

    async def legacy_execute(self, handle):
        req = handle.request
        result = ExecuteAutoTask.Result()
        result.task_id = req.task_id
        expected = self.core.revision
        try:
            spec = None
            if req.command == 'start':
                goals = await self.geographic_goals(req.waypoints)
                from usv_interfaces.msg import HoldTaskOptions
                spec = self.specification(req.mode or 'NAVIGATION', goals, HoldTaskOptions())
                spec['loop'] = req.loop_execution
                spec['policy'] = 'REQUIRED'
                spec['speeds'] = [w.speed_target for w in req.waypoints]
            if handle.is_cancel_requested:
                result.success, result.code, result.message = False, 499, 'management canceled before activation'
                result.final_state = TaskProgress.STATE_CANCELLED
                handle.canceled()
                return result
            result.success, result.code, result.message = self.core.manage(
                req.command, req.task_id, spec, identity(), expected)
        except Exception as exc:
            result.success, result.code, result.message = False, 400, str(exc)
        # This adapter acknowledges management; task completion is on task_progress.
        result.final_state = TaskProgress.STATE_ACCEPTED if result.success else TaskProgress.STATE_FAILED
        handle.succeed() if result.success else handle.abort()
        return result

    def navigation_control(self, request, response):
        try:
            if request.operation == 'passed':
                task = self.core.task
                index = int(request.key)
                if not task or not 0 <= index < len(task['spec']['goals']):
                    response.status = FAILURE
                    return response
                if (self.pose is None or monotonic() - self.pose_time > self.pose_timeout
                        or (index >= task['index'] and self.distance(task['spec']['goals'][index]) > self.via_radius)):
                    response.status = RUNNING
                    return response
            response.status = self.core.navigation_control(
                request.operation, request.execution_id, request.attempt_id,
                request.epoch, request.key, request.outcome, request.retryable, request.goal_id)
            if request.operation == 'accepted' and response.status == SUCCESS:
                self.child_results.watch(request)
        except Exception as exc:
            self.fail_storage(exc)
            response.status = FAILURE
        response.reason = self.core.reason
        response.boot_id, response.attempt_id, response.epoch = (
            self.core.boot, self.core.attempt, self.core.epoch)
        response.via_radius = self.via_radius
        return response

    def start_navigation(self, goal):
        task = self.core.task
        batch = task['spec']['mode'] == 'NAVIGATION' and task['spec'].get('policy') == 'VIA'
        client = self.nav_many if batch else self.nav
        if not client.server_is_ready():
            self.core.reason = '等待导航 Action 服务就绪'
            return RUNNING
        execution = self.core.execution
        request = NavigateThroughPoses.Goal() if batch else NavigateToPose.Goal()
        if batch:
            request.poses = [pose_message(g) for g in task['spec']['goals'][task['index']:]]
            request.usv_start_index = task['index']
        else:
            request.pose = pose_message(goal)
            request.pose.header.stamp = self.get_clock().now().to_msg()
        stand_on = self.active_stand_on and task['spec']['mode'] == 'NAVIGATION'
        request.behavior_tree = ((self.stand_on_through_tree if batch else self.stand_on_tree)
                                 if stand_on else (self.through_tree if batch else self.tree))
        if stand_on and not request.behavior_tree:
            self.core.set_blocker('stand_on_tree_missing', True)
            return FAILURE
        request.usv_execution_id = execution
        self.core.outer_pending, self.core.outer_terminal = True, False
        self.cancel_sent, self.nav_result = False, None
        self.navigation_started = monotonic()
        self.nav_handle = None
        future = client.send_goal_async(request)
        def accepted(f):
            try:
                handle = f.result()
                self.core.audit('outer_goal_response', {'execution': execution, 'accepted': handle.accepted})
                self.core.outer_pending = False
                if not handle.accepted:
                    self.core.outer_terminal = True
                    self.nav_result = 'REJECTED'
                    return
                self.nav_handle = handle
                result_future = handle.get_result_async()
                def finished(r):
                    wrapped = r.result()
                    self.core.audit('outer_terminal', {'execution': execution, 'status': wrapped.status})
                    self.core.outer_terminal = True
                    if execution == self.core.execution and (
                            not self.core.frozen or self.core.cancel_scope == 'ATTEMPT'):
                        self.nav_result = ('SUCCEEDED' if wrapped.status == GoalStatus.STATUS_SUCCEEDED
                                           else 'FAILED')
                        self.core.result = self.nav_result
                result_future.add_done_callback(finished)
                if execution != self.core.execution or self.core.frozen:
                    self.cancel_navigation()
            except Exception as exc:
                self.core.fault = 'uncertain goal acceptance: ' + str(exc)
                self.core.stop(cause=self.core.fault)
        future.add_done_callback(accepted)
        return RUNNING

    def cancel_navigation(self):
        if self.nav_handle and not self.cancel_sent and not self.core.outer_terminal:
            self.cancel_sent = True
            self.nav_handle.cancel_goal_async()  # Response is not a terminal/idle acknowledgement.

    def step(self, request, response):
        if request.expected_boot_id != self.core.boot or request.expected_revision != self.core.revision:
            response.status, response.reason = FAILURE, 'stale BT revision'
            return response
        try:
            response.status = self.primitive(request.operation)
            response.reason = self.core.reason
        except Exception as exc:
            self.fail_storage(exc)
            response.status, response.reason = FAILURE, str(exc)
        return response

    def primitive(self, op):
        c, task = self.core, self.core.task
        if op == 'quiesce':
            c.stop(cause='BT quiesce')
            self.cancel_navigation()
            return SUCCESS if c.quiet() else RUNNING
        if op == 'request_manual':
            return self.request_manual()
        if op.startswith('wait_'):
            return RUNNING
        if not task or not c.runnable():
            return FAILURE
        if op == 'next_goal':
            if task['index'] >= len(task['spec']['goals']):
                return FAILURE
            previous = c.execution
            status = c.activate()
            if status == SUCCESS:
                self.goal_data = task['spec']['goals'][task['index']]
                if task['spec'].get('policy') == 'VIA':
                    self.goal_data = task['spec']['goals'][-1]
                if c.execution != previous:
                    self.nav_result = None
                    self.nav_handle = None
                    self.navigation_started = None
            return status
        if op in ('navigate', 'hold_return'):
            if c.execution and c.frozen and c.cancel_scope == 'ATTEMPT':
                if c.outer_terminal and self.nav_result in ('FAILED', 'REJECTED'):
                    return FAILURE  # A failed recovery is not an indefinite local wait.
                return RUNNING  # Nav2 BT still owns its local stop/Wait/replan.
            if not c.execution or c.frozen:
                return FAILURE
            if (op == 'hold_return' and self.pose is not None
                    and monotonic() - self.pose_time <= self.pose_timeout
                    and self.distance(task['hold_anchor']) <= task['spec']['hold']['inner']):
                # Regional HOLD ends its return at the inner boundary, without
                # waiting for Nav2's point/yaw goal checker or another replan.
                return SUCCESS
            if self.nav_result:
                if self.nav_result != 'SUCCEEDED':
                    return FAILURE
                if not self.pose or self.distance(self.goal_data) > self.goal_tolerance:
                    c.reason = 'arrival evidence does not match target'
                    return FAILURE
                if (task['spec'].get('policy') == 'VIA' and task['spec']['mode'] == 'NAVIGATION'
                        and task['index'] != len(task['spec']['goals']) - 1):
                    c.reason = 'batch finished with unconfirmed intermediate goals'
                    return FAILURE
                return SUCCESS
            if c.outer_terminal and not c.outer_pending:
                return self.start_navigation(self.goal_data)
            return RUNNING
        if op == 'commit':
            return c.commit_waypoint(c.execution)
        if op == 'complete':
            if task['spec']['mode'] == 'NAVIGATION' and task['index'] < len(task['spec']['goals']):
                return FAILURE
            if task['spec']['mode'] == 'NAVIGATION' and task['spec'].get('loop'):
                task['index'], task['completed'] = 0, []
                task['lap'] = task.get('lap', 0) + 1
                task['checkpoint'] += 1
                c._persist('lap_completed')
                c.stop(cause='lap_completed')
                c.revision += 1
                return SUCCESS
            task['state'], task['intent'] = 'COMPLETED', 'END'
            task['return_confirmation'] = False
            c._persist('task_completed')
            c.stop(cause='completed')
            c.revision += 1
            return SUCCESS
        if op == 'execution_failed':
            c.set_blocker('execution_failed', True)
            return RUNNING
        if op == 'hold_anchor':
            if not self.pose:
                return FAILURE
            if task['hold_anchor'] is None:
                task['hold_anchor'] = dict(frame=self.frame, position=list(self.pose),
                                           orientation=[0.0, 0.0, 0.0, 1.0])
                c._persist('hold_anchor_saved')
            return SUCCESS
        if op == 'hold_outside':
            return SUCCESS if self.distance(task['hold_anchor']) >= task['spec']['hold']['outer'] else FAILURE
        if op == 'hold_prepare':
            previous = c.execution
            status = c.activate()
            if status == SUCCESS and c.execution != previous:
                self.goal_data = task['hold_anchor']
                self.nav_result = None
                self.nav_handle = None
                self.navigation_started = None
            return status
        if op == 'hold_finish':
            if not c.execution:
                return FAILURE
            if task.get('hold_last_return') != c.execution:
                task['hold_returns'] += 1
                task['hold_last_return'] = c.execution
                task['reservation'] = None
                c._persist('hold_return_completed')
            c.stop(cause='hold_return_completed')
            self.cancel_navigation()
            self.navigation_started = None
            return SUCCESS
        if op == 'hold_monitor':
            # No active holding torque: region monitoring is a distinct behavior.
            return RUNNING
        return FAILURE

    def distance(self, goal):
        return math.hypot(self.pose[0] - goal['position'][0], self.pose[1] - goal['position'][1]) if self.pose else float('inf')

    def fail_storage(self, exc):
        self.core.fault = 'persistence_or_execution_error: ' + str(exc)
        # Do not recursively access a failed database from the emergency path.
        self.core.permitted, self.core.frozen = False, True
        self.core.permit_revision += 1
        self.core.revision += 1
        self.core.cancel_scope = 'EXECUTION'
        if self.core.cancel_since is None:
            self.core.cancel_since = monotonic()
        self.cancel_navigation()
        self.get_logger().error(self.core.fault)

    def tick(self):
        now = monotonic()
        if self.backend_future is not None and now - self.backend_query_at > 2.:
            self.backend_client.remove_pending_request(self.backend_future)
            self.backend_future = None
            self.backend_state = None
        if self.backend_future is None and now - self.backend_query_at >= .5 and self.backend_client.service_is_ready():
            future = self.backend_client.call_async(GetState.Request())
            self.backend_future, self.backend_query_at = future, now
            def backend_done(result):
                if result is not self.backend_future:
                    return
                self.backend_future = None
                try:
                    self.backend_state = result.result().current_state.id
                    self.backend_received = monotonic()
                except Exception:
                    self.backend_state = None
            future.add_done_callback(backend_done)
        try:
            transform = self.tf.lookup_transform(self.frame, self.base, rclpy.time.Time())
            if fresh_stamp(self, transform.header, self.pose_timeout):
                p = transform.transform.translation
                if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                    self.pose, self.pose_time = (p.x, p.y, p.z), now
        except TransformException:
            pass
        c = self.core
        if (c.blockers.get('planner_switch') and c.quiet()
                and now - self.policy_changed_at >= self.planner_settle_time):
            self.active_stand_on = self.desired_stand_on
            c.set_blocker('planner_switch', False)
        c.set_blocker('navigation_backend', self.backend_state != LifecycleState.PRIMARY_STATE_ACTIVE
                      or now - self.backend_received > 2.)
        holding = c.task if c.task and c.task['spec']['mode'] == 'HOLD' and c.runnable() else None
        if self.hold_clock_task:
            old = self.hold_clock_task
            old['hold_elapsed'] = old.get('hold_elapsed', 0.0) + now - self.hold_last_tick
            if holding is not old or now - self.hold_last_save >= 1.0:
                try:
                    with self.store.transaction():
                        self.store.save(old, 'hold_duration_checkpoint')
                    self.hold_last_save = now
                except Exception as exc:
                    self.fail_storage(exc)
        self.hold_clock_task, self.hold_last_tick = holding, now
        if (holding and self.navigation_started is not None and not c.outer_terminal
                and now - self.navigation_started > holding['spec']['hold']['timeout']):
            c.set_blocker('hold_timeout', True)
        if now - self.authority_time > 1.0:
            c.set_authority(False)
        c.set_blocker('safety_stale', now - self.safety_time > 1.0)
        c.set_blocker('regular_safety_stale', now - self.regular_time > .5)
        c.set_blocker('rules_unknown', now - self.rules_time > 1.0 or not self.rules_valid)
        c.set_blocker('position', now - self.pose_time > self.pose_timeout)
        c.set_blocker('bt_heartbeat', now - self.bt_time > 1.0)
        c.set_blocker('startup', not c.producer or not self.gate_boot)
        c.set_blocker('producer_stale', not c.producer or now - c.producer[5] > 1.0)
        if c.frozen and c.cancel_scope == 'EXECUTION':
            self.cancel_navigation()
        try:
            self.child_results.tick()
            c.tick()
        except Exception as exc:
            self.fail_storage(exc)
        self.publish()

    def request_manual(self):
        if not self.handover_event:
            return FAILURE
        if monotonic() - self.manual_authority_time <= 1.0:
            self.core.reason = 'manual takeover confirmed; safety reset required'
            return SUCCESS
        if not self.core.quiet():
            return RUNNING
        now = monotonic()
        if self.handover_future is not None:
            if not self.handover_future.done() and now - self.handover_sent_at < self.handover_timeout:
                return RUNNING
            if not self.handover_future.done():
                self.manual_client.remove_pending_request(self.handover_future)
                self.handover_future.cancel()
            self.handover_future = None
        if now - self.handover_sent_at < self.handover_retry_interval:
            return RUNNING
        self.handover_sent_at = now
        if self.manual_client.service_is_ready():
            self.handover_future = self.manual_client.call_async(SetBool.Request(data=True))
        self.core.reason = 'manual takeover requested; waiting for actual authority feedback'
        self.get_logger().warning(self.core.reason)
        return RUNNING

    def publish(self):
        c, task = self.core, self.core.task
        state = AutonomyState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.boot_id, state.revision = c.boot, c.revision
        state.execution_id, state.attempt_id, state.epoch = c.execution, c.attempt, c.epoch
        state.blockers = sorted(c.blockers)
        if c.fault:
            state.blockers.append('fault')
        if c.fault:
            state.reason = c.fault + '; ' + '; '.join(c.exit_obstacles())
        elif c.cancel_since is not None:
            state.reason = '; '.join(c.exit_obstacles(c.cancel_scope == 'EXECUTION')) or c.reason
        elif task and task['intent'] == 'PAUSE':
            state.reason = '人工暂停；点击继续重新校验准入并规划未完成目标'
        elif c.blockers.get('navigation_backend'):
            state.reason = '等待 bt_navigator active；当前生命周期状态: ' + str(self.backend_state)
        elif task and task['spec']['mode'] == 'HOLD' and c.runnable() and not c.execution:
            state.reason = 'HOLD 持续监测；越过外半径后返航，进入内半径后撤权并继续监测'
        else:
            state.reason = c.reason
        state.phase = 'FAULT' if c.fault else (
            ('TRANSITIONING' if c.cancel_scope == 'EXECUTION' else 'RECOVERING')
            if c.cancel_since is not None else (
                ('RECOVERING' if c.frozen and c.cancel_scope == 'ATTEMPT' else 'ACTIVE')
                if c.execution else 'READY'))
        if task:
            state.task_id, state.mode, state.user_intent = task['id'], task['spec']['mode'], task['intent']
            if task.get('return_confirmation'):
                state.blockers.append('authority_return_confirmation')
            if task.get('budget_uncertain'):
                state.blockers.append('budget_uncertain')
            state.waypoint_index, state.waypoint_count = task['index'], len(task['spec']['goals'])
            state.lap = task.get('lap', 0)
            if state.mode == 'HOLD':
                limit = task['spec']['hold']['duration']
                state.hold_duration_done = limit > 0 and task.get('hold_elapsed', 0.0) >= limit
            if c.fault:
                state.phase = 'FAULT'
            elif not c.runnable() and state.user_intent == 'RUN':
                state.phase = 'SUSPENDED'
            elif task['intent'] == 'PAUSE' and c.cancel_since is None:
                state.phase = 'PAUSED'
            elif task['state'] in ('COMPLETED', 'CANCELED') and not c.execution:
                state.phase = task['state']
        self.state_pub.publish(state)
        now = monotonic()
        if now - self.hold_marker_time >= .2:
            self.hold_marker_pub.publish(hold_markers(task, state.header.stamp))
            self.hold_marker_time = now
        permit = MotionPermit()
        permit.header = state.header
        permit.boot_id, permit.gate_boot_id = c.boot, self.gate_boot
        permit.execution_id, permit.attempt_id, permit.epoch = c.execution, c.attempt, c.epoch
        permit.revision = c.permit_revision
        permit.active = bool(c.permitted and c.runnable())
        permit.max_speed = max(0.0, min(self.safe_speed, self.regular_speed))
        if task and task['spec']['mode'] == 'HOLD':
            permit.max_speed = min(permit.max_speed, task['spec']['hold']['speed'])
        elif task and task['index'] < len(task['spec'].get('speeds', [])):
            target_speed = task['spec']['speeds'][task['index']]
            if target_speed > 0:
                permit.max_speed = min(permit.max_speed, target_speed)
        duration(permit.valid_for, 0.5)
        self.permit_pub.publish(permit)
        if task:
            progress = TaskProgress()
            progress.header = state.header
            progress.task_id = task['id']
            progress.current_waypoint_index = task['index']
            progress.progress_percent = 100.0 * task['index'] / max(1, len(task['spec']['goals']))
            progress.state = dict(COMPLETED=4, CANCELED=6, PAUSED=3, RUNNING=2).get(task['state'], 1)
            if task['state'] not in ('COMPLETED', 'CANCELED') and not c.runnable():
                progress.state = progress.STATE_PAUSED
            progress.status_text = state.phase + ': ' + state.reason
            self.progress_pub.publish(progress)


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    finally:
        node.core.stop(cause='shutdown')
        node.store.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
