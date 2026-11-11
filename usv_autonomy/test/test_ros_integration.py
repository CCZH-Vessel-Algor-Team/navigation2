"""Real task BT + real patched Nav2 navigator + fake hardware/action servers.

Run only in the isolated Humble v2 overlay. The fake controller is deliberately
separate from the gate; it can replay old identities and delay cancellation.
"""
import os
import signal
import subprocess
from pathlib import Path
from time import monotonic

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses, FollowPath, Wait
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from tf2_ros import TransformBroadcaster
from usv_interfaces.msg import ActuatorCommand, ControlAuthorityState, ExecutionEvidence, ManagedMotion, MaritimeRuleDecision, SafetyConstraintState
from usv_interfaces.srv import ManageMission
from usv_interfaces.action import ExecuteAutoTask
from usv_interfaces.msg import Waypoint
from robot_localization.srv import FromLL
from usv_autonomy.gate_node import MotionGate
from usv_autonomy.mission_node import Mission
from usv_autonomy.rules import MaritimeRules


class Peers(Node):
    def __init__(self):
        super().__init__('fault_injection_peer')
        self.group = ReentrantCallbackGroup()
        self.authority_allowed = True
        self.waiting = False
        self.finish = False
        self.active = False
        self.current = None
        self.generation = 0
        self.sequence = 0
        self.plans = 0
        self.planner_ids = []
        self.stand_on = False
        self.maritime_policy = None
        self.encounter_report = None
        self.fail_plans = 0
        self.plan_delay = 0.
        self.planning_active = False
        self.wait_calls = 0
        self.fail_wait = False
        self.regular_allowed, self.regular_cap = True, 1.
        self.pose_x = 0.0
        self.last_motion = None
        self.commands = []
        self.geo_delay = 0.
        self.tf = TransformBroadcaster(self)
        self.authority_pub = self.create_publisher(ControlAuthorityState, 'control/authority', 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)
        self.evidence_pub = self.create_publisher(ExecutionEvidence, 'autonomy/producer', 10)
        self.rule_pub = self.create_publisher(MaritimeRuleDecision, 'autonomy/maritime', 1)
        self.regular_pub = self.create_publisher(SafetyConstraintState, 'autonomy/regular_safety', 1)
        self.motion_pub = self.create_publisher(ManagedMotion, 'autonomy/motion_raw', 1)
        self.create_subscription(ActuatorCommand, 'control/actuator_command', self.commands.append, 10)
        self.manage_client = self.create_client(ManageMission, 'autonomy/manage', callback_group=self.group)
        self.servers = [
            ActionServer(self, ComputePathToPose, 'compute_path_to_pose', self.plan, callback_group=self.group,
                         cancel_callback=lambda _: CancelResponse.ACCEPT),
            ActionServer(self, ComputePathThroughPoses, 'compute_path_through_poses', self.plan_many,
                         callback_group=self.group),
            ActionServer(self, FollowPath, 'follow_path', self.follow, callback_group=self.group,
                         cancel_callback=lambda _: CancelResponse.ACCEPT),
            ActionServer(self, Wait, 'wait', self.wait, callback_group=self.group,
                         cancel_callback=lambda _: CancelResponse.ACCEPT),
        ]
        self.create_timer(.05, self.inputs, callback_group=self.group)
        self.create_service(FromLL, 'fromLL', self.convert, callback_group=self.group)

    async def convert(self, request, response):
        if self.geo_delay:
            await self.delay(self.geo_delay)
        response.map_point.x = request.ll_point.latitude * 10.
        return response

    async def delay(self, interval):
        future = Future()
        timer = self.create_timer(interval, lambda: future.set_result(True) if not future.done() else None,
                                  callback_group=self.group)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def inputs(self):
        stamp = self.get_clock().now().to_msg()
        authority = ControlAuthorityState()
        authority.header.stamp = stamp
        authority.valid = authority.connected = authority.armed = True
        authority.authority = authority.AUTONOMOUS if self.authority_allowed else authority.EXTERNAL
        authority.motion_allowed = self.authority_allowed
        self.authority_pub.publish(authority)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'boat/base_link'
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, 'map'
        transform.child_frame_id = 'boat/base_link'
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = self.pose_x
        self.tf.sendTransform(transform)
        evidence = ExecutionEvidence()
        evidence.header.stamp, evidence.role, evidence.boot_id = stamp, 'controller', 'fake-controller'
        evidence.active = self.active
        if self.current:
            evidence.execution_id, evidence.attempt_id, evidence.epoch = (
                self.current.usv_execution_id, self.current.usv_attempt_id, self.current.usv_epoch)
        self.evidence_pub.publish(evidence)
        rule = MaritimeRuleDecision()
        rule.header.stamp, rule.valid = stamp, True
        rule.intervention_required, rule.clear_confirmed = self.waiting, not self.waiting
        rule.stand_on = self.stand_on
        if self.maritime_policy is not None:
            self.maritime_policy.update([self.encounter_report])
            waiting, valid, clear = self.maritime_policy.decision()
            rule.intervention_required, rule.valid, rule.clear_confirmed = waiting, valid, clear
            rule.stand_on = bool(self.maritime_policy.stand_on_target and not waiting and valid)
        self.rule_pub.publish(rule)
        regular = SafetyConstraintState()
        regular.header.stamp = stamp
        regular.task_allowed = regular.hold_allowed = self.regular_allowed
        regular.max_speed, regular.valid_for.nanosec = self.regular_cap, 500000000
        self.regular_pub.publish(regular)
        if self.active and self.current:
            command = ManagedMotion()
            command.header.stamp = stamp
            command.boot_id, command.execution_id = self.current.usv_boot_id, self.current.usv_execution_id
            command.attempt_id, command.epoch = self.current.usv_attempt_id, self.current.usv_epoch
            self.sequence += 1
            command.sequence, command.valid_for.nanosec = self.sequence, 250000000
            command.velocity.linear.x = .8
            self.last_motion = command
            self.motion_pub.publish(command)

    async def plan(self, handle):
        self.plans += 1
        self.planner_ids.append(handle.request.planner_id)
        result = ComputePathToPose.Result()
        if self.fail_plans:
            self.fail_plans -= 1
            result.usv_retryable = True
            handle.abort()
            return result
        self.planning_active = True
        if self.plan_delay:
            await self.delay(self.plan_delay)
        self.planning_active = False
        if handle.is_cancel_requested:
            handle.canceled()
            return result
        result.path = self.path(handle.request.goal)
        handle.succeed()
        return result

    async def plan_many(self, handle):
        result = ComputePathThroughPoses.Result()
        result.path = self.path(handle.request.goals[-1])
        handle.succeed()
        return result

    def path(self, goal):
        path = NavPath()
        path.header = goal.header
        start = PoseStamped()
        start.header = goal.header
        start.pose.orientation.w = 1.0
        start.pose.position.x = self.pose_x
        path.poses = [start, goal]
        return path

    async def follow(self, handle):
        self.generation += 1
        generation = self.generation
        self.current, self.active = handle.request, True
        self.sequence = 0
        while rclpy.ok():
            if generation != self.generation:
                handle.abort()
                return FollowPath.Result()
            if handle.is_cancel_requested:
                self.active = False
                handle.canceled()
                return FollowPath.Result()
            if self.finish:
                self.pose_x = handle.request.path.poses[-1].pose.position.x
                self.inputs()
                await self.delay(.15)
                self.active = False
                handle.succeed()
                return FollowPath.Result()
            await self.delay(.05)
        return FollowPath.Result()

    async def wait(self, handle):
        self.wait_calls += 1
        if self.fail_wait:
            handle.abort()
            return Wait.Result()
        start = monotonic()
        while monotonic() - start < handle.request.time.sec:
            if handle.is_cancel_requested:
                handle.canceled()
                return Wait.Result()
            await self.delay(.05)
        handle.succeed()
        return Wait.Result()


class DelayedRegistrationMission(Mission):
    """Register durably, but lose the acknowledgement past the BT deadline."""
    delay_registration = False
    delay_registration_action = ''

    async def navigation_control(self, request, response):
        result = super().navigation_control(request, response)
        if (request.operation == 'register' and self.delay_registration
                and request.key.startswith(self.delay_registration_action)):
            self.delay_registration = False
            await self.registration_pause(.35)
        return result


def test_real_trees_pause_risk_authority_retry_and_checkpoint(tmp_path):
    bt_share = get_package_share_directory('usv_autonomy_bt')
    rclpy.init(args=['--ros-args', '-r', '__ns:=/btv2',
        '-p', 'database:=' + str(tmp_path / 'tasks.db'), '-p', 'base_frame:=boat/base_link',
        '-p', 'require_radar:=false', '-p', 'emergency_requires_reset:=false',
        '-p', 'left_crossing_stand_on:=true',
        '-p', 'stand_on_navigation_tree:=' + bt_share + '/trees/stand_on_to_pose.xml',
        '-p', 'stand_on_navigation_through_tree:=' + bt_share + '/trees/stand_on_through_poses.xml',
        '-p', 'navigation_tree:=' + bt_share + '/trees/navigate_to_pose.xml'])
    peers, mission, gate = Peers(), DelayedRegistrationMission(), MotionGate()
    mission.registration_pause = peers.delay
    executor = SingleThreadedExecutor()
    for node in (peers, mission, gate):
        executor.add_node(node)
    logs = []
    processes = []
    def start(package, executable, args):
        log = (tmp_path / (executable + '.log')).open('w+')
        logs.append(log)
        prefix = get_package_prefix(package)
        process = subprocess.Popen([prefix + '/lib/' + package + '/' + executable,
            '--ros-args', '-r', '__ns:=/btv2'] + args, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process
    def until(predicate, timeout=10):
        end = monotonic() + timeout
        while monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        for log in logs:
            log.flush()
            log.seek(0)
            print(log.read())
        pytest.fail('timeout; core=' + repr((mission.core.reason, mission.core.blockers,
            mission.core.fault, mission.core.records, mission.core.cancel_scope,
            mission.core.execution, mission.core.producer, mission.core.result)))
    def call(command, task_id='integration', mode='NAVIGATION', policy='REQUIRED', duration=0., loop=False,
             manual_anchor=None):
        request = ManageMission.Request()
        request.command, request.task_id, request.mode = command, task_id, mode
        request.waypoint_policy = policy
        request.loop_execution = loop
        request.request_id = command + str(monotonic())
        request.expected_boot_id = mission.core.boot
        if command in ('start', 'switch'):
            pose = PoseStamped()
            pose.header.frame_id, pose.pose.orientation.w, pose.pose.position.x = 'map', 1., 10.
            request.goals = [pose]
            if mode == 'NAVIGATION':
                second = PoseStamped()
                second.header.frame_id, second.pose.orientation.w, second.pose.position.x = 'map', 1., 20.
                request.goals.append(second)
            if mode == 'HOLD':
                request.hold.inner_radius, request.hold.outer_radius = 2., 3.
                request.hold.return_timeout_s = 15.
                request.hold.duration_s = duration
                if manual_anchor is not None:
                    request.hold.use_current_pose = False
                    request.hold.anchor.header.frame_id = 'map'
                    request.hold.anchor.pose.position.x = manual_anchor
                    request.hold.anchor.pose.orientation.w = 1.
        future = peers.manage_client.call_async(request)
        until(future.done)
        return future.result()
    def output(intent):
        return bool(peers.commands and peers.commands[-1].intent == intent)
    try:
        plugins = '[nav2_compute_path_to_pose_action_bt_node,nav2_compute_path_through_poses_action_bt_node,' \
                  'nav2_follow_path_action_bt_node,nav2_wait_action_bt_node,nav2_rate_controller_bt_node,' \
                  'nav2_remove_passed_goals_action_bt_node,' \
                  'nav2_recovery_node_bt_node,nav2_pipeline_sequence_bt_node,usv_navigation_contract_bt_node]'
        start('nav2_bt_navigator', 'bt_navigator', ['-p', 'robot_base_frame:=boat/base_link',
            '-p', 'require_managed_execution:=true', '-p', 'plugin_lib_names:=' + plugins,
            '-p', 'default_server_timeout:=200',
            '-p', 'default_nav_to_pose_bt_xml:=' + bt_share + '/trees/navigate_to_pose.xml',
            '-p', 'default_nav_through_poses_bt_xml:=' + bt_share + '/trees/navigate_through_poses.xml'])
        start('usv_autonomy_bt', 'task_tree', [])
        lifecycle = peers.create_client(ChangeState, 'bt_navigator/change_state')
        until(lifecycle.service_is_ready)
        until(lambda: mission.core.blockers.get('navigation_backend'))
        assert call('start').accepted
        assert not mission.core.execution and not mission.core.permitted
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = lifecycle.call_async(request)
            until(future.done)
            assert future.result().success
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        assert peers.planner_ids[-1] == 'VORRTStar'
        # Real maritime policy -> ROS decision -> mission -> real Nav2/task BT:
        # turning right must not turn a confirmed head-on into GridBased/WAIT.
        peers.maritime_policy = MaritimeRules(monotonic, stand_on_enabled=True,
            encounter_confirm_time=.15, stand_on_confirm_time=.1, observe=.3)
        peers.encounter_report = ('head-on-target', False, True, 5., 20., 2, 80., 1)
        until(lambda: peers.maritime_policy.responsibilities() == {'head-on-target': 1})
        head_on_execution = mission.core.execution
        peers.encounter_report = ('head-on-target', True, True, 5., 20., 2, 80., 3)
        end = monotonic() + 1.
        until(lambda: monotonic() >= end)
        assert mission.core.execution == head_on_execution and output(ActuatorCommand.VELOCITY_CONTROL)
        assert set(peers.planner_ids) == {'VORRTStar'}
        assert 'maritime' not in mission.core.blockers and not mission.active_stand_on
        peers.maritime_policy = None
        old_execution = mission.core.execution
        peers.stand_on = True
        until(lambda: peers.planner_ids[-1] == 'GridBased' and mission.core.execution != old_execution
              and output(ActuatorCommand.VELOCITY_CONTROL))
        assert mission.core.task['index'] == 0 and mission.core.task['task_retries'] == 0
        old_execution = mission.core.execution
        peers.stand_on = False
        until(lambda: peers.planner_ids[-1] == 'VORRTStar' and mission.core.execution != old_execution
              and output(ActuatorCommand.VELOCITY_CONTROL))
        first_execution = mission.core.execution
        assert call('switch', 'dock', 'DOCK').code == 501
        assert mission.core.execution == first_execution
        assert call('pause').accepted
        until(lambda: output(ActuatorCommand.NEUTRAL) and not mission.core.execution)
        assert not mission.core.fault
        assert call('resume').accepted
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        assert mission.core.execution != first_execution
        # Reproduce CCS: pause while periodic planning is still in its backend.
        # Its terminal result comes AFTER BtActionNode::halt() stopped spinning.
        peers.plan_delay = .9
        until(lambda: peers.planning_active and any(
            k.startswith('compute_path_to_pose:') for k in mission.child_results.pending))
        old_execution, old_epoch = mission.core.execution, mission.core.epoch
        old_budget = mission.core.task['task_retries']
        assert call('pause').accepted
        pending_resume = call('resume')
        assert pending_resume.accepted and pending_resume.code in (200, 202)
        until(lambda: mission.core.execution != old_execution and output(ActuatorCommand.VELOCITY_CONTROL))
        assert not mission.core.fault and mission.core.epoch > old_epoch
        assert mission.core.task['index'] == 0 and mission.core.task['task_retries'] == old_budget
        peers.plan_delay = 0.
        peers.waiting = True
        until(lambda: output(ActuatorCommand.NEUTRAL) and not mission.core.execution)
        assert call('pause').accepted
        peers.waiting = False
        until(lambda: 'maritime' not in mission.core.blockers)
        end = monotonic() + .4
        until(lambda: monotonic() > end)
        assert output(ActuatorCommand.NEUTRAL) and mission.core.task['intent'] == 'PAUSE'
        peers.fail_plans = 1
        assert call('resume').accepted
        until(lambda: peers.wait_calls == 1)
        assert output(ActuatorCommand.NEUTRAL)
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL), timeout=10)
        assert mission.core.task['task_retries'] == 1
        peers.regular_cap = .2
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL) and peers.commands[-1].velocity.linear.x <= .2)
        peers.regular_allowed = False
        until(lambda: output(ActuatorCommand.NEUTRAL) and not mission.core.execution)
        end = monotonic() + .2
        until(lambda: monotonic() > end)
        assert mission.core.blockers['regular_safety']  # independent NORMAL cannot clear this source
        peers.regular_allowed, peers.regular_cap = True, 1.
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        old_motion = peers.last_motion
        peers.authority_allowed = False
        until(lambda: output(ActuatorCommand.DISABLED) and not mission.core.execution)
        peers.authority_allowed = True
        until(lambda: mission.core.authority)
        assert not mission.core.runnable()  # explicit authority-return confirmation
        assert call('resume').accepted
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        accepted = gate.gate.last_sequence
        old_motion.header.stamp = peers.get_clock().now().to_msg()
        old_motion.sequence = 999999
        peers.motion_pub.publish(old_motion)
        end = monotonic() + .1
        until(lambda: monotonic() > end)
        assert gate.gate.last_sequence != 999999
        # HOLD must repeatedly return to its inner region, without requiring the
        # point/yaw Action to succeed, and continue monitoring between returns.
        assert call('switch', 'hold', 'HOLD').accepted
        until(lambda: mission.core.task['id'] == 'hold' and mission.core.task['hold_anchor'] is not None)
        anchor = mission.core.task['hold_anchor']['position'][0]
        executions = set()
        for count in range(1, 4):
            peers.pose_x = anchor + 5.
            until(lambda: mission.core.execution and output(ActuatorCommand.VELOCITY_CONTROL))
            executions.add(mission.core.execution)
            # Inside inner=2 m, but outside the point goal tolerance=1.5 m.
            # The fake FollowPath never succeeds: HOLD must cancel it itself.
            peers.pose_x = anchor + 1.8
            until(lambda: mission.core.task['hold_returns'] == count and not mission.core.execution
                  and output(ActuatorCommand.NEUTRAL))
            assert mission.navigation_started is None
            peers.pose_x = anchor + 2.5  # Hysteresis band must remain monitoring.
            end = monotonic() + (15.2 if count == 1 else .4)
            until(lambda: monotonic() > end, timeout=20)
            assert not mission.core.fault and not mission.core.blockers
            assert not mission.core.execution and mission.core.task['intent'] == 'RUN'
            assert mission.store.tasks()['hold']['hold_returns'] == count
            assert mission.core.task['hold_anchor']['position'][0] == anchor
            assert mission.core.task['task_retries'] == 0
        assert len(executions) == 3
        assert mission.core.task['index'] == 0
        # Field regression: a replacement FollowPath registration reply arrives
        # after the C++ deadline on a subsequent return. The old accepted controller
        # must exit before bounded Wait recovery authorizes a fresh attempt.
        peers.pose_x = anchor + 5.
        until(lambda: mission.core.execution and output(ActuatorCommand.VELOCITY_CONTROL))
        old_execution, old_epoch = mission.core.execution, mission.core.epoch
        wait_calls, generation = peers.wait_calls, peers.generation
        mission.delay_registration_action = 'follow_path:'
        mission.delay_registration = True
        peers.pose_x = anchor + 4.5  # Force a changed path on periodic replanning.
        until(lambda: peers.wait_calls > wait_calls)
        assert not peers.active and not mission.core.records
        assert output(ActuatorCommand.NEUTRAL) and not mission.core.permitted
        assert peers.generation == generation  # timed-out replacement was never sent
        assert mission.core.task['task_retries'] == 1
        until(lambda: mission.core.epoch > old_epoch and output(ActuatorCommand.VELOCITY_CONTROL))
        assert mission.core.execution == old_execution
        assert not mission.core.fault and 'execution_failed' not in mission.core.blockers
        peers.pose_x = anchor + 1.8
        until(lambda: mission.core.task['hold_returns'] == 4 and not mission.core.execution)
        assert mission.core.task['hold_anchor']['position'][0] == anchor
        assert mission.store.tasks()['hold']['task_retries'] == 1
        assert call('resume_navigation').accepted
        until(lambda: mission.core.task['id'] == 'integration' and not mission.core.execution)
        assert mission.core.task['intent'] == 'PAUSE' and output(ActuatorCommand.NEUTRAL)
        end = monotonic() + .4
        until(lambda: monotonic() > end)
        assert not mission.core.execution and not mission.core.permitted
        assert call('resume').accepted
        until(lambda: mission.core.task['id'] == 'integration' and output(ActuatorCommand.VELOCITY_CONTROL))
        assert mission.core.task['index'] == 0 and mission.core.task['task_retries'] == 1
        # Mode re-entry captures NOW, not the previous HOLD anchor or the later
        # Continue position. Explicit manual coordinates work on the same task.
        peers.pose_x = 2.5
        until(lambda: mission.pose and mission.pose[0] == 2.5)
        assert call('switch', 'ignored-hold-id', 'HOLD').accepted
        until(lambda: mission.core.task['id'] == 'hold' and not mission.core.execution)
        assert mission.core.task['hold_anchor']['position'][0] == 2.5
        assert mission.core.task['intent'] == 'PAUSE' and output(ActuatorCommand.NEUTRAL)
        peers.pose_x = 4.
        until(lambda: mission.pose[0] == 4.)
        assert mission.core.task['hold_anchor']['position'][0] == 2.5
        assert call('switch', 'ignored-manual-id', 'HOLD', manual_anchor=-12.).accepted
        until(lambda: mission.core.pending_task is None)
        assert mission.core.task['hold_anchor']['position'][0] == -12.
        assert mission.core.task['hold_returns'] == 4 and mission.core.task['task_retries'] == 1
        assert mission.core.task['intent'] == 'PAUSE' and not mission.core.execution
        assert call('resume_navigation').accepted
        until(lambda: mission.core.task['id'] == 'integration' and not mission.core.execution)
        assert call('resume').accepted
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        # A frozen task-tree process must not keep autonomy moving.
        runner = processes[1]
        runner.send_signal(signal.SIGSTOP)
        stopped = monotonic()
        until(lambda: output(ActuatorCommand.NEUTRAL), timeout=2.)
        assert monotonic() - stopped < 1.5
        until(lambda: not mission.core.execution)
        runner.send_signal(signal.SIGCONT)
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        # Suspend the mission event loop too: the gate must expire its lease itself.
        executor.remove_node(mission)
        until(lambda: output(ActuatorCommand.NEUTRAL), timeout=1.5)
        executor.add_node(mission)
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        peers.finish = True
        until(lambda: mission.core.task['state'] == 'COMPLETED', timeout=10)
        assert mission.store.tasks()['integration']['completed'] == [0, 1]
        assert mission.core.task['task_retries'] == 1
        assert not mission.core.fault
        peers.finish, peers.pose_x = False, 0.
        assert call('start', 'batch', policy='VIA').accepted
        until(lambda: mission.core.task['id'] == 'batch' and output(ActuatorCommand.VELOCITY_CONTROL))
        peers.pose_x = 10.
        until(lambda: mission.core.task['index'] == 1)
        assert mission.store.tasks()['batch']['completed'] == [0]
        peers.finish = True
        until(lambda: mission.core.task['state'] == 'COMPLETED')
        assert mission.store.tasks()['batch']['completed'] == [0, 1]
        # Geographic compatibility uses the localization service, not a fake origin.
        legacy = ActionClient(peers, ExecuteAutoTask, 'execute_auto_task')
        until(legacy.server_is_ready)
        def legacy_request(task_id):
            req = ExecuteAutoTask.Goal()
            req.command, req.task_id, req.mode = 'start', task_id, 'NAVIGATION'
            waypoint = Waypoint()
            waypoint.latitude, waypoint.longitude, waypoint.speed_target = 1., 2., .5
            req.waypoints = [waypoint]
            return req
        peers.geo_delay, peers.finish = .3, False
        future = legacy.send_goal_async(legacy_request('canceled-geo'))
        until(future.done)
        geo_handle = future.result()
        canceled = geo_handle.cancel_goal_async()
        until(canceled.done)
        result = geo_handle.get_result_async()
        until(result.done)
        assert result.result().result.code == 499 and 'canceled-geo' not in mission.core.tasks
        peers.geo_delay = 0.
        future = legacy.send_goal_async(legacy_request('geographic'))
        until(future.done)
        result = future.result().get_result_async()
        until(result.done)
        assert result.result().result.code == 202
        until(lambda: mission.core.task['id'] == 'geographic' and output(ActuatorCommand.VELOCITY_CONTROL))
        assert mission.core.task['spec']['goals'][0]['position'][0] == 10.
        assert peers.commands[-1].velocity.linear.x <= .5
        assert call('stop', 'geographic').accepted
        until(lambda: not mission.core.execution and output(ActuatorCommand.NEUTRAL))
        assert call('start', 'timed-hold', 'HOLD', duration=.7).accepted
        until(lambda: mission.core.task['id'] == 'timed-hold' and mission.core.task['state'] == 'COMPLETED')
        assert mission.store.tasks()['timed-hold']['hold_elapsed'] >= .7
        peers.finish = True
        assert call('start', 'loop', loop=True).accepted
        until(lambda: mission.core.task['id'] == 'loop' and mission.core.task['lap'] >= 1)
        assert call('pause', 'loop').accepted
        until(lambda: not mission.core.execution and output(ActuatorCommand.NEUTRAL))
        assert mission.store.tasks()['loop']['lap'] >= 1
        # Recovery backend failure must exit to task-level manual handling,
        # not remain frozen forever in a falsely RUNNING navigation primitive.
        peers.fail_plans, peers.fail_wait = 1, True
        assert call('start', 'failed-wait').accepted
        until(lambda: 'execution_failed' in mission.core.blockers and not mission.core.execution)
        assert output(ActuatorCommand.NEUTRAL) and not mission.core.fault
        assert mission.core.task['task_retries'] == 1
        # The actual C++ BtActionNode times out AFTER the durable registration,
        # but before sending an Action Goal. Its not_sent proof must retire the
        # pending record and recover through the real bounded navigation tree.
        peers.fail_plans, peers.fail_wait, peers.finish = 0, False, False
        plans = peers.plans
        mission.delay_registration = True
        mission.delay_registration_action = 'compute_path_to_pose:'
        assert call('start', 'registration-timeout').accepted
        until(lambda: mission.core.task['id'] == 'registration-timeout'
              and mission.core.task['task_retries'] == 1 and not mission.core.permitted)
        assert peers.plans == plans
        until(lambda: output(ActuatorCommand.VELOCITY_CONTROL))
        assert not mission.core.fault and 'execution_failed' not in mission.core.blockers
        assert mission.core.task['task_retries'] == 1 and peers.plans > plans
        assert mission.store.db.execute("SELECT COUNT(*) FROM events WHERE kind='child_not_sent'").fetchone()[0] >= 1
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for node in (mission, gate, peers):
            executor.remove_node(node)
            node.destroy_node()
        mission.store.close()
        executor.shutdown()
        rclpy.shutdown()
        for log in logs:
            log.close()
