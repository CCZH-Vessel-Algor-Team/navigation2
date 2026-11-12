"""Missing targets latch takeover; the public safety reset clears durable events."""
import json
import signal
import subprocess
from contextlib import contextmanager
from pathlib import Path
from time import monotonic

import pytest
import rclpy
from nav2_maritime_situation_msgs.msg import SituationReportArray, SituationReport
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from usv_interfaces.msg import ExecutionEvidence, MaritimeRuleDecision, GateState, AutonomyState
from usv_interfaces.srv import ResetSafetyLatch
from usv_interfaces.srv import ResetMaritimeEvent

from usv_autonomy.mission_node import Mission
from usv_autonomy.rules_node import RulesNode
from usv_autonomy.gate_node import MotionGate
from usv_autonomy.simulation_controls import SimulationControls


def test_missing_target_takeover_and_public_reset_with_real_task_tree(tmp_path):
    checkpoint = tmp_path / 'maritime.json'
    checkpoint.write_text(json.dumps({'waiting_targets': ['ship']}))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/missing_target_contract',
        '-p', 'database:=' + str(tmp_path / 'tasks.db'),
        '-p', 'rules_checkpoint:=' + str(checkpoint),
        '-p', 'missing_target_timeout:=2.0', '-p', 'simulation_mode:=true',
        '-p', 'simulate_authority:=true',
        '-p', 'left_crossing_stand_on:=true', '-p', 'require_radar:=false',
        '-p', 'emergency_clear_time:=0.0'])
    rules, mission, gate, controls, peer = RulesNode(), Mission(), MotionGate(), SimulationControls(), Node('peer')
    controls.flags['authority'] = True
    executor = SingleThreadedExecutor()
    nodes = (rules, mission, gate, controls, peer)
    for node in nodes:
        executor.add_node(node)
    publisher = peer.create_publisher(SituationReportArray, 'maritime/situations', 10)
    now = [0.]
    rules.rules.clock = lambda: now[0]
    odom_pub = peer.create_publisher(Odometry, 'odom', 1)
    estop_pub = peer.create_publisher(Bool, 'safety/estop', 1)
    producer_pub = peer.create_publisher(ExecutionEvidence, 'autonomy/producer', 1)
    def inputs():
        odom = Odometry()
        odom.header.stamp = peer.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        producer = ExecutionEvidence()
        producer.header = odom.header
        producer.role, producer.boot_id = 'controller', 'test-controller'
        producer_pub.publish(producer)
    peer.create_timer(.05, inputs)
    tree_log = (tmp_path / 'tree.log').open('w')
    tree = subprocess.Popen([get_package_prefix('usv_autonomy_bt') + '/lib/usv_autonomy_bt/task_tree', '--ros-args',
                             '-r', '__ns:=/missing_target_contract'], stdout=tree_log, stderr=tree_log)

    def until(predicate):
        deadline = monotonic() + 4.
        while monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('rule release timeout: ' + repr(mission.core.blockers))

    def report():
        revision = rules.rules.revision
        msg = SituationReportArray()
        msg.header.stamp = peer.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.observations_valid = True
        publisher.publish(msg)
        until(lambda: rules.rules.revision > revision)

    try:
        assert rules.rules.missing_target_timeout == 2.
        until(lambda: publisher.get_subscription_count() > 0)
        until(lambda: mission.core.authority and mission.core.quiet())
        spec = dict(mode='NAVIGATION', goals=[dict(frame='map', position=[10., 0., 0.],
                    orientation=[0., 0., 0., 1.])], hold={})
        assert mission.core.manage('start', 'mission', spec, 'start')[0]
        until(lambda: mission.core.blockers.get('maritime')
              and mission.core.blockers.get('rules_unknown'))
        report()
        now[0] = 1.
        report()
        assert rules.rules.waiting_ids() == ['ship']
        assert json.loads(checkpoint.read_text())['waiting_targets'] == ['ship']
        now[0] = 2.
        report()
        until(lambda: mission.core.blockers.get('maritime_handover'))
        event = rules.rules.event_id
        assert event and json.loads(checkpoint.read_text())['event_id'] == event
        # The real task XML must request the backend and observe SIM_MANUAL,
        # even though rules_unknown prevents navigation.
        until(lambda: not controls.flags['authority'] and mission.manual_authority_time > 0)
        assert not mission.core.runnable()
        assert mission.core.task['return_confirmation']
        reset = peer.create_client(ResetSafetyLatch, 'autonomy/reset_safety')
        until(reset.service_is_ready)
        future = reset.call_async(ResetSafetyLatch.Request(confirmed=False))
        until(future.done)
        assert not future.result().accepted
        assert rules.rules.event_id == event
        # EXTERNAL alone is not sufficient: the actual mode must be manual.
        mission.manual_modes = ['MANUAL']
        until(lambda: mission.manual_authority_time == float('-inf'))
        future = reset.call_async(ResetSafetyLatch.Request(confirmed=True))
        until(future.done)
        assert not future.result().accepted and rules.rules.event_id == event
        mission.manual_modes = ['SIM_MANUAL']
        until(lambda: mission.manual_authority_time > 0)
        # An actual emergency cannot be cleared through a maritime reset.
        estop_pub.publish(Bool(data=True))
        until(lambda: gate.latched and gate.estop)
        future = reset.call_async(ResetSafetyLatch.Request(confirmed=True))
        until(future.done)
        assert not future.result().accepted and rules.rules.event_id == event
        estop_pub.publish(Bool(data=False))
        until(lambda: not gate.estop and gate.clear_since is not None)
        future = reset.call_async(ResetSafetyLatch.Request(confirmed=True))
        until(future.done)
        assert future.result().accepted, future.result().reason
        report()
        until(lambda: 'maritime' not in mission.core.blockers
              and 'rules_unknown' not in mission.core.blockers
              and 'maritime_handover' not in mission.core.blockers)
        assert not gate.latched
        assert json.loads(checkpoint.read_text())['waiting_targets'] == []
        assert json.loads(checkpoint.read_text())['event_id'] == ''
        assert mission.core.task['intent'] == 'PAUSE'
        assert mission.core.task['return_confirmation']
        assert not mission.core.runnable()
        executor.remove_node(rules)
        rules.destroy_node()
        # A process restart also closes the checkpoint's single-writer lock.
        rules.checkpoint_lock.close()
        restarted = RulesNode()
        try:
            assert restarted.rules.waiting_ids() == []
        finally:
            restarted.destroy_node()
            restarted.checkpoint_lock.close()
    finally:
        tree.send_signal(signal.SIGINT)
        try:
            tree.wait(timeout=5)
        except subprocess.TimeoutExpired:
            tree.kill()
            tree.wait()
        tree_log.close()
        for node in (mission, gate, controls, peer):
            executor.remove_node(node)
            node.destroy_node()
        if rules in executor.get_nodes():
            executor.remove_node(rules)
            rules.destroy_node()
        rules.checkpoint_lock.close()
        mission.store.close()
        executor.shutdown()
        rclpy.shutdown()


def test_handover_survives_restart_and_failed_or_stale_reset(tmp_path, monkeypatch):
    checkpoint = tmp_path / 'maritime.json'
    rclpy.init(args=['--ros-args', '-r', '__ns:=/maritime_checkpoint_contract',
                    '-p', 'rules_checkpoint:=' + str(checkpoint)])
    node = RulesNode()
    try:
        node.rules.restore_waiting(['ship'])
        node.rules.restore_responsibilities({'ship': 1})
        node.rules.latch_handover('target_missing')
        event = node.rules.event_id
        node.tick()
        node.destroy_node()
        node.checkpoint_lock.close()
        node = RulesNode()
        assert node.rules.event_id == event and node.rules.decision()[0]
        assert node.rules.event_target == 'ship'
        assert node.rules.responsibilities() == {'ship': 1}
        response = node.reset(ResetMaritimeEvent.Request(confirmed=True, event_id='old'),
                              ResetMaritimeEvent.Response())
        assert not response.accepted and node.rules.event_id == event
        assert node.rules.responsibilities() == {'ship': 1}
        def fail(_):
            raise OSError('injected disk failure')
        monkeypatch.setattr(node, 'persist', fail)
        response = node.reset(ResetMaritimeEvent.Request(confirmed=True, event_id=event),
                              ResetMaritimeEvent.Response())
        assert not response.accepted and node.rules.event_id == event
        assert json.loads(checkpoint.read_text())['event_id'] == event
        assert json.loads(checkpoint.read_text())['responsibilities'] == {'ship': 1}
    finally:
        node.destroy_node()
        node.checkpoint_lock.close()
        rclpy.shutdown()


def test_head_on_checkpoint_migrates_and_prevents_left_policy_after_restart(tmp_path):
    checkpoint = tmp_path / 'maritime.json'
    checkpoint.write_text(json.dumps(dict(version=2, waiting_targets=[], event_id='',
                                          event_target='', handover_reason='')))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/head_on_responsibility_contract',
                    '-p', 'rules_checkpoint:=' + str(checkpoint),
                    '-p', 'left_crossing_stand_on:=true'])
    node, peer = RulesNode(), Node('peer')
    executor = SingleThreadedExecutor()
    executor.add_node(peer)
    messages = []
    peer.create_subscription(MaritimeRuleDecision, 'autonomy/maritime', messages.append, 10)
    clock = [0.]
    target = '8d5e7d86fd415aa5b05cab1b7aa9d5e9'

    def report(kind):
        state = AutonomyState()
        state.header.stamp = node.get_clock().now().to_msg()
        state.mode = 'NAVIGATION'
        node.task_state(state)
        msg = SituationReportArray()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.observations_valid = True
        ship = SituationReport()
        ship.target_id.uuid = list(bytes.fromhex(target))
        ship.encounter_type, ship.cpa_valid = kind, True
        ship.dcpa, ship.tcpa, ship.current_range, ship.risk_level = 5., 20., 80., 2
        msg.targets.append(ship)
        node.report(msg)

    try:
        assert node.rules.responsibilities() == {}  # Legacy checkpoint stays readable.
        node.rules.clock = lambda: clock[0]
        for instant in (0., .5, 1.):
            clock[0] = instant
            report(1)
        node.tick()
        data = json.loads(checkpoint.read_text())
        assert data['version'] == 3 and data['responsibilities'] == {target: 1}
        node.destroy_node()
        node.checkpoint_lock.close()
        node = RulesNode()
        node.rules.clock = lambda: clock[0]
        assert node.rules.responsibilities() == {target: 1}
        for i in range(15):
            clock[0] += .5
            report(3)
        assert not node.rules.stand_on_target and not node.rules.waiting_ids()
        executor.add_node(node)
        end = monotonic() + 3.
        while monotonic() < end and not any(m.reason == 'head_on_responsibility' for m in messages):
            executor.spin_once(timeout_sec=.01)
        assert any(m.reason == 'head_on_responsibility' and m.valid and not m.stand_on
                   and not m.intervention_required for m in messages)
        # An explicitly confirmed handover reset cleans roles in the same
        # durable transaction as the risk event, not through a second public API.
        node.rules.latch_handover('target_missing', target)
        node.tick()
        result = node.reset(ResetMaritimeEvent.Request(confirmed=True, event_id=node.rules.event_id),
                            ResetMaritimeEvent.Response())
        assert result.accepted and not node.rules.responsibilities()
        assert json.loads(checkpoint.read_text())['responsibilities'] == {}
    finally:
        for item in (node, peer):
            if item in executor.get_nodes():
                executor.remove_node(item)
            item.destroy_node()
        node.checkpoint_lock.close()
        executor.shutdown()
        rclpy.shutdown()


def test_unresponsive_reset_backend_times_out_without_stalling_independent_gate():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/maritime_reset_timeout_contract',
                    '-p', 'require_radar:=false', '-p', 'emergency_clear_time:=0.0'])
    gate, peer = MotionGate(), Node('peer')
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(peer)
    held = []
    completed = []
    async def hold_reset(request, response):
        future = Future()
        held.append(future)
        await future
        completed.append(True)
        response.accepted = True
        return response
    peer.create_service(ResetSafetyLatch, 'autonomy/internal/reset_maritime', hold_reset,
                        callback_group=ReentrantCallbackGroup())
    odom_pub = peer.create_publisher(Odometry, 'odom', 1)
    rule_pub = peer.create_publisher(MaritimeRuleDecision, 'autonomy/maritime', 1)
    states = []
    peer.create_subscription(GateState, 'autonomy/gate_state', states.append, 10)
    def inputs():
        odom = Odometry()
        odom.header.stamp = peer.get_clock().now().to_msg()
        odom_pub.publish(odom)
        rule = MaritimeRuleDecision()
        rule.header = odom.header
        rule.event_id, rule.handover_required = 'event', True
        rule_pub.publish(rule)
    peer.create_timer(.05, inputs)
    def until(predicate, timeout=5.):
        end = monotonic() + timeout
        while monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('reset timeout contract failed')
    client = peer.create_client(ResetSafetyLatch, 'autonomy/reset_safety')
    try:
        until(lambda: client.service_is_ready() and gate.maritime_event == 'event'
              and gate.clear_since is not None)
        gate.latched = True
        before = len(states)
        future = client.call_async(ResetSafetyLatch.Request(confirmed=True))
        until(future.done)
        assert not future.result().accepted and 'timed out' in future.result().reason
        assert gate.latched and len(states) - before >= 20
    finally:
        for future in held:
            if not future.done():
                future.set_result(True)
        if held:
            until(lambda: len(completed) == len(held))
        for node in (gate, peer):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.)
        rclpy.shutdown()


def test_new_ccs_session_is_clean_but_node_restart_keeps_task_and_handover(tmp_path):
    from usv_sim_full.autonomy_session import new_autonomy_session, autonomy_session_paths

    @contextmanager
    def nodes(paths):
        rclpy.init(args=['--ros-args', '-r', '__ns:=/session_restart_contract',
                        '-p', 'database:=' + paths['autonomy_database'],
                        '-p', 'rules_checkpoint:=' + paths['autonomy_rules_checkpoint']])
        mission, rules = None, None
        try:
            mission, rules = Mission(), RulesNode()
            yield mission, rules
        finally:
            if rules is not None:
                rules.destroy_node()
                rules.checkpoint_lock.close()
            if mission is not None:
                mission.destroy_node()
                mission.store.close()
                mission.database_lock.close()
            rclpy.shutdown()

    first = autonomy_session_paths(new_autonomy_session(tmp_path), 'boat')
    with nodes(first) as (mission, rules):
        assert mission.core.tasks == {} and rules.rules.waiting_ids() == []
        spec = dict(mode='NAVIGATION', goals=[dict(frame='map', position=[10., 0., 0.],
                    orientation=[0., 0., 0., 1.])], hold={})
        assert mission.core.manage('start', 'retained-task', spec, 'start')[0]
        rules.rules.restore_waiting(['retained-target'])
        rules.rules.restore_responsibilities({'retained-target': 1})
        rules.rules.latch_handover('target_missing', 'retained-target')
        event = rules.rules.event_id
        rules.tick()
    # Restarting nodes with the launch's fixed arguments keeps all safety data.
    with nodes(first) as (mission, rules):
        assert 'retained-task' in mission.core.tasks
        assert mission.core.tasks['retained-task']['intent'] == 'PAUSE'
        assert rules.rules.event_id == event
        assert rules.rules.waiting_ids() == ['retained-target']
        assert rules.rules.responsibilities() == {'retained-target': 1}
    # Restarting the whole system creates different paths, not a reset request.
    second = autonomy_session_paths(new_autonomy_session(tmp_path), 'boat')
    with nodes(second) as (mission, rules):
        assert mission.core.tasks == {} and mission.core.task is None
        assert not rules.rules.event_id and rules.rules.waiting_ids() == []
        assert rules.rules.responsibilities() == {}
    assert json.loads(Path(first['autonomy_rules_checkpoint']).read_text())['event_id'] == event
