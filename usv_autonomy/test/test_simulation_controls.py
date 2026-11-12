"""Real CCS test services, isolated inputs, and protection/lifecycle semantics."""
import importlib.util
from pathlib import Path
from time import monotonic
import pytest
import rclpy
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from usv_interfaces.msg import MaritimeRuleDecision
from usv_interfaces.srv import ResetSafetyLatch
from usv_autonomy.simulation_controls import SimulationControls
from usv_autonomy.mission_node import Mission
from usv_autonomy.gate_node import MotionGate
from usv_autonomy.health_node import HealthMonitor


def test_ccs_services_do_not_overwrite_real_risk_and_require_emergency_reset(tmp_path):
    rclpy.init(args=['--ros-args', '-r', '__ns:=/simulation_controls_contract',
        '-p', 'database:=' + str(tmp_path / 'tasks.db'), '-p', 'simulation_mode:=true',
        '-p', 'simulate_authority:=true',
        '-p', 'enable_simulation_tests:=true', '-p', 'require_radar:=false',
        '-p', 'regular_clear_time:=0.0', '-p', 'emergency_clear_time:=0.0'])
    controls, mission, gate, health, peer = SimulationControls(), Mission(), MotionGate(), HealthMonitor(), Node('peer')
    executor = SingleThreadedExecutor()
    nodes = (controls, mission, gate, health, peer)
    for node in nodes:
        executor.add_node(node)
    odom_pub = peer.create_publisher(Odometry, 'odom', 1)
    rule_pub = peer.create_publisher(MaritimeRuleDecision, 'autonomy/maritime', 1)
    def inputs():
        odom = Odometry()
        odom.header.stamp = peer.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        rule = MaritimeRuleDecision()
        rule.header.stamp = peer.get_clock().now().to_msg()
        rule.valid, rule.intervention_required = True, True
        rule_pub.publish(rule)
    peer.create_timer(.05, inputs)
    def until(predicate, timeout=4.):
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('simulation service timeout: ' + repr(mission.core.blockers))
    def change(name, value):
        client = peer.create_client(SetBool, 'autonomy/test/' + name)
        until(client.service_is_ready)
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        until(future.done)
        assert future.result().success
        peer.destroy_client(client)
    try:
        until(lambda: mission.authority_time > 0 and gate.gate.authority[1] > 0)
        # DDS discovery/callback order is asynchronous: establish the real rule
        # blocker before testing that synthetic toggles cannot clear it.
        until(lambda: mission.core.blockers.get('maritime'))
        assert not mission.core.authority
        change('authority', True)
        until(lambda: mission.core.authority and gate.gate.authority[0])
        change('maritime_wait', True)
        until(lambda: mission.core.blockers.get('maritime_test') and gate.test_maritime_wait)
        change('maritime_wait', False)
        until(lambda: 'maritime_test' not in mission.core.blockers and not gate.test_maritime_wait)
        assert mission.core.blockers['maritime']  # actual rule decision was not overwritten
        change('localization_failure', True)
        until(lambda: mission.core.blockers.get('regular_safety'))
        assert not health.policy.evaluate()[0]
        change('localization_failure', False)
        until(lambda: 'regular_safety' not in mission.core.blockers)
        change('emergency', True)
        until(lambda: gate.latched)
        reset = peer.create_client(ResetSafetyLatch, 'autonomy/reset_safety')
        until(reset.service_is_ready)
        request = ResetSafetyLatch.Request()
        request.confirmed = True
        future = reset.call_async(request)
        until(future.done)
        assert not future.result().accepted
        change('emergency', False)
        until(lambda: not gate.estop and gate.clear_since is not None)
        assert gate.latched
        future = reset.call_async(request)
        until(future.done)
        assert future.result().accepted and not gate.latched
        change('authority', False)
        until(lambda: not mission.core.authority and not gate.gate.authority[0])
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        mission.store.close()
        executor.shutdown()
        rclpy.shutdown()


def test_test_inputs_are_ignored_without_readonly_enable_flag(tmp_path):
    rclpy.init(args=['--ros-args', '-r', '__ns:=/disabled_simulation_contract',
                    '-p', 'database:=' + str(tmp_path / 'tasks.db')])
    mission, gate, health, peer = Mission(), MotionGate(), HealthMonitor(), Node('peer')
    executor = SingleThreadedExecutor()
    nodes = (mission, gate, health, peer)
    for node in nodes:
        executor.add_node(node)
    pubs = [peer.create_publisher(Bool, name, 1) for name in (
        'autonomy/test_maritime_wait', 'autonomy/test_localization_failure', 'autonomy/test_emergency')]
    peer.create_timer(.05, lambda: [pub.publish(Bool(data=True)) for pub in pubs])
    try:
        deadline = monotonic() + .5
        while monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert 'maritime_test' not in mission.core.blockers
        assert not gate.test_maritime_wait
        assert not gate.test_emergency
        assert 'simulation_test' not in health.policy.required_sources
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        mission.store.close()
        executor.shutdown()
        rclpy.shutdown()


def test_test_launch_cannot_compete_with_a_real_authority_adapter():
    path = Path(__file__).resolve().parents[1] / 'launch/autonomy.launch.py'
    spec = importlib.util.spec_from_file_location('test_autonomy_launch_guard', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations['simulation_test_controls'] = 'true'
    context.launch_configurations['simulation_authority'] = 'true'
    context.launch_configurations['simulation_thrusters'] = 'true'
    with pytest.raises(ValueError, match='no FCU authority adapter'):
        next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)


def test_fault_only_controls_cannot_publish_or_override_real_fcu_authority(tmp_path):
    from cc_msgs.msg import USVState
    from usv_autonomy.authority_node import AuthorityAdapter

    rclpy.init(args=['--ros-args', '-r', '__ns:=/fault_only_contract',
        '-p', 'database:=' + str(tmp_path / 'tasks.db'), '-p', 'simulation_mode:=true',
        '-p', 'simulate_authority:=false', '-p', 'enable_simulation_tests:=true',
        '-p', 'fcu_state_topic:=/fault_only_contract/cc/state'])
    controls, adapter, mission, gate, peer = SimulationControls(), AuthorityAdapter(), Mission(), MotionGate(), Node('peer')
    executor = SingleThreadedExecutor()
    nodes = (controls, adapter, mission, gate, peer)
    for node in nodes:
        executor.add_node(node)
    publisher = peer.create_publisher(USVState, 'cc/state', 1)
    real_estop = peer.create_publisher(Bool, 'safety/estop', 1)
    localization = []
    peer.create_subscription(Bool, 'autonomy/test_localization_failure', localization.append, 1)
    mode = ['OFFBOARD']

    def until(predicate, timeout=5.):
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            state = USVState()
            state.header.stamp = peer.get_clock().now().to_msg()
            state.connected = state.armed = True
            state.mode = mode[0]
            publisher.publish(state)
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('real authority / fault-only test timeout')

    def change(key, value):
        client = peer.create_client(SetBool, 'autonomy/test/' + key)
        until(client.service_is_ready)
        future = client.call_async(SetBool.Request(data=value))
        until(future.done)
        assert future.result().success
        peer.destroy_client(client)

    try:
        until(lambda: mission.core.authority and gate.gate.authority[0])
        assert controls.authority_pub is None and 'authority' not in controls.flags
        # The mission's request_manual CLIENT also advertises the name in the
        # global graph; check this node's actual service servers, not names alone.
        services = dict(peer.get_service_names_and_types_by_node(controls.get_name(), controls.get_namespace()))
        assert '/fault_only_contract/autonomy/test/authority' not in services
        assert '/fault_only_contract/control/request_manual' not in services
        publishers = peer.get_publishers_info_by_topic('/fault_only_contract/control/authority')
        assert len(publishers) == 1 and publishers[0].node_name == 'usv_authority_v2'
        response = controls.handler('authority')(SetBool.Request(data=True), SetBool.Response())
        assert not response.success and 'authority' not in controls.flags
        response = controls.request_manual(SetBool.Request(data=True), SetBool.Response())
        assert not response.success
        change('maritime_wait', True)
        until(lambda: mission.core.blockers.get('maritime_test') and gate.test_maritime_wait)
        assert mission.core.authority  # Fault injection did not overwrite the observed authority.
        change('localization_failure', True)
        until(lambda: any(msg.data for msg in localization))
        change('emergency', True)
        until(lambda: gate.test_emergency)
        real_estop.publish(Bool(data=True))
        until(lambda: gate.estop)
        change('emergency', False)
        until(lambda: not gate.test_emergency)
        assert gate.estop and gate.hazards()[1]  # Clearing the test cannot mask a real emergency.
        real_estop.publish(Bool(data=False))
        until(lambda: not gate.estop)
        mode[0] = 'MANUAL'
        until(lambda: not mission.core.authority and not gate.gate.authority[0])
        for key in ('maritime_wait', 'localization_failure', 'emergency'):
            change(key, False)
        assert not mission.core.authority  # Clearing a test fault cannot grant autonomous authority.
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        mission.store.close()
        executor.shutdown()
        rclpy.shutdown()


def test_launch_allows_fault_only_controls_with_real_adapter(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch/autonomy.launch.py'
    spec = importlib.util.spec_from_file_location('test_fault_only_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: captured.append(kwargs))
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(simulation_test_controls='true', simulation_thrusters='true',
        simulation_authority='false', authority_adapter='true', launch_nav2='false')
    next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)
    controls = next(n for n in captured if n['executable'] == 'simulation_controls')
    assert controls['parameters'][-1]['simulate_authority'] is False
    assert any(n['executable'] == 'authority_adapter' for n in captured)
