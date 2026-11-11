"""Real CCS test services, isolated inputs, and protection/lifecycle semantics."""
import importlib.util
from pathlib import Path
from time import monotonic
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
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
        'autonomy/test_maritime_wait', 'autonomy/test_localization_failure')]
    peer.create_timer(.05, lambda: [pub.publish(Bool(data=True)) for pub in pubs])
    try:
        deadline = monotonic() + .5
        while monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert 'maritime_test' not in mission.core.blockers
        assert not gate.test_maritime_wait
        assert 'simulation_test' not in health.policy.required_sources
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        mission.store.close()
        executor.shutdown()
        rclpy.shutdown()


def test_test_launch_cannot_compete_with_a_real_authority_adapter():
    path = Path(get_package_share_directory('usv_autonomy')) / 'launch/autonomy.launch.py'
    spec = importlib.util.spec_from_file_location('test_autonomy_launch_guard', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations['simulation_test_controls'] = 'true'
    context.launch_configurations['simulation_thrusters'] = 'true'
    with pytest.raises(ValueError, match='no FCU authority adapter'):
        next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)
