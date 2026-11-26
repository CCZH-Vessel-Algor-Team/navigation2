"""Read-only cc/state adapter, source liveness and independent ROS output clock."""
from types import SimpleNamespace
from uuid import uuid4

import pytest
import rclpy
from cc_msgs.msg import USVState
from rclpy.parameter import Parameter
from rclpy.time import Time
from usv_interfaces.msg import ControlAuthorityState
import usv_autonomy.authority_node as authority_module
from usv_autonomy.authority_node import AuthorityAdapter


class Clock:
    def __init__(self, ns):
        self.ns = ns

    def now(self):
        return Time(nanoseconds=self.ns)


@pytest.fixture
def adapter(monkeypatch):
    rclpy.init()
    node = AuthorityAdapter(namespace='authority_' + uuid4().hex[:8])
    source = Clock(1_794_000_000_000_000_000)
    output = Clock(123_000_000_000)
    steady = SimpleNamespace(now=10.0)
    node.input_clock = source
    monkeypatch.setattr(node, 'get_clock', lambda: output)
    monkeypatch.setattr(authority_module, 'monotonic', lambda: steady.now)
    messages = []
    node.pub = SimpleNamespace(publish=messages.append)
    try:
        yield node, source, output, steady, messages
    finally:
        node.destroy_node()
        rclpy.shutdown()


def state(source, mode='OFFBOARD', armed=True, connected=True, offset=0):
    msg = USVState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source.ns + offset, 1_000_000_000)
    msg.mode, msg.armed, msg.connected = mode, armed, connected
    return msg


@pytest.mark.parametrize('mode,armed,connected,expected', [
    ('OFFBOARD', True, True, True), ('GUIDED', True, True, True),
    ('MANUAL', True, True, False), ('POSCTL', True, True, False),
    ('OFFBOARD', False, True, False), ('OFFBOARD', True, False, False),
])
def test_real_fields_gate_authority_and_preserve_bt_clock(adapter, mode, armed, connected, expected):
    node, source, output, _, messages = adapter
    node.receive(state(source, mode, armed, connected))
    node.tick()
    msg = messages[-1]
    assert msg.valid and msg.motion_allowed == expected
    assert (msg.connected, msg.armed, msg.mode) == (connected, armed, mode)
    assert msg.header.stamp.sec == 123  # NOT the wall-clock source epoch.
    if not connected:
        assert msg.authority == msg.UNKNOWN
    elif mode == 'MANUAL':
        assert msg.authority == msg.EXTERNAL


def test_missing_input_is_unknown(adapter):
    node, _, _, _, messages = adapter
    node.tick()
    assert not messages[-1].valid and not messages[-1].motion_allowed
    assert messages[-1].authority == ControlAuthorityState.UNKNOWN


@pytest.mark.parametrize('offset', [-2_000_000_000, 2_000_000_000])
def test_old_or_future_input_immediately_revokes_cached_permission(adapter, offset):
    node, source, _, _, messages = adapter
    node.receive(state(source))
    node.tick()
    assert messages[-1].motion_allowed
    node.receive(state(source, offset=offset))
    node.tick()
    assert not messages[-1].valid and not messages[-1].motion_allowed


def test_replayed_stamps_do_not_renew_watchdog_with_frozen_ros_clock(adapter):
    node, source, _, steady, messages = adapter
    node.receive(state(source))
    for _ in range(6):
        steady.now += .25
        node.receive(state(source))  # Even repeated OFFBOARD cannot renew the lease.
        node.tick()
    assert not messages[-1].valid and not messages[-1].motion_allowed


def test_source_clock_jump_and_output_clock_rewind_require_new_observation(adapter):
    node, source, output, _, messages = adapter
    node.receive(state(source))
    source.ns += 2_000_000_000
    node.tick()
    assert not messages[-1].valid
    node.receive(state(source))
    node.tick()
    assert messages[-1].motion_allowed
    output.ns -= 1_000_000_000
    node.tick()
    assert not messages[-1].motion_allowed
    source.ns += 1_000_000
    node.receive(state(source))
    node.tick()
    assert messages[-1].motion_allowed


def test_fresh_disconnection_and_manual_mode_revoke_without_timeout(adapter):
    node, source, _, _, messages = adapter
    node.receive(state(source))
    source.ns += 1_000_000
    node.receive(state(source, connected=False))
    node.tick()
    assert not messages[-1].motion_allowed
    source.ns += 1_000_000
    node.receive(state(source, mode='MANUAL'))
    node.tick()
    assert messages[-1].valid and messages[-1].authority == ControlAuthorityState.EXTERNAL


def test_explicit_input_topic_and_ros_clock_can_be_configured():
    rclpy.init()
    node = AuthorityAdapter(namespace='configured_' + uuid4().hex[:8], parameter_overrides=[
        Parameter('fcu_state_topic', value='/test_fcu/cc/state'),
        Parameter('fcu_state_clock', value='ros'),
        Parameter('use_sim_time', value=True),
    ])
    try:
        assert node.input_clock is node.get_clock()
        topics = [name for name, _ in node.get_subscriber_names_and_types_by_node(
            node.get_name(), node.get_namespace())]
        assert '/test_fcu/cc/state' in topics
        assert not node.get_client_names_and_types_by_node(node.get_name(), node.get_namespace())
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_system_clock_cc_state_drives_real_mission_and_gate_in_sim_time(tmp_path):
    from time import monotonic
    from rclpy.clock import Clock as RosClock, ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock as ClockMessage
    from usv_autonomy.mission_node import Mission
    from usv_autonomy.gate_node import MotionGate

    topic = '/test_fcu_' + uuid4().hex[:8] + '/cc/state'
    rclpy.init(args=['--ros-args', '-r', '__ns:=/authority_source_integration',
                    '-p', 'database:=' + str(tmp_path / 'tasks.db'),
                    '-p', 'use_sim_time:=true', '-p', 'state_timeout:=0.25',
                    '-p', 'fcu_state_topic:=' + topic])
    adapter, mission, gate, peer = AuthorityAdapter(), Mission(), MotionGate(), Node('peer')
    executor = SingleThreadedExecutor()
    nodes = (adapter, mission, gate, peer)
    for node in nodes:
        executor.add_node(node)
    publisher = peer.create_publisher(USVState, topic, 1)
    clock_pub = peer.create_publisher(ClockMessage, '/clock', 10)
    system_clock = RosClock(clock_type=ClockType.SYSTEM_TIME)
    started = monotonic()
    flags = dict(send=True, mode='OFFBOARD', connected=True, armed=True)

    def until(predicate, timeout=5.0):
        end = monotonic() + timeout
        while monotonic() < end:
            clock = ClockMessage()
            clock.clock = Time(seconds=100 + monotonic() - started).to_msg()
            clock_pub.publish(clock)
            if flags['send']:
                msg = USVState()
                msg.header.stamp = system_clock.now().to_msg()
                msg.connected, msg.armed, msg.mode = flags['connected'], flags['armed'], flags['mode']
                publisher.publish(msg)
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('authority did not propagate to mission and motion gate')

    try:
        until(lambda: mission.core.authority and gate.gate.authority[0])
        assert 100 <= adapter.get_clock().now().nanoseconds * 1e-9 < 1000
        assert adapter.state.header.stamp.sec > 1_000_000_000
        flags['mode'] = 'MANUAL'
        until(lambda: not mission.core.authority and not gate.gate.authority[0])
        flags['mode'] = 'OFFBOARD'
        until(lambda: mission.core.authority and gate.gate.authority[0])
        flags['send'] = False
        until(lambda: not mission.core.authority and not gate.gate.authority[0])
        flags.update(send=True, connected=False)
        until(lambda: adapter.state is not None and not adapter.state.connected)
        assert not mission.core.authority and not gate.gate.authority[0]
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        mission.store.close()
        executor.shutdown()
        rclpy.shutdown()
