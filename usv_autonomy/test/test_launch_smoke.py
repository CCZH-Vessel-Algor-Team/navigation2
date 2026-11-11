"""Launch the packaged processes and real Gazebo thrust bridge without hardware."""
import signal
import subprocess
import json
from time import monotonic
import yaml
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64
from usv_interfaces.msg import ActuatorCommand, AutonomyState, MaritimeRuleDecision


@pytest.mark.parametrize('full_navigation,stand_on', [(False, False), (True, False), (True, True)])
def test_packaged_launch_unknown_authority_and_no_twist_bypass(tmp_path, full_navigation, stand_on):
    params = tmp_path / 'autonomy.yaml'
    legacy = tmp_path / 'old_maritime.json'
    legacy_data = json.dumps(dict(version=3, waiting_targets=['old-target'], event_id='old-handover',
                                  event_target='old-target', handover_reason='target_missing',
                                  responsibilities={'old-target': 1}))
    legacy.write_text(legacy_data)
    fresh = tmp_path / 'fresh_session' / 'maritime.json'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'rules_checkpoint': str(legacy)}}}))
    log = (tmp_path / 'launch.log').open('w+')
    nav_params = tmp_path / 'nav2.yaml'
    nav_params.write_text(yaml.safe_dump({
        'planner_server': {'ros__parameters': {'planner_plugins': ['VORRTStar', 'GridBased'],
            'GridBased': {'plugin': 'nav2_navfn_planner/NavfnPlanner'},
            'VORRTStar': {'plugin': 'nav2_navfn_planner/NavfnPlanner'}}},
        'controller_server': {'ros__parameters': {'controller_plugins': ['FollowPath'],
            'FollowPath': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'global_frame': 'map', 'robot_base_frame': 'smoke_v2/base_link',
            'width': 10, 'height': 10, 'resolution': .5, 'rolling_window': True,
            'plugins': ['inflation'], 'inflation': {'plugin': 'nav2_costmap_2d::InflationLayer'}}}},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'global_frame': 'map', 'robot_base_frame': 'smoke_v2/base_link',
            'width': 10, 'height': 10, 'resolution': .5, 'rolling_window': True,
            'plugins': ['inflation'], 'inflation': {'plugin': 'nav2_costmap_2d::InflationLayer'}}}},
    }))
    process = subprocess.Popen(['ros2', 'launch', 'usv_autonomy', 'autonomy.launch.py',
        'namespace:=smoke_v2', 'launch_nav2:=' + str(full_navigation).lower(), 'authority_adapter:=false',
        'left_crossing_stand_on:=' + str(stand_on).lower(),
        'nav2_params:=' + str(nav_params),
        'simulation_thrusters:=true', 'database:=' + str(fresh.parent / 'tasks.db'),
        'rules_checkpoint:=' + str(fresh),
        'autonomy_params:=' + str(params)], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    rclpy.init()
    peer = Node('launch_smoke_peer')
    outputs, left, right, states = [], [], [], []
    rules = []
    peer.create_subscription(ActuatorCommand, '/smoke_v2/control/actuator_command', outputs.append, 10)
    peer.create_subscription(Float64, '/smoke_v2/thrusters/left/thrust', left.append, 10)
    peer.create_subscription(Float64, '/smoke_v2/thrusters/right/thrust', right.append, 10)
    peer.create_subscription(AutonomyState, '/smoke_v2/autonomy/state', states.append, 10)
    peer.create_subscription(MaritimeRuleDecision, '/smoke_v2/autonomy/maritime', rules.append, 10)
    raw = peer.create_publisher(Twist, '/smoke_v2/cmd_vel', 10)
    def until(predicate):
        deadline = monotonic() + 10.
        while monotonic() < deadline:
            rclpy.spin_once(peer, timeout_sec=.02)
            if predicate():
                return
        log.flush()
        log.seek(0)
        raise AssertionError(log.read())
    try:
        until(lambda: outputs and left and right and states and rules)
        assert outputs[-1].intent == outputs[-1].DISABLED
        assert 'authority' in states[-1].blockers
        assert not rules[-1].handover_required and not rules[-1].intervention_required
        assert rules[-1].event_id == ''
        assert json.loads(fresh.read_text())['waiting_targets'] == []
        assert legacy.read_text() == legacy_data
        command = Twist()
        command.linear.x = 10.
        timer = peer.create_timer(.02, lambda: raw.publish(command))
        end = monotonic() + .4
        until(lambda: monotonic() >= end)
        assert all(msg.data == 0. for msg in left + right)
        assert all(msg.intent == msg.DISABLED for msg in outputs)
        peer.destroy_timer(timer)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            import os
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        peer.destroy_node()
        rclpy.shutdown()
        log.close()
