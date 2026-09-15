"""Verify the real TS launch, parameter services and geometry outputs in isolation."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList, ProcessedTSList
from nav2_colregs_msgs.srv import GetAvoidancePoint, GetBarrierLines
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParametersAtomically
from tf2_ros import StaticTransformBroadcaster
import yaml


def stop_process(proc):
    """Stop only the subprocess group owned by this test.

    :param proc: A process started with start_new_session enabled.
    """
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)


@pytest.mark.parametrize('threat_scale,expected_threat', [(2.0, True), (1.0, False)])
def test_ts_launch_and_outputs(tmp_path, monkeypatch, threat_scale, expected_threat):
    """Load nondefault YAML via a parent with a conflicting params_file.

    :param tmp_path: Pytest evidence directory.
    :param monkeypatch: Environment fixture.
    :param threat_scale: Manager-only distance multiplier.
    :param expected_threat: Expected classification for the fixed input scene.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('TS_TEST_ROS_DOMAIN_ID', '92'))
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    share = Path(get_package_share_directory('nav2_colregs_bringup'))
    config = yaml.safe_load((share / 'params/ts_subsystem.yaml').read_text())
    config['ts_state_manager']['ros__parameters'].update(
        update_frequency=20.0, track_list_timeout=2.5, threat_tcpa_horizon=40.0,
        threat_radius_scale=threat_scale, os_radius=4.0)
    config['avoidance_point_node']['ros__parameters'].update(
        avoidance_radius_scale=1.5 if expected_threat else 1.0,
        point_extension_distance=20.0)
    config['barrier_node']['ros__parameters'].update(
        lateral_margin=2.0, closing_segment_length=20.0)
    params = tmp_path / 'custom_ts.yaml'
    params.write_text(yaml.safe_dump(config))
    decoy = tmp_path / 'nav2_params.yaml'
    decoy.write_text('ts_state_manager:\n  ros__parameters:\n    threat_tcpa_horizon: 999.0\n')
    parent = tmp_path / 'parent.launch.py'
    parent.write_text(
        'from launch import LaunchDescription\n'
        'from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription\n'
        'from launch.launch_description_sources import PythonLaunchDescriptionSource\n'
        'from launch.substitutions import LaunchConfiguration\n'
        'def generate_launch_description():\n'
        '    return LaunchDescription([\n'
        f'        DeclareLaunchArgument("params_file", default_value={str(decoy)!r}),\n'
        f'        DeclareLaunchArgument("ts_params_file", default_value={str(params)!r}),\n'
        '        IncludeLaunchDescription(PythonLaunchDescriptionSource('
        f'{str(share / "launch/ts_subsystem_launch.py")!r}), launch_arguments={{\n'
        '            "ts_params_file": LaunchConfiguration("ts_params_file"),\n'
        '            "use_sim_time": "false", "robot_base_frame": "test_base",\n'
        '            "odom_topic": "/contract/odom",\n'
        '            "tracked_ship_topic": "/contract/ships"}.items())])\n')
    log_path = tmp_path / 'ts_launch.log'
    rclpy.init()
    node = rclpy.create_node('ts_contract_test')
    proc = None
    latest = []

    def wait(future):
        deadline = time.monotonic() + 8
        while not future.done() and time.monotonic() < deadline:
            assert proc.poll() is None, f'TS launch exited: {proc.returncode}'
            rclpy.spin_once(node, timeout_sec=0.05)
        assert future.done() and future.result() is not None
        return future.result()

    def call(service_type, name, request):
        client = node.create_client(service_type, name)
        try:
            assert client.wait_for_service(timeout_sec=10), name
            return wait(client.call_async(request))
        finally:
            node.destroy_client(client)

    def set_values(name, values, accepted):
        request = SetParametersAtomically.Request(parameters=[
            Parameter(key, value=value).to_parameter_msg() for key, value in values.items()])
        result = call(SetParametersAtomically, '/' + name + '/set_parameters_atomically', request)
        assert result.result.successful == accepted, result
        if not accepted:
            assert result.result.reason
        print(name, values, result.result, flush=True)

    try:
        with log_path.open('w') as log:
            proc = subprocess.Popen(['ros2', 'launch', str(parent)], stdout=log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
        broadcaster = StaticTransformBroadcaster(node)
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'test_base'
        tf.transform.rotation.w = 1.0
        broadcaster.sendTransform(tf)
        odom_pub = node.create_publisher(Odometry, '/contract/odom', 10)
        ships_pub = node.create_publisher(TrackedShipList, '/contract/ships', 10)
        node.create_subscription(ProcessedTSList, '/processed_ts_list',
                                 lambda msg: latest.append(msg), 10)

        def publish():
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.child_frame_id = 'test_base'
            odom.twist.twist.linear.x = 1.0
            odom_pub.publish(odom)
            ships = TrackedShipList()
            ships.header.frame_id = 'map'
            ships.header.stamp = node.get_clock().now().to_msg()
            ship = TrackedShip()
            ship.target_id.uuid[0] = 1
            ship.pose.position.x = 30.0
            ship.pose.position.y = 7.0
            ship.pose.orientation.w = 1.0
            ship.radius = 1.0
            ships.ships = [ship]
            ships_pub.publish(ships)

        node.create_timer(0.05, publish)
        for name, section in config.items():
            values = dict(section['ros__parameters'])
            if name == 'ts_state_manager':
                values.update(robot_base_frame='test_base', odom_topic='/contract/odom',
                              tracked_ship_topic='/contract/ships')
            result = call(GetParameters, '/' + name + '/get_parameters',
                          GetParameters.Request(names=list(values)))
            for key, value in zip(values, result.values):
                field = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
                assert getattr(value, field[value.type]) == values[key], (name, key, value)
            descriptors = call(DescribeParameters, '/' + name + '/describe_parameters',
                               DescribeParameters.Request(names=list(values))).descriptors
            assert all(d.description for d in descriptors)
            assert all(
                d.read_only == (name != 'avoidance_point_node' or
                                d.name in ('snapshot_timeout', 'max_request_position_delta'))
                for d in descriptors)
            print('YAML effective:', name, values, flush=True)
        set_values('ts_state_manager', {'threat_radius_scale': 9.0}, False)
        set_values('ts_state_manager', {'update_frequency': 5.0}, False)
        set_values('barrier_node', {'closing_segment_length': 9.0}, False)
        for invalid in (-1.0, 0.0, 0.5, float('nan'), float('inf'), 'bad'):
            set_values('avoidance_point_node', {'avoidance_radius_scale': invalid}, False)
        set_values('avoidance_point_node', {'point_extension_distance': -1.0}, False)

        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (latest and latest[-1].ships and
                    abs(latest[-1].ships[0].tcpa - 30.0) < 1e-6):
                break
        assert latest and latest[-1].ships
        assert latest[-1].os_radius == pytest.approx(4.0)
        target = latest[-1].ships[0]
        assert target.tcpa == pytest.approx(30.0)
        assert target.dcpa == pytest.approx(7.0)
        assert target.has_threat == expected_threat
        req = GetAvoidancePoint.Request()
        req.header.frame_id = 'map'
        req.goal.position.x = 100.0
        req.avoid_direction = 'right'
        response = call(GetAvoidancePoint, '/get_avoidance_point', req)
        assert response.has_feasible_angle == expected_threat
        if expected_threat:
            assert response.safe_heading == pytest.approx(math.radians(358))
            old_range = math.hypot(response.point.x, response.point.y)
            assert old_range == pytest.approx(math.sqrt(949) + 20.0)
            set_values('avoidance_point_node', {'avoidance_radius_scale': 2.0}, True)
            response = call(GetAvoidancePoint, '/get_avoidance_point', req)
            assert response.safe_heading == pytest.approx(math.radians(354))
            assert math.hypot(response.point.x, response.point.y) == pytest.approx(old_range)
            print('REAL radius inflation 1.5->2: heading -2deg->-6deg; point range unchanged',
                  flush=True)
            set_values('avoidance_point_node', {'point_extension_distance': 25.0}, True)
            response = call(GetAvoidancePoint, '/get_avoidance_point', req)
            assert response.safe_heading == pytest.approx(math.radians(354))
            assert math.hypot(response.point.x, response.point.y) == pytest.approx(old_range + 5.0)
        barrier_req = GetBarrierLines.Request()
        barrier_req.header = response.header
        barrier_req.snapshot_id = response.snapshot_id
        barrier_req.target_id = target.target_id
        barrier_req.avoid_direction = 'right'
        barrier = call(GetBarrierLines, '/get_barrier_lines', barrier_req).barriers.points
        assert len(barrier) == 6
        assert math.hypot(barrier[1].x - barrier[0].x,
                          barrier[1].y - barrier[0].y) == pytest.approx(3.0)
        assert math.hypot(barrier[3].x - barrier[2].x,
                          barrier[3].y - barrier[2].y) == pytest.approx(math.sqrt(949) + 3.0)
        assert math.hypot(barrier[5].x - barrier[4].x,
                          barrier[5].y - barrier[4].y) == pytest.approx(20.0)
        print(f'REAL manager: TCPA=30 DCPA=7 scale={threat_scale} threat={expected_threat}; '
              'barrier lengths=3m/20m', flush=True)
    finally:
        if proc is not None:
            stop_process(proc)
        node.destroy_node()
        rclpy.shutdown()
        if log_path.exists():
            print(log_path.read_text(), flush=True)


@pytest.mark.parametrize('executable,argument', [
    ('ts_state_manager', 'update_frequency:=0.0'),
    ('ts_state_manager', 'track_list_timeout:=0.0'),
    ('ts_state_manager', 'threat_radius_scale:=-1.0'),
    ('ts_state_manager', 'threat_radius_scale:=0.5'),
    ('ts_state_manager', 'os_radius:=-1.0'),
    ('barrier_node', 'closing_segment_length:=0.0'),
])
def test_invalid_startup(tmp_path, monkeypatch, executable, argument):
    """Reject invalid startup overrides in real TS executables.

    :param tmp_path: Pytest evidence directory.
    :param monkeypatch: Environment fixture.
    :param executable: TS executable name.
    :param argument: Invalid ROS parameter override.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('TS_TEST_ROS_DOMAIN_ID', '92'))
    binary = Path(get_package_prefix('nav2_colregs_ts_manager')) / 'lib/nav2_colregs_ts_manager'
    log_path = tmp_path / 'invalid.log'
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(binary / executable), '--ros-args', '-p', argument],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        assert proc.wait(timeout=5) != 0
        assert 'must be' in log_path.read_text()
        print(executable, argument, log_path.read_text(), flush=True)
    finally:
        stop_process(proc)
