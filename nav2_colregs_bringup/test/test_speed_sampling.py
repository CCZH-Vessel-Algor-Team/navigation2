"""Verify finite-speed VO decisions through the actual avoidance service."""

import math
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav2_colregs_msgs.msg import ProcessedTS, ProcessedTSList
from nav2_colregs_msgs.srv import GetAvoidancePoint
import pytest
import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter

from test_ts_parameter_contract import stop_process


class Probe:
    """Own one real service process and stamped synthetic observations."""

    def __init__(self, directory, tolerance=.3, count=5, alpha=1., initial=False):
        self.log = directory / 'node.log'
        executable = Path(get_package_prefix('nav2_colregs_ts_manager')) / \
            'lib/nav2_colregs_ts_manager/avoidance_point_node'
        with self.log.open('w') as output:
            self.process = subprocess.Popen([
                str(executable), '--ros-args', '-p', f'speed_tolerance:={tolerance}',
                '-p', f'speed_sample_count:={count}', '-p', f'heading_smoothing_alpha:={alpha}',
                '-p', 'smooth_initial_heading:=' + str(initial).lower()],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        self.node = rclpy.create_node('speed_sampling_probe')
        self.pub = self.node.create_publisher(ProcessedTSList, '/processed_ts_list', 10)
        self.client = self.node.create_client(GetAvoidancePoint, '/get_avoidance_point')
        self.state = ProcessedTSList()
        self.state.header.frame_id = 'map'
        self.state.valid = True
        self.state.snapshot_id.uuid[0] = 1
        self.state.os_radius = 5.
        self.state.os_pose.orientation.w = 1.
        self.state.os_twist.linear.x = 3.
        self.node.create_timer(.03, self.publish)

    def publish(self):
        """Refresh timestamps without changing the experimental geometry."""
        self.state.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(self.state)

    def spin(self, duration=.15):
        """Allow discovery and sensor delivery.

        :param duration: Wall-clock wait in seconds.
        """
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.log.read_text()
            rclpy.spin_once(self.node, timeout_sec=.01)

    def ships(self, entries):
        """Set target geometry; each entry is x, y, vx, vy, radius, threat.

        :param entries: Target tuples in the map frame.
        """
        self.state.ships = []
        self.state.snapshot_id.uuid[0] += 1
        for index, (x, y, vx, vy, radius, threat) in enumerate(entries):
            ship = ProcessedTS()
            ship.target_id.uuid[0] = index+1
            ship.pose.position.x, ship.pose.position.y = float(x), float(y)
            ship.twist.linear.x, ship.twist.linear.y = float(vx), float(vy)
            ship.radius, ship.has_threat = float(radius), threat
            ship.tcpa, ship.dcpa = 10., 0.
            self.state.ships.append(ship)
        self.spin()

    def call(self, goal=(100., 0.)):
        """Query the current synthetic snapshot.

        :param goal: Goal XY coordinates.
        :return: Real GetAvoidancePoint response.
        """
        assert self.client.wait_for_service(timeout_sec=8)
        self.spin()
        request = GetAvoidancePoint.Request()
        request.header.frame_id = 'map'
        request.os_pose = self.state.os_pose
        request.goal.position.x, request.goal.position.y = goal
        request.avoid_direction = 'right'
        future = self.client.call_async(request)
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            self.spin(.02)
        assert future.done()
        result = future.result()
        assert result.snapshot_id == self.state.snapshot_id
        return result

    def close(self):
        """Release only this probe's node and process group."""
        self.node.destroy_node()
        stop_process(self.process)


@pytest.fixture
def probe(tmp_path, monkeypatch, request):
    """Create a real-node probe with test-specific startup parameters.

    :param tmp_path: Evidence directory.
    :param monkeypatch: Environment fixture.
    :param request: Optional parameter dictionary.
    :return: Active probe, cleaned up after the test.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('SPEED_TEST_DOMAIN', '97'))
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    rclpy.init()
    instance = Probe(tmp_path, **getattr(request, 'param', {}))
    try:
        yield instance
    finally:
        instance.close()
        rclpy.shutdown()


def clearance(probe, heading, speed, ship):
    """Independently calculate future closest distance for one constant-speed model.

    :param probe: Snapshot owner.
    :param heading: OS heading in radians.
    :param speed: OS speed in metres per second.
    :param ship: Processed target.
    :return: Minimum future centre distance.
    """
    x = ship.pose.position.x-probe.state.os_pose.position.x
    y = ship.pose.position.y-probe.state.os_pose.position.y
    vx = ship.twist.linear.x-speed*math.cos(heading)
    vy = ship.twist.linear.y-speed*math.sin(heading)
    norm = vx*vx+vy*vy
    t = max(0., -(x*vx+y*vy)/norm) if norm > 1e-12 else 0.
    return math.hypot(x+t*vx, y+t*vy)


@pytest.mark.parametrize('probe', [{'count': 2}, {'count': 4}], indirect=True)
def test_interior_speed_changes_decision(probe):
    """Endpoints can be safe while an interior crossing speed is unsafe.

    :param probe: Actual avoidance node with a 0.3 m/s tolerance.
    """
    # Current speed 2.5, band 2.2..2.8: collision is at 2.6, not at either endpoint.
    probe.state.os_twist.linear.x = 2.5
    probe.state.os_radius = .05
    probe.ships([(40, -40, 0, 2.6, .05, True)])
    result = probe.call()
    assert result.status == result.SUCCESS
    count = 'speed_sample_count=2' in probe.log.read_text()
    direct = abs(math.atan2(math.sin(result.safe_heading), math.cos(result.safe_heading))) < 1e-8
    assert direct == count
    assert clearance(probe, 0., 2.6, probe.state.ships[0]) < .1


@pytest.mark.parametrize('probe', [{'count': 2}], indirect=True)
def test_measured_speed_and_secondary_target(probe):
    """Check the off-grid measured speed and an unflagged secondary target.

    :param probe: Real node with endpoint sampling.
    """
    probe.state.os_radius = .05
    probe.ships([(0, 100, 0, 0, .05, True), (40, -40, 0, 3, .05, False)])
    result = probe.call()
    assert result.status == result.SUCCESS
    assert abs(math.sin(result.safe_heading)) > .01
    for speed in (2.7, 3., 3.3):
        for ship in probe.state.ships:
            assert clearance(probe, result.safe_heading, speed, ship) > .15


@pytest.mark.parametrize('probe', [{'tolerance': 1.}], indirect=True)
def test_range_failure_does_not_shrink_radius(probe):
    """A zero-speed sample has no inflated-radius solution but nominal speed does.

    :param probe: Real node with a band including zero.
    """
    probe.state.os_twist.linear.x = 1.
    probe.ships([(40, 12, -3, 0, 5, True)])
    assert max(clearance(probe, i*math.pi/90, 1., probe.state.ships[0])
               for i in range(180)) > 15.
    result = probe.call()
    assert result.status == result.INESCAPABLE
    assert 'radius retained' in result.message
    assert 'retrying at scale=1.000' not in probe.log.read_text()


def test_physical_fallback_keeps_sample_checks(probe):
    """Preserve real physical fallback without dropping any sampled speed.

    :param probe: Real avoidance node.
    """
    probe.ships([(14, 0, 0, 0, 5, True)])
    result = probe.call()
    assert result.status == result.SUCCESS
    assert 'Physical-radius fallback' in result.message
    for speed in (2.7, 2.85, 3., 3.15, 3.3):
        assert clearance(probe, result.safe_heading, speed, probe.state.ships[0]) > 10.
    probe.ships([(9, 0, 0, 0, 5, True)])
    assert probe.call().status == result.INESCAPABLE


@pytest.mark.parametrize('probe', [{'alpha': .5, 'initial': True}], indirect=True)
def test_smoothing_cannot_invalidate_sampled_output(probe):
    """Use the checked raw candidate when interpolation would violate the samples.

    :param probe: Real node with initial smoothing enabled.
    """
    probe.ships([(40, -40, 0, 3, 5, True)])
    result = probe.call()
    assert result.status == result.SUCCESS
    assert 'smoothing rejected' in result.message
    for speed in (2.7, 2.85, 3., 3.15, 3.3):
        assert clearance(probe, result.safe_heading, speed, probe.state.ships[0]) > 15.


def test_low_speed_and_parameter_contract(probe):
    """Clamp at zero and reject runtime changes to the two startup settings.

    :param probe: Real node.
    """
    probe.state.os_twist.linear.x = .1
    probe.ships([(100, 100, 0, 0, 5, True)])
    assert probe.call().status == GetAvoidancePoint.Response.SUCCESS
    assert 'speed_min=0.000000 speed_max=0.400000' in probe.log.read_text()
    client = probe.node.create_client(
        SetParametersAtomically, '/avoidance_point_node/set_parameters_atomically')
    assert client.wait_for_service(timeout_sec=3)
    for name, value in [('speed_tolerance', .2), ('speed_sample_count', 7)]:
        future = client.call_async(SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg()]))
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            probe.spin(.02)
        assert future.done() and not future.result().result.successful


@pytest.mark.parametrize('setting', [
    'speed_tolerance:=-0.1', 'speed_tolerance:=.nan',
    'speed_sample_count:=1', 'speed_sample_count:=102'])
def test_invalid_startup(setting):
    """Reject settings that cannot define the documented sampling interval.

    :param setting: Invalid ROS parameter override.
    """
    executable = Path(get_package_prefix('nav2_colregs_ts_manager')) / \
        'lib/nav2_colregs_ts_manager/avoidance_point_node'
    result = subprocess.run([str(executable), '--ros-args', '-p', setting],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert 'speed_' in result.stderr
