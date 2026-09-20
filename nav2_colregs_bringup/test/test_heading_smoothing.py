"""Exercise real-node heading damping and encounter-history reset behavior."""

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

from test_ts_parameter_contract import stop_process


def difference(a, b):
    """Return the circular difference of two headings.

    :param a: First heading.
    :param b: Reference heading.
    :return: Difference in radians.
    """
    return math.atan2(math.sin(a-b), math.cos(a-b))


@pytest.mark.parametrize('alpha,initial', [(1.0, False), (0.5, False), (0.5, True)])
def test_heading_history(tmp_path, monkeypatch, alpha, initial):
    """Check two encounters, query-free clears, expiry, wrap and low-speed yaw.

    :param tmp_path: Evidence directory.
    :param monkeypatch: Environment fixture.
    :param alpha: Output blend coefficient.
    :param initial: Whether the first output is blended from measured course.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('DAMPING_TEST_DOMAIN', '96'))
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    executable = Path(get_package_prefix('nav2_colregs_ts_manager')) / \
        'lib/nav2_colregs_ts_manager/avoidance_point_node'
    with (tmp_path / 'node.log').open('w') as log:
        proc = subprocess.Popen([
            str(executable), '--ros-args', '-p', f'heading_smoothing_alpha:={alpha}',
            '-p', 'smooth_initial_heading:=' + str(initial).lower(),
            '-p', 'snapshot_timeout:=0.4'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('heading_history_test')
    pub = node.create_publisher(ProcessedTSList, '/processed_ts_list', 10)
    client = node.create_client(GetAvoidancePoint, '/get_avoidance_point')
    state = ProcessedTSList()
    enabled = True

    def publish():
        if enabled:
            state.header.stamp = node.get_clock().now().to_msg()
            pub.publish(state)

    node.create_timer(0.03, publish)

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert proc.poll() is None
            rclpy.spin_once(node, timeout_sec=0.01)

    def scene(angle=0.):
        nonlocal state
        state = ProcessedTSList()
        state.valid = True
        state.header.frame_id = 'map'
        state.snapshot_id.uuid[0] = 1
        state.os_radius = 5.
        state.os_pose.orientation.z = math.sin(angle/2)
        state.os_pose.orientation.w = math.cos(angle/2)
        state.os_twist.linear.x = 3*math.cos(angle)
        state.os_twist.linear.y = 3*math.sin(angle)
        ship = ProcessedTS()
        ship.target_id.uuid[0] = 1
        ship.pose.position.x = 40*math.cos(angle)+40*math.sin(angle)
        ship.pose.position.y = 40*math.sin(angle)-40*math.cos(angle)
        ship.twist.linear.x = -3*math.sin(angle)
        ship.twist.linear.y = 3*math.cos(angle)
        ship.radius = 5.
        ship.tcpa = 40/3
        ship.dcpa = 0.
        ship.has_threat = True
        state.ships = [ship]
        spin(.12)

    def call(angle=0., x=0.):
        request = GetAvoidancePoint.Request()
        request.header.frame_id = 'map'
        request.os_pose.position.x = x
        request.goal.position.x = 100*math.cos(angle)
        request.goal.position.y = 100*math.sin(angle)
        request.avoid_direction = 'right'
        future = client.call_async(request)
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            spin(.02)
        assert future.done()
        return future.result()

    def first_expected(angle):
        return angle-math.radians(32)*(alpha if initial else 1.)

    def check_first(angle):
        response = call(angle)
        assert response.status == response.SUCCESS
        assert abs(difference(response.safe_heading, first_expected(angle))) < 1e-6
        assert abs(math.hypot(response.point.x, response.point.y)-(math.hypot(40, 40)+20)) < 1e-6
        print('first', angle, response.safe_heading, response.message, flush=True)
        return response.safe_heading

    try:
        assert client.wait_for_service(timeout_sec=10)
        deadline = time.monotonic()+5
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            spin(.02)
        assert pub.get_subscription_count() > 0
        scene()
        first = check_first(0.)
        second = call().safe_heading
        expected = first+alpha*difference(-math.radians(32), first)
        assert abs(difference(second, expected)) < 1e-6

        # No service request during this no-threat interval: callback must clear.
        state.ships[0].has_threat = False
        spin(.15)
        scene(math.pi/2)
        check_first(math.pi/2)

        # Explicit NO_THREAT followed by another encounter.
        state.ships[0].has_threat = False
        spin(.12)
        assert call().status == GetAvoidancePoint.Response.NO_THREAT
        scene(-math.pi/2)
        check_first(-math.pi/2)

        # Empty list, invalid snapshot, stale stream, and rejected request reset.
        state.ships = []
        spin(.12)
        scene()
        check_first(0.)
        state.valid = False
        spin(.12)
        scene(math.pi/2)
        check_first(math.pi/2)
        enabled = False
        spin(.8)
        enabled = True
        scene()
        check_first(0.)
        assert call(x=10.).status == GetAvoidancePoint.Response.INVALID_REQUEST
        check_first(0.)

        # 179 -> -179 degrees must take the short circular update.
        state.ships = []
        spin(.12)
        scene(math.radians(179))
        state.ships[0].pose.position.x = 0.
        state.ships[0].pose.position.y = -100.
        state.ships[0].twist.linear.x = 0.
        state.ships[0].twist.linear.y = 0.
        spin(.12)
        assert abs(difference(call(math.radians(179)).safe_heading, math.radians(179))) < 1e-6
        wrapped = call(math.radians(-179)).safe_heading
        assert abs(difference(wrapped, math.radians(179+2*alpha))) < 1e-6

        # At zero speed a safe, flagged synthetic target isolates yaw fallback.
        state.ships = []
        spin(.12)
        scene(math.pi/2)
        state.os_twist.linear.x = state.os_twist.linear.y = 0.
        state.ships[0].pose.position.x = state.ships[0].pose.position.y = 100.
        state.ships[0].twist.linear.x = state.ships[0].twist.linear.y = 0.
        spin(.12)
        response = call()
        assert response.status == response.SUCCESS
        expected = (1-alpha)*math.pi/2 if initial else 0.
        assert abs(difference(response.safe_heading, expected)) < 1e-6

        # Existing filter history must not dilute a physical-radius fallback.
        state.ships = []
        spin(.12)
        scene()
        check_first(0.)
        state.ships[0].pose.position.x = 14.
        state.ships[0].pose.position.y = 0.
        state.ships[0].twist.linear.x = state.ships[0].twist.linear.y = 0.
        spin(.12)
        fallback = call()
        assert fallback.status == fallback.SUCCESS
        assert 'Physical-radius fallback' in fallback.message
        assert abs(difference(fallback.safe_heading, math.radians(-46))) < 1e-6
        assert math.hypot(fallback.point.x, fallback.point.y) == pytest.approx(34.)
        assert 14*abs(math.sin(fallback.safe_heading)) > 10.

        # The retry checks every target, using physical radii for all of them.
        secondary = ProcessedTS()
        secondary.target_id.uuid[0] = 2
        secondary.pose.position.x = 30*math.cos(math.radians(-46))
        secondary.pose.position.y = 30*math.sin(math.radians(-46))
        secondary.radius = 1.
        state.ships.append(secondary)
        spin(.12)
        fallback = call()
        assert fallback.status == fallback.SUCCESS
        assert abs(difference(fallback.safe_heading, math.radians(-58))) < 1e-6
        assert 30*abs(math.sin(fallback.safe_heading-math.radians(-46))) > 6.

        # When the configured scale becomes feasible again, blend from the hard
        # fallback output. The configured scale itself must not be overwritten.
        scene()
        resumed = call()
        assert resumed.status == resumed.SUCCESS
        assert 'fallback' not in resumed.message
        expected = fallback.safe_heading+alpha*difference(-math.radians(32), fallback.safe_heading)
        assert abs(difference(resumed.safe_heading, expected)) < 1e-6

        # Even on the first request, fallback bypasses initial-heading smoothing.
        state.ships = []
        spin(.12)
        scene()
        state.ships[0].pose.position.x = 14.
        state.ships[0].pose.position.y = 0.
        state.ships[0].twist.linear.x = state.ships[0].twist.linear.y = 0.
        spin(.12)
        assert abs(difference(call().safe_heading, math.radians(-46))) < 1e-6
        for distance in (10., 9.):
            state.ships[0].pose.position.x = distance
            spin(.12)
            failed = call()
            assert failed.status == failed.INESCAPABLE
            assert not failed.has_feasible_angle
        scene()
        check_first(0.)

        # Retry is also needed outside the inflated domain: with zero OS speed,
        # this target's future 12m pass fails 15m but satisfies physical 10m.
        state.ships = []
        spin(.12)
        scene(math.pi/2)
        state.os_twist.linear.x = state.os_twist.linear.y = 0.
        state.ships[0].pose.position.x = 40.
        state.ships[0].pose.position.y = 12.
        state.ships[0].twist.linear.x = -3.
        state.ships[0].twist.linear.y = 0.
        spin(.12)
        fallback = call()
        assert fallback.status == fallback.SUCCESS
        assert 'Physical-radius fallback' in fallback.message
        assert abs(difference(fallback.safe_heading, 0.)) < 1e-6
        state.ships[0].pose.position.y = 9.
        spin(.12)
        assert call().status == GetAvoidancePoint.Response.INESCAPABLE

        log_text = (tmp_path / 'node.log').read_text()
        assert '[WARN]' in log_text and 'retrying at scale=1.000' in log_text
        assert 'effective_scale=1.000 fallback=1' in log_text
        warnings = log_text.count('retrying at scale=1.000')
        state.valid = False
        spin(.12)
        assert call().status == GetAvoidancePoint.Response.STALE_STATE
        assert (tmp_path / 'node.log').read_text().count('retrying at scale=1.000') == warnings
        print('PASS: physical fallback hard set, secondary target, history resume, '
              'physical rejection, future conflict, warning and stale-state gate', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_process(proc)
