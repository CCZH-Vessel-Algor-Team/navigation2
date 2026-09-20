"""Real-node COLREGS decision, freshness, geometry and planner failure regressions."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import SetParametersAtomically
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList, ProcessedTSList
from nav2_colregs_msgs.srv import GetAvoidancePoint, GetBarrierLines
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses
from nav_msgs.msg import Odometry, OccupancyGrid
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import MarkerArray, Marker
import yaml


def pose(x, y):
    """Construct a map pose.

    :param x: X coordinate in metres.
    :param y: Y coordinate in metres.
    :return: PoseStamped with identity orientation.
    """
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


class Scene:
    """Own the test's ROS nodes/processes and synthetic sensor inputs."""

    def __init__(self, directory, start_manager=True, parameters=None):
        self.directory = directory
        self.node = rclpy.create_node('decision_test')
        self.processes = {}
        self.enabled = True
        self.odom_enabled = True
        self.track_age = 0.0
        self.odom_age = 0.0
        self.frame = 'map'
        self.velocity = (1.0, 0.0)
        self.ships = [(30.0, 20.0, 0.0, 0.0, 1.0)]
        self.state = None
        self.states = {}
        self.avoidance_markers = []
        self.barrier_markers = []
        self.cpa_markers = []
        self.node.create_subscription(ProcessedTSList, '/processed_ts_list', self.receive, 10)
        for topic, target in [('avoidance_point_marker', self.avoidance_markers),
                              ('barrier_markers', self.barrier_markers),
                              ('cpa_markers', self.cpa_markers)]:
            self.node.create_subscription(
                MarkerArray, '/' + topic,
                lambda msg, target=target: target.extend(msg.markers), 10)
        self.odom = self.node.create_publisher(Odometry, '/decision/odom', 10)
        self.tracks = self.node.create_publisher(TrackedShipList, '/decision/ships', 10)
        self.broadcaster = StaticTransformBroadcaster(self.node)
        transforms = []
        for frame, x, y, yaw in [('test_base', 20., 20., 0.),
                                 ('sensor', 20., 20., math.pi / 2)]:
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation.z = math.sin(yaw / 2)
            transform.transform.rotation.w = math.cos(yaw / 2)
            transforms.append(transform)
        self.broadcaster.sendTransform(transforms)
        config = {
            'ts_state_manager': {'ros__parameters': {
                'use_sim_time': False, 'update_frequency': 20.0,
                'track_list_timeout': 0.5, 'own_ship_state_timeout': 0.5,
                'threat_tcpa_horizon': 30.0, 'threat_radius_scale': 1.5, 'os_radius': 1.0,
                'global_frame': 'map', 'robot_base_frame': 'test_base',
                'odom_topic': '/decision/odom', 'tracked_ship_topic': '/decision/ships'}},
            'avoidance_point_node': {'ros__parameters': {
                'avoidance_radius_scale': 1.0, 'point_extension_distance': 3.0,
                'snapshot_timeout': 0.6}},
            'barrier_node': {'ros__parameters': {
                'lateral_margin': 1.0, 'closing_segment_length': 30.0, 'snapshot_timeout': 0.6}}}
        for name, values in (parameters or {}).items():
            config[name]['ros__parameters'].update(values)
        params = directory / 'ts.yaml'
        params.write_text(yaml.safe_dump(config))
        for executable in ('avoidance_point_node', 'barrier_node'):
            self.spawn('nav2_colregs_ts_manager', executable, ['--params-file', str(params)])
        self.until(lambda: {'avoidance_point_node', 'barrier_node'} <= {
            info.node_name for info in
            self.node.get_subscriptions_info_by_topic('/processed_ts_list')})
        if start_manager:
            self.spawn('nav2_colregs_ts_manager', 'ts_state_manager',
                       ['--params-file', str(params)])
        self.node.create_timer(0.05, self.publish)

    def receive(self, msg):
        """Store the latest processed snapshot.

        :param msg: ProcessedTSList message.
        """
        self.state = msg
        self.states[bytes(msg.snapshot_id.uuid)] = msg
        if len(self.states) > 128:
            self.states.pop(next(iter(self.states)))

    def spawn(self, package, executable, args):
        """Start a real ROS executable and retain its log.

        :param package: ROS package name.
        :param executable: Executable name.
        :param args: ROS arguments.
        """
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        with (self.directory / (executable + '.log')).open('w') as log:
            self.processes[executable] = subprocess.Popen(
                [str(path), '--ros-args'] + args, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)

    def publish(self):
        """Publish stamped input snapshots while the relevant stream is enabled."""
        now = self.node.get_clock().now()
        if self.odom_enabled:
            msg = Odometry()
            msg.header.stamp = (now - rclpy.duration.Duration(seconds=self.odom_age)).to_msg()
            msg.child_frame_id = 'test_base'
            msg.twist.twist.linear.x, msg.twist.twist.linear.y = self.velocity
            self.odom.publish(msg)
        if self.enabled:
            msg = TrackedShipList()
            msg.header.stamp = (now - rclpy.duration.Duration(seconds=self.track_age)).to_msg()
            msg.header.frame_id = self.frame
            for i, (x, y, vx, vy, radius) in enumerate(self.ships):
                ship = TrackedShip()
                ship.target_id.uuid[0] = i + 1
                ship.pose.position.x, ship.pose.position.y = float(x), float(y)
                ship.pose.orientation.w = 1.0
                ship.twist.linear.x, ship.twist.linear.y = float(vx), float(vy)
                ship.radius = float(radius)
                msg.ships.append(ship)
            self.tracks.publish(msg)

    def until(self, condition, timeout=5.):
        """Spin with a bounded wait.

        :param condition: Predicate to await.
        :param timeout: Maximum wait in seconds.
        """
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        assert condition(), 'Timed out waiting for test condition'

    def call(self, service_type, name, request):
        """Make a bounded service call.

        :param service_type: ROS service type.
        :param name: Fully qualified service name.
        :param request: Request message.
        :return: Response message.
        """
        client = self.node.create_client(service_type, name)
        try:
            assert client.wait_for_service(timeout_sec=5)
            future = client.call_async(request)
            self.until(future.done)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def avoid(self, goal=(60., 20.), frame='map', direction='right'):
        """Request an avoidance decision at the live OS pose.

        :param goal: Goal XY coordinates.
        :param frame: Request frame.
        :param direction: Avoidance side.
        :return: GetAvoidancePoint response.
        """
        req = GetAvoidancePoint.Request()
        req.header.frame_id = frame
        req.os_pose = pose(20, 20).pose
        req.goal = pose(*goal).pose
        req.avoid_direction = direction
        result = self.call(GetAvoidancePoint, '/get_avoidance_point', req)
        deadline = time.monotonic() + 2.0
        # The observer receiving a first sample does not prove this service has
        # discovered the writer yet. Wait for startup only; never mask stale data.
        while (result.status == result.NO_DATA and self.state is not None and self.state.valid
               and time.monotonic() < deadline):
            rclpy.spin_once(self.node, timeout_sec=0.05)
            result = self.call(GetAvoidancePoint, '/get_avoidance_point', req)
        return result

    def ready_decision(self):
        """Establish a snapshot known to both services before testing its retention.

        :return: Successful avoidance response already observed by the barrier node.
        """
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            decision = self.avoid()
            if decision.status == decision.SUCCESS:
                result = self.barrier(decision)
                if result.status == result.SUCCESS:
                    return decision
            rclpy.spin_once(self.node, timeout_sec=0.05)
        raise AssertionError('Services did not establish a shared snapshot')

    def barrier(self, decision, target_id=None):
        """Request the barrier for one exact avoidance snapshot.

        :param decision: Prior avoidance response.
        :param target_id: Optional target override.
        :return: GetBarrierLines response.
        """
        req = GetBarrierLines.Request()
        req.header = decision.header
        req.snapshot_id = decision.snapshot_id
        req.target_id = decision.primary_target_id if target_id is None else target_id
        req.os_pose = pose(20, 20).pose
        req.avoid_direction = 'right'
        return self.call(GetBarrierLines, '/get_barrier_lines', req)

    def start_planner(self, blocked=False, origin_y=0.0, static_discs=(), blocked_tail=False):
        """Start a standard planner_server with a real static costmap.

        :param blocked: Place an obstacle across the deterministic avoidance leg.
        :param origin_y: Costmap origin y coordinate.
        :param static_discs: Optional static obstacle discs as (x, y, radius).
        :param blocked_tail: Separate AP from the goal with a full-height costmap wall.
        """
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(OccupancyGrid, '/decision_map', qos)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.5
        grid.info.width = grid.info.height = 200
        grid.info.origin.orientation.w = 1.0
        grid.info.origin.position.y = origin_y
        grid.data = [0] * 40000
        for x, y, radius in static_discs:
            for row in range(200):
                for col in range(200):
                    if math.hypot((col+0.5)*0.5-x, origin_y+(row+0.5)*0.5-y) <= radius:
                        grid.data[row*200+col] = 100
        if blocked:
            for x in range(48, 53):
                for y in range(36, 41):
                    grid.data[y * 200 + x] = 100
        if blocked_tail:
            for row in range(200):
                grid.data[row * 200 + 90] = 100
        self.map_pub.publish(grid)
        config = {
            '/decision/planner_server': {'ros__parameters': {
                'planner_plugins': ['VO', 'Skeleton'],
                'VO': {'plugin': 'nav2_colregs_vo_rrt_star_planner::VORRTStarPlanner',
                       'max_iterations': 100, 'max_optimize_iters': 0,
                       'step_size': 4.0, 'eta': 50.0, 'safety_dist': 0.5},
                'Skeleton': {'plugin': 'nav2_colregs_vo_skeleton_planner/VOSkeletonPlanner',
                             'safety_dist': 0.5}}},
            '/decision/global_costmap/global_costmap': {'ros__parameters': {
                'global_frame': 'map', 'robot_base_frame': 'test_base', 'robot_radius': 0.5,
                'update_frequency': 20.0, 'plugins': ['static_layer'],
                'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer',
                                 'map_topic': '/decision_map',
                                 'map_subscribe_transient_local': True}}}}
        params = self.directory / 'planner.yaml'
        params.write_text(yaml.safe_dump(config))
        self.spawn('nav2_planner', 'planner_server',
                   ['-r', '__ns:=/decision', '--params-file', str(params)])
        for transition in (1, 3):
            req = ChangeState.Request()
            req.transition.id = transition
            assert self.call(ChangeState, '/decision/planner_server/change_state', req).success

    def plan(self, planner, through=False, goal=(60., 20.)):
        """Run a real planning action and return its final status.

        :param planner: Plugin ID.
        :param through: Use a multi-waypoint planning action.
        :param goal: Single-goal XY coordinates.
        :return: Action result wrapper.
        """
        kind = ComputePathThroughPoses if through else ComputePathToPose
        name = 'compute_path_through_poses' if through else 'compute_path_to_pose'
        client = ActionClient(self.node, kind, '/decision/' + name)
        try:
            assert client.wait_for_server(timeout_sec=5)
            req = kind.Goal()
            req.planner_id = planner
            if through:
                req.goals = [pose(35, 20), pose(50, 20)]
            else:
                req.goal = pose(*goal)
            future = client.send_goal_async(req)
            self.until(future.done)
            handle = future.result()
            assert handle.accepted
            future = handle.get_result_async()
            self.until(future.done, 10)
            assert self.processes['planner_server'].poll() is None
            return future.result()
        finally:
            client.destroy()

    def stop(self, name):
        """Stop one owned subprocess.

        :param name: Executable name.
        """
        proc = self.processes[name]
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)

    def close(self):
        """Stop owned nodes and retain their logs."""
        for name in reversed(self.processes):
            self.stop(name)
        self.node.destroy_node()
        print(f'Runtime evidence: {self.directory}', flush=True)


@pytest.fixture
def scene(tmp_path, monkeypatch, request):
    """Create an isolated real-node scene.

    :param tmp_path: Evidence directory.
    :param monkeypatch: Environment fixture.
    :param request: Optional fixture settings.
    :yield: Scene harness.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('COLREGS_TEST_ROS_DOMAIN_ID', '93'))
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    rclpy.init()
    options = getattr(request, 'param', {})
    if isinstance(options, bool):
        options = {'start_manager': options}
    harness = Scene(tmp_path, **options)
    try:
        yield harness
    finally:
        harness.close()
        rclpy.shutdown()


def test_heading_boundary_and_secondary_target(scene):
    """Reject unsafe sampled boundaries and revalidate against secondary targets.

    :param scene: Real-node harness.
    """
    scene.ships = [(30., 20., 0., 0., 5.)]
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    result = scene.avoid()
    assert result.status == result.SUCCESS
    assert result.safe_heading == pytest.approx(math.radians(322))
    assert 10 * abs(math.sin(result.safe_heading)) > 6.0
    angle = math.radians(-40)
    scene.ships.append((20 + 10 * math.cos(angle), 20 + 10 * math.sin(angle), 0., 0., 1.))
    scene.until(lambda: len(scene.state.ships) == 2)
    result = scene.avoid()
    assert result.status == result.SUCCESS
    assert result.safe_heading == pytest.approx(math.radians(308))
    assert 10 * abs(math.sin(result.safe_heading - angle)) > 2.0
    print('PASS: unsafe -36deg boundary rejected; secondary target checked', flush=True)


def test_narrow_cone_between_samples_is_not_treated_as_safe(scene):
    """An empty sampled cone must not bypass the continuous heading check.

    :param scene: Real-node harness.
    """
    angle = math.radians(1.)
    scene.velocity = (10 * math.cos(angle), 10 * math.sin(angle))
    scene.ships = [(20 + 100 * math.cos(angle), 20 + 100 * math.sin(angle), 0., 0., 0.01)]
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    assert scene.state.ships[0].has_threat
    assert not scene.state.ships[0].collision_cone_min
    result = scene.avoid(goal=(20 + 200 * math.cos(angle), 20 + 200 * math.sin(angle)))
    assert result.status == result.SUCCESS
    assert result.safe_heading == pytest.approx(math.radians(359.))
    print('PASS: narrow unsampled cone and angle wrapping revalidated', flush=True)


@pytest.mark.parametrize('velocity,ships,expected', [
    ((0., 0.), [(30., 20., -1., 0., 1.)], GetAvoidancePoint.Response.INESCAPABLE),
    ((1., 0.), [(21., 20., 1., 0., 1.)], GetAvoidancePoint.Response.INESCAPABLE),
    ((1., 0.), [(30., 20., 1., 0., 1.)], GetAvoidancePoint.Response.NO_THREAT),
    ((0., 0.), [], GetAvoidancePoint.Response.NO_THREAT),
])
def test_stationary_overlap_and_equal_velocity(scene, velocity, ships, expected):
    """Handle zero-speed, overlap and equal-relative-speed states explicitly.

    :param scene: Real-node harness.
    :param velocity: Own-ship body velocity.
    :param ships: Target list.
    :param expected: Expected response status.
    """
    scene.velocity, scene.ships = velocity, ships
    scene.until(lambda: scene.state is not None and scene.state.valid)
    result = scene.avoid()
    assert result.status == expected, result
    assert all(math.isfinite(m.pose.position.x) and math.isfinite(m.pose.position.y)
               for m in scene.cpa_markers if m.action == Marker.ADD)
    print('PASS: special-state response', expected, result.message, flush=True)


@pytest.mark.parametrize('fault', [
    'tracks_stopped', 'old_tracks', 'old_odom', 'bad_frame', 'manager_stopped'])
def test_stale_and_invalid_inputs_fail_closed(scene, fault):
    """Fail closed and clear stale markers after critical input loss.

    :param scene: Real-node harness.
    :param fault: Input failure to inject.
    """
    scene.until(lambda: scene.state is not None and scene.state.valid)
    decision = scene.ready_decision()
    assert decision.status == decision.SUCCESS
    barrier = scene.barrier(decision)
    assert barrier.status == barrier.SUCCESS
    scene.avoidance_markers.clear()
    scene.barrier_markers.clear()
    if fault == 'tracks_stopped':
        scene.enabled = False
    elif fault == 'old_tracks':
        scene.track_age = 5.0
    elif fault == 'old_odom':
        scene.odom_age = 5.0
    elif fault == 'bad_frame':
        scene.frame = 'missing_sensor_frame'
    else:
        scene.stop('ts_state_manager')
    if fault != 'manager_stopped':
        scene.until(lambda: not scene.state.valid)
    scene.until(lambda: any(m.action == Marker.DELETEALL for m in scene.avoidance_markers) and
                any(m.action == Marker.DELETEALL for m in scene.barrier_markers))
    result = scene.avoid()
    assert result.status == result.STALE_STATE
    assert scene.barrier(decision).status == GetBarrierLines.Response.STALE_STATE
    print('PASS: stale-state rejection and marker cleanup:', fault, flush=True)


def test_rotated_velocity_snapshot_and_request_validation(scene):
    """Transform velocities with positions and reject invalid service frames/directions.

    :param scene: Real-node harness.
    """
    scene.frame = 'sensor'
    scene.ships = [(0., -10., 1., 0., 1.)]
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    ship = scene.state.ships[0]
    assert ship.pose.position.x == pytest.approx(30.)
    assert ship.twist.linear.x == pytest.approx(0., abs=1e-8)
    assert ship.twist.linear.y == pytest.approx(1.)
    assert scene.avoid(frame='odom').status == GetAvoidancePoint.Response.INVALID_REQUEST
    assert scene.avoid(direction='starboard').status == GetAvoidancePoint.Response.INVALID_REQUEST
    request = GetAvoidancePoint.Request()
    request.header.frame_id = 'map'
    request.os_pose = pose(23.01, 20).pose
    request.goal = pose(60, 20).pose
    request.avoid_direction = 'right'
    assert scene.call(GetAvoidancePoint, '/get_avoidance_point', request).status == \
        GetAvoidancePoint.Response.INVALID_REQUEST
    print('PASS: rotated frame position AND velocity, invalid request rejection', flush=True)


def test_snapshot_pin_removal_and_expiry(scene):
    """Use the avoidance snapshot even if a newer target pose arrives before barriers.

    :param scene: Real-node harness.
    """
    scene.until(lambda: scene.state is not None and scene.state.valid)
    decision = scene.ready_decision()
    assert decision.status == decision.SUCCESS
    scene.ships = [(32., 20., 0., 0., 1.)]
    scene.until(lambda: scene.state.valid and scene.state.ships[0].pose.position.x == 32.)
    barrier = scene.barrier(decision)
    assert barrier.status == barrier.SUCCESS
    assert barrier.snapshot_id == decision.snapshot_id
    assert barrier.barriers.points[0].x == pytest.approx(30.)
    unknown = GetAvoidancePoint.Response()
    unknown.header = decision.header
    unknown.snapshot_id.uuid[0] = 255
    unknown.primary_target_id = decision.primary_target_id
    assert scene.barrier(unknown).status == GetBarrierLines.Response.SNAPSHOT_MISMATCH
    scene.until(lambda: (scene.node.get_clock().now() -
                         rclpy.time.Time.from_msg(decision.header.stamp)).nanoseconds > 700000000)
    assert scene.barrier(decision).status == GetBarrierLines.Response.STALE_STATE
    decision = scene.avoid()
    scene.ships = []
    scene.until(lambda: scene.state.valid and not scene.state.ships)
    assert scene.avoid().status == GetAvoidancePoint.Response.NO_THREAT
    assert scene.barrier(decision).status == GetBarrierLines.Response.TARGET_NOT_FOUND
    print('PASS: snapshot pin, mismatch, expiry and target removal', flush=True)


@pytest.mark.parametrize('future', [False, True])
def test_out_of_order_and_future_measurements(scene, future):
    """Prevent timestamp rollback and reject future-epoch observations.

    :param scene: Real-node harness.
    :param future: Inject a future stamp rather than an older packet.
    """
    scene.until(lambda: scene.state is not None and scene.state.valid)
    scene.enabled = False
    packet = TrackedShipList()
    offset = -2.0 if future else 0.4
    packet.header.stamp = (scene.node.get_clock().now() -
                           rclpy.duration.Duration(seconds=offset)).to_msg()
    packet.header.frame_id = 'map'
    ship = TrackedShip()
    ship.target_id.uuid[0] = 1
    ship.pose = pose(90, 20).pose
    ship.radius = 1.0
    packet.ships = [ship]
    scene.tracks.publish(packet)
    if future:
        scene.until(lambda: not scene.state.valid)
        assert scene.avoid().status == GetAvoidancePoint.Response.STALE_STATE
    else:
        for _ in range(3):
            old = scene.state.snapshot_id
            scene.until(lambda: scene.state.snapshot_id != old)
            assert scene.state.valid
            assert scene.state.ships[0].pose.position.x == pytest.approx(30.)
    scene.enabled = True
    scene.until(lambda: scene.state.valid and scene.state.ships[0].pose.position.x == 30.)
    assert scene.avoid().status == GetAvoidancePoint.Response.SUCCESS
    print('PASS: timestamp handling and recovery, future=', future, flush=True)


@pytest.mark.parametrize('scene', [False], indirect=True)
def test_no_data_is_not_no_threat(scene):
    """Distinguish an absent producer from a valid empty scene.

    :param scene: Harness without a manager process.
    """
    assert scene.avoid().status == GetAvoidancePoint.Response.NO_DATA


@pytest.mark.parametrize('fault', ['none', 'overlap', 'blocked_leg', 'blocked_tail', 'stale'])
def test_real_planner_failure_policy(scene, fault):
    """Exercise both VO planners against the actual TS services and costmap.

    :param scene: Real-node harness.
    :param fault: Successful or failing navigation scenario.
    """
    if fault == 'overlap':
        scene.ships = [(21., 20., 1., 0., 1.)]
    scene.until(lambda: scene.state is not None and scene.state.valid)
    scene.start_planner(blocked=fault == 'blocked_leg', blocked_tail=fault == 'blocked_tail')
    if fault == 'stale':
        scene.enabled = False
        scene.until(lambda: not scene.state.valid)
    for planner in ('VO', 'Skeleton'):
        result = scene.plan(planner)
        succeeds = fault in ('none', 'blocked_leg')
        expected = GoalStatus.STATUS_SUCCEEDED if succeeds else GoalStatus.STATUS_ABORTED
        assert result.status == expected, (planner, fault, result)
        assert bool(result.result.path.poses) == succeeds
        if fault == 'blocked_leg':
            # The first leg is not rerouted around even a static occupied patch:
            # first-leg static obstacle handling is outside the open-water model.
            assert any(24. <= p.pose.position.x < 26.5 and 18. <= p.pose.position.y < 20.5
                       for p in result.result.path.poses)
        print('REAL planner', planner, fault, 'status=', result.status, flush=True)
    if fault == 'none':
        assert scene.plan('VO', through=True).status == GoalStatus.STATUS_SUCCEEDED
        scene.ships = []
        scene.until(lambda: scene.state.valid and not scene.state.ships)
        assert scene.plan('VO').status == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.parametrize('scene', [{'parameters': {
    'ts_state_manager': {'threat_radius_scale': 3.0}}}], indirect=True)
def test_vo_leg_crosses_current_moving_ship_occupancy(scene):
    """Accept a future-safe straight leg through the moving TS's current costmap disc.

    :param scene: Actual TS services and planner action harness.
    """
    scene.velocity = (3., 0.)
    scene.ships = [(30., 20., 0., 2., 1.)]
    scene.until(lambda: scene.state is not None and scene.state.valid and
                scene.state.ships and scene.state.ships[0].has_threat)
    decision = scene.ready_decision()
    assert decision.status == decision.SUCCESS
    assert decision.safe_heading == pytest.approx(0.0)
    assert decision.point.x > 32.0
    # Relative r=(10,0), v=(-3,2): continuous DCPA=20/sqrt(13),
    # despite the geometric ray passing through the current TS centre.
    dcpa = 20. / math.sqrt(13.)
    assert dcpa > 2.0
    scene.start_planner(static_discs=[(30., 20., 1.)])
    for planner in ('VO', 'Skeleton'):
        result = scene.plan(planner)
        assert result.status == GoalStatus.STATUS_SUCCEEDED, (planner, result)
        assert min(math.hypot(p.pose.position.x-30., p.pose.position.y-20.)
                   for p in result.result.path.poses) < 0.5
        print('PASS:', planner, 'crosses current TS occupancy; predicted DCPA=', dcpa,
              flush=True)


def test_right_crossing_vo_leg_can_exit_search_barrier(scene):
    """A certified VO leg may cross the artificial U wall used by the later search.

    :param scene: Real-node harness with an empty physical costmap.
    """
    scene.velocity = (3., 0.)
    scene.ships = [(50., 0., 0., 2., 1.)]
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    decision = scene.ready_decision()
    assert decision.status == decision.SUCCESS
    barrier = scene.barrier(decision)
    assert barrier.status == barrier.SUCCESS
    start, end = (20., 20.), (decision.point.x, decision.point.y)

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

    intersects = False
    for i in range(0, len(barrier.barriers.points), 2):
        a, b = barrier.barriers.points[i:i+2]
        a, b = (a.x, a.y), (b.x, b.y)
        intersects |= (cross(start, end, a)*cross(start, end, b) < 0 and
                       cross(a, b, start)*cross(a, b, end) < 0)
    assert intersects, 'Scenario must reproduce an artificial first-leg intersection'
    # Verify continuous closest approach of the selected ray independently.
    ux = -3 * math.cos(decision.safe_heading)
    uy = 2 - 3 * math.sin(decision.safe_heading)
    tcpa = -(30*ux - 20*uy)/(ux*ux + uy*uy)
    dcpa = math.hypot(30 + ux*tcpa, -20 + uy*tcpa)
    assert dcpa > 2.0
    print('RIGHT_CROSSING_CERT:', 'heading=', decision.safe_heading,
          'dcpa=', dcpa, 'required_clearance=2.0', 'artificial_U_intersection=', intersects,
          flush=True)
    scene.start_planner()
    for planner in ('VO', 'Skeleton'):
        result = scene.plan(planner)
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.path.poses
    print('PASS: right crossing, continuous clearance=', dcpa,
          'U intersection confined to certified VO leg; both planners succeed', flush=True)


INFLATION_SETTINGS = {'parameters': {
    'ts_state_manager': {'os_radius': 5.0, 'threat_radius_scale': 3.0},
    'avoidance_point_node': {'avoidance_radius_scale': 2.0,
                             'point_extension_distance': 20.0}}}


def set_avoidance(scene, **values):
    """Set a batch of avoidance parameters through the actual ROS service.

    :param scene: Real-node harness.
    :param values: Parameter names and new values.
    """
    request = SetParametersAtomically.Request(parameters=[
        Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
    result = scene.call(SetParametersAtomically, '/avoidance_point_node/set_parameters_atomically',
                        request)
    assert result.result.successful, result


def decision_measurements(scene, result):
    """Independently compute clearance and point range using the exact response snapshot.

    :param scene: Real-node harness.
    :param result: Successful avoidance response.
    :return: Predicted DCPA and point range extension in metres.
    """
    key = bytes(result.snapshot_id.uuid)
    scene.until(lambda: key in scene.states)
    state = scene.states[key]
    ship = next(s for s in state.ships if s.target_id == result.primary_target_id)
    rx, ry = ship.pose.position.x - 20, ship.pose.position.y - 20
    speed = math.hypot(state.os_twist.linear.x, state.os_twist.linear.y)
    ux = ship.twist.linear.x - speed * math.cos(result.safe_heading)
    uy = ship.twist.linear.y - speed * math.sin(result.safe_heading)
    speed_sq = ux * ux + uy * uy
    tcpa = max(0., -(rx * ux + ry * uy) / speed_sq) if speed_sq > 1e-12 else 0.
    dcpa = math.hypot(rx + ux * tcpa, ry + uy * tcpa)
    extension = math.hypot(result.point.x - 20, result.point.y - 20) - math.hypot(rx, ry)
    return dcpa, extension


@pytest.mark.parametrize('scene', [INFLATION_SETTINGS], indirect=True)
def test_recorded_13m_goal_requires_inflated_20m_clearance(scene):
    """Replay the recorded straight-goal counterexample with the real avoidance node.

    :param scene: Real-node harness with 5m radii and manager inflation 3.
    """
    # Recorded 490.272s encounter, translated so the request starts at (20,20).
    ox, oy = 995.9622024788268, -107.33591186500246
    scene.velocity = (1.4959069311671953, 0.16709165170345544)
    scene.ships = [(20 + 990.8477239213775 - ox, 20 - 70.70090404529354 - oy,
                    0.20999429877731252, -2.992641374184522, 5.)]
    goal = (20 + 194.46487426757812 - ox, 20 - 56.257049560546875 - oy)
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    set_avoidance(scene, avoidance_radius_scale=1.0)
    uninflated = scene.avoid(goal=goal)
    assert uninflated.status == uninflated.SUCCESS
    goal_angle = math.atan2(goal[1] - 20, goal[0] - 20) % (2 * math.pi)
    assert uninflated.safe_heading == pytest.approx(goal_angle)
    old_dcpa, old_extension = decision_measurements(scene, uninflated)
    assert 13.0 < old_dcpa < 13.7
    assert old_extension == pytest.approx(20.)

    set_avoidance(scene, avoidance_radius_scale=2.0)
    inflated = scene.avoid(goal=goal)
    assert inflated.status == inflated.SUCCESS
    assert inflated.safe_heading != pytest.approx(goal_angle)
    new_dcpa, extension = decision_measurements(scene, inflated)
    assert new_dcpa > 20.0
    assert extension == pytest.approx(20.0)

    set_avoidance(scene, point_extension_distance=25.0)
    extended = scene.avoid(goal=goal)
    assert extended.status == extended.SUCCESS
    dcpa, extension = decision_measurements(scene, extended)
    assert dcpa > 20.0
    assert extension == pytest.approx(25.0)
    print('INFLATION REPLAY: factor=1 goal DCPA=', old_dcpa,
          '; factor=2 selected DCPA=', new_dcpa,
          '; inflation/endpoint controls independently verified', flush=True)


@pytest.mark.parametrize('scene', [INFLATION_SETTINGS], indirect=True)
@pytest.mark.parametrize('distance,target_vx,expected', [
    (9., 0., GetAvoidancePoint.Response.INESCAPABLE),
    (10., 0., GetAvoidancePoint.Response.INESCAPABLE),
    (15., 1., GetAvoidancePoint.Response.SUCCESS),
    (15., 2., GetAvoidancePoint.Response.SUCCESS),
    (20., 0., GetAvoidancePoint.Response.SUCCESS),
    (20.1, 0., GetAvoidancePoint.Response.SUCCESS),
])
def test_inflated_domain_falls_back_but_physical_overlap_fails(
        scene, distance, target_vx, expected):
    """Retry physical radii inside the inflated domain; retain physical rejection.

    :param scene: Real-node harness with effective 20m avoidance radius.
    :param distance: Initial OS-TS separation.
    :param target_vx: TS velocity, including equal-velocity and receding cases.
    :param expected: Expected response status.
    """
    scene.ships = [(20 + distance, 20., target_vx, 0., 5.)]
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    assert scene.state.ships[0].has_threat
    if target_vx == 1.0:
        assert math.isinf(scene.state.ships[0].tcpa)
    result = scene.avoid()
    assert result.status == expected, result
    if expected == result.INESCAPABLE:
        scene.start_planner()
        for planner in ('VO', 'Skeleton'):
            assert scene.plan(planner).status == GoalStatus.STATUS_ABORTED
    else:
        fallback = distance <= 20.
        assert ('Physical-radius fallback' in result.message) == fallback
        assert decision_measurements(scene, result)[0] > (10. if fallback else 20.)
        if fallback:
            assert 'retrying at scale=1.000' in (
                scene.directory / 'avoidance_point_node.log').read_text()
    print('INFLATED DOMAIN:', distance, target_vx, 'status=', result.status, flush=True)


@pytest.mark.parametrize('scene', [INFLATION_SETTINGS], indirect=True)
def test_physical_fallback_real_planners(scene):
    """Carry a fallback response through snapshot-pinned barriers and both planners.

    :param scene: Real-node harness with physical radius 10m and preferred radius 20m.
    """
    scene.ships = [(35., 20., 2., 0., 5.)]
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    decision = scene.ready_decision()
    assert 'Physical-radius fallback' in decision.message
    assert decision_measurements(scene, decision)[0] > 10.
    scene.start_planner()
    for planner in ('VO', 'Skeleton'):
        result = scene.plan(planner)
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.path.poses
    print('PASS: fallback snapshot/barrier and both real planning actions', flush=True)


@pytest.mark.parametrize('scene', [INFLATION_SETTINGS], indirect=True)
def test_inflated_right_crossing_real_planners(scene):
    """Run both actual planners with radius-inflated avoidance services.

    :param scene: Real-node harness using manager=3 and avoidance=2.
    """
    scene.velocity = (3., 0.)
    scene.ships = [(60., 5., 0., 3., 5.)]
    scene.until(lambda: scene.state is not None and scene.state.valid and scene.state.ships)
    decision = scene.ready_decision()
    dcpa, extension = decision_measurements(scene, decision)
    assert dcpa > 20.0 and extension == pytest.approx(20.0)
    scene.start_planner()
    for planner in ('VO', 'Skeleton'):
        result = scene.plan(planner)
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.path.poses
    print('REAL INFLATED CROSSING: DCPA=', dcpa, '>20m; both planners succeeded', flush=True)


@pytest.mark.parametrize('scene', [{'parameters': {
    'ts_state_manager': {'robot_base_frame': 'delayed_base'}}}], indirect=True)
def test_os_and_ts_positions_share_calculation_time(scene):
    """Extrapolate a delayed dynamic OS pose to the same time as TS positions.

    :param scene: Harness using a delayed dynamic base transform.
    """
    scene.velocity = (3.0, 0.0)
    broadcaster = TransformBroadcaster(scene.node)

    def publish_tf():
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'delayed_base'
        transform.header.stamp = (scene.node.get_clock().now() -
                                  rclpy.duration.Duration(seconds=0.3)).to_msg()
        transform.transform.translation.x = 20.0
        transform.transform.translation.y = 20.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

    scene.node.create_timer(0.02, publish_tf)
    scene.until(lambda: scene.state is not None and scene.state.valid)
    assert 20.89 <= scene.state.os_pose.position.x <= 21.3
    assert scene.state.os_pose.position.y == pytest.approx(20.0)
    print('COMMON TIME: delayed TF x=20, extrapolated snapshot x=',
          scene.state.os_pose.position.x, flush=True)
