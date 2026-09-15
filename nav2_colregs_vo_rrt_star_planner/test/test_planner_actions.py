"""Exercise the loaded planner plugin, live TF, services and both planning actions.

Run in a sourced ROS environment with pytest. The test uses an isolated domain
(NTP_TEST_ROS_DOMAIN_ID, default 91) and owns only its planner subprocess.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import yaml

from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_colregs_msgs.srv import GetAvoidancePoint, GetBarrierLines
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParametersAtomically
from tf2_ros import StaticTransformBroadcaster


PARAMS = """
/anchor_test/planner_server:
  ros__parameters:
    use_sim_time: false
    planner_plugins: [VORRTStar, Plain, Skeleton, VOSkeleton]
    Plain:
      plugin: nav2_rrt_star_planner::RRTStarPlanner
      max_iterations: 100
      max_optimize_iters: 0
    Skeleton:
      plugin: nav2_skeleton_planner/SkeletonRRTPlanner
      max_work: 3000000000
    VOSkeleton:
      plugin: nav2_colregs_vo_skeleton_planner/VOSkeletonPlanner
      max_work: 3000000000
    VORRTStar:
      plugin: nav2_colregs_vo_rrt_star_planner::VORRTStarPlanner
      step_size: 4.0
      max_iterations: 100
      max_optimize_iters: 0
      goal_bias: 0.1
      goal_threshold: 2.0
      safety_dist: 0.5
      eta: 50.0
      colregs_anchor_max_dist: 3.0
/anchor_test/global_costmap/global_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    update_frequency: 10.0
    publish_frequency: 1.0
    width: 100
    height: 100
    resolution: 1.0
    origin_x: 0.0
    origin_y: 0.0
    rolling_window: false
    track_unknown_space: false
    robot_radius: 0.5
    plugins: [obstacle_layer]
    obstacle_layer:
      plugin: nav2_costmap_2d::ObstacleLayer
      observation_sources: ''
"""


def pose(x, y):
    """Create a map-frame pose.

    :param x: Map x coordinate in metres.
    :param y: Map y coordinate in metres.
    :return: A pose with identity orientation.
    """
    result = PoseStamped()
    result.header.frame_id = 'map'
    result.pose.position.x = float(x)
    result.pose.position.y = float(y)
    result.pose.orientation.w = 1.0
    return result


def test_planner_actions(tmp_path, monkeypatch):
    """Verify lifecycle retention and actual near/far service gating.

    :param tmp_path: Pytest directory for parameters and retained planner logs.
    :param monkeypatch: Pytest environment override fixture.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('NTP_TEST_ROS_DOMAIN_ID', '91'))
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    rclpy.init()
    node = rclpy.create_node('anchor_action_test')
    proc = None
    clients = []
    calls = {'avoidance': [], 'barrier': []}
    behavior = {}
    log_path = tmp_path / 'planner.log'
    print(f'Planner log: {log_path}', flush=True)

    def avoidance(request, response):
        calls['avoidance'].append(request)
        time.sleep(behavior.get('avoidance_delay', 0.0))
        response.status = behavior.get('avoidance_status', GetAvoidancePoint.Response.SUCCESS)
        response.message = f'Test decision status {response.status}'
        response.header = request.header
        response.snapshot_id.uuid[0] = 1
        response.has_feasible_angle = response.status == GetAvoidancePoint.Response.SUCCESS
        response.point = Point(x=20.0, y=6.0)
        if behavior.get('bad_point'):
            response.point.x = float('nan')
        if behavior.get('bad_frame'):
            response.header.frame_id = 'wrong_frame'
        return response

    def barrier(request, response):
        calls['barrier'].append(request)
        time.sleep(behavior.get('barrier_delay', 0.0))
        response.status = behavior.get('barrier_status', GetBarrierLines.Response.SUCCESS)
        response.header = request.header
        response.snapshot_id = request.snapshot_id
        response.barriers.points = [Point(x=90.0, y=80.0), Point(x=90.0, y=90.0),
                                    Point(x=90.0, y=90.0), Point(x=95.0, y=90.0),
                                    Point(x=95.0, y=90.0), Point(x=95.0, y=80.0)]
        if behavior.get('empty_barrier'):
            response.barriers.points = []
        if behavior.get('crossing_barrier'):
            response.barriers.points = [Point(x=15., y=0.), Point(x=15., y=20.),
                                        Point(x=15., y=20.), Point(x=16., y=20.),
                                        Point(x=16., y=20.), Point(x=16., y=0.)]
        if behavior.get('enclosed_avoidance'):
            response.barriers.points = [Point(x=18., y=4.), Point(x=22., y=4.),
                                        Point(x=22., y=4.), Point(x=20., y=8.),
                                        Point(x=20., y=8.), Point(x=18., y=4.)]
        if behavior.get('mismatched_snapshot'):
            response.snapshot_id.uuid[0] = 2
        return response

    def wait(future, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            assert proc.poll() is None, f'planner_server exited: {proc.returncode}'
            rclpy.spin_once(node, timeout_sec=0.05)
        assert future.done(), 'ROS request timed out'
        result = future.result()
        assert result is not None
        return result

    def transition(transition_id, expected_state):
        change = node.create_client(ChangeState, '/anchor_test/planner_server/change_state')
        state = node.create_client(GetState, '/anchor_test/planner_server/get_state')
        try:
            assert change.wait_for_service(timeout_sec=10.0)
            request = ChangeState.Request()
            request.transition.id = transition_id
            assert wait(change.call_async(request)).success
            assert state.wait_for_service(timeout_sec=5.0)
            assert wait(state.call_async(GetState.Request())).current_state.id == expected_state
        finally:
            node.destroy_client(change)
            node.destroy_client(state)

    def plan(action_type, action_name, goals, start=None, expected_calls=0, planner='VORRTStar'):
        before = {key: len(value) for key, value in calls.items()}
        client = ActionClient(node, action_type, '/anchor_test/' + action_name)
        clients.append(client)
        assert client.wait_for_server(timeout_sec=10.0)
        request = action_type.Goal()
        request.planner_id = planner
        if action_type is ComputePathToPose:
            request.goal = goals[0]
        else:
            request.goals = goals
        if start is not None:
            request.use_start = True
            request.start = start
        handle = wait(client.send_goal_async(request))
        assert handle.accepted
        result = wait(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        path = result.result.path
        assert path.header.frame_id == 'map'
        assert len(path.poses) >= 2
        for key, value in calls.items():
            assert len(value) - before[key] == expected_calls, (key, before, calls)

        def contains(target):
            return any(math.hypot(p.pose.position.x - target.pose.position.x,
                                  p.pose.position.y - target.pose.position.y) < 1e-6
                       for p in path.poses)

        actual_start = start if start is not None else pose(10, 10)
        assert contains(actual_start)
        assert all(contains(goal) for goal in goals)
        assert contains(pose(20, 6)) == bool(expected_calls)
        if expected_calls:
            assert calls['avoidance'][-1].os_pose.position.x == actual_start.pose.position.x
            assert calls['barrier'][-1].os_pose.position.x == actual_start.pose.position.x
        assert proc.poll() is None
        print(f'{planner} {action_name}: SUCCEEDED, poses={len(path.poses)}, '
              f'calls per service={expected_calls}', flush=True)

    def parameter_request(service, suffix, request):
        client = node.create_client(service, '/anchor_test/planner_server/' + suffix)
        try:
            assert client.wait_for_service(timeout_sec=5.0)
            return wait(client.call_async(request))
        finally:
            node.destroy_client(client)

    def set_values(values, accepted=True):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(key, value=value).to_parameter_msg()
                              for key, value in values.items()]
        result = parameter_request(SetParametersAtomically, 'set_parameters_atomically', request)
        assert result.result.successful == accepted, result
        if not accepted:
            assert result.result.reason
        print(f'PARAM {values}: {result.result}', flush=True)

    def get_values(names):
        request = GetParameters.Request(names=names)
        return parameter_request(GetParameters, 'get_parameters', request).values

    def assert_applied(planner, informed, offset):
        lines = [line for line in log_path.read_text()[offset:].splitlines()
                 if f'Applied {planner} parameters:' in line]
        assert lines and f'informed={str(informed).lower()}' in lines[-1], lines
        print(lines[-1], flush=True)

    def decision_plan(planner, expected):
        client = ActionClient(node, ComputePathToPose, '/anchor_test/compute_path_to_pose')
        clients.append(client)
        assert client.wait_for_server(timeout_sec=5)
        request = ComputePathToPose.Goal()
        request.planner_id = planner
        request.goal = pose(30, 10)
        handle = wait(client.send_goal_async(request))
        assert handle.accepted
        result = wait(handle.get_result_async())
        assert result.status == expected, (planner, behavior, result)
        assert bool(result.result.path.poses) == (expected == GoalStatus.STATUS_SUCCEEDED)
        assert proc.poll() is None
        print('DECISION POLICY:', planner, behavior, 'status=', result.status, flush=True)

    try:
        node.create_service(GetAvoidancePoint, '/get_avoidance_point', avoidance)
        node.create_service(GetBarrierLines, '/get_barrier_lines', barrier)
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x = 10.0
        transform.transform.translation.y = 10.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        params = tmp_path / 'params.yaml'
        params.write_text(PARAMS)
        executable = Path(get_package_prefix('nav2_planner')) / 'lib/nav2_planner/planner_server'
        with log_path.open('w') as log:
            proc = subprocess.Popen(
                [str(executable), '--ros-args', '-r', '__ns:=/anchor_test',
                 '--params-file', str(params)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        transition(Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE)
        transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE)
        plan(ComputePathToPose, 'compute_path_to_pose', [pose(30, 10)], expected_calls=1)
        plan(ComputePathToPose, 'compute_path_to_pose', [pose(70, 10)], start=pose(50, 10))
        plan(ComputePathToPose, 'compute_path_to_pose', [pose(30, 10)],
             start=pose(13, 10), expected_calls=1)
        plan(ComputePathToPose, 'compute_path_to_pose', [pose(30, 10)], start=pose(13.01, 10))
        plan(ComputePathThroughPoses, 'compute_path_through_poses',
             [pose(30, 10), pose(50, 10), pose(70, 10)], expected_calls=1)

        # Check the core configuration actually used by planning, not just parameter readback.
        for planner in ('VORRTStar', 'Plain'):
            offset = len(log_path.read_text())
            set_values({planner + '.use_informed_sampling': False})
            plan(ComputePathToPose, 'compute_path_to_pose', [pose(70, 10)],
                 start=pose(50, 10), planner=planner)
            assert_applied(planner, False, offset)
            for key, value in [('step_size', 3.0), ('max_iterations', 110),
                               ('max_optimize_iters', 10), ('goal_bias', 0.2),
                               ('goal_threshold', 3.0), ('safety_dist', 0.75),
                               ('cost_weight', 0.5), ('eta', 40.0), ('prune_path', False)]:
                offset = len(log_path.read_text())
                set_values({planner + '.' + key: value})
                plan(ComputePathToPose, 'compute_path_to_pose', [pose(70, 10)],
                     start=pose(50, 10), planner=planner)
                assert_applied(planner, False, offset)
            for key, value in [('step_size', 0.0), ('goal_bias', 1.1),
                               ('safety_dist', -1.0), ('cost_weight', float('nan')),
                               ('eta', float('inf')), ('max_iterations', 0),
                               ('max_optimize_iters', -1), ('max_iterations', 2147483647),
                               ('step_size', 'bad'), ('tolerance', 2.0)]:
                set_values({planner + '.' + key: value}, accepted=False)
            offset = len(log_path.read_text())
            set_values({planner + '.use_informed_sampling': True})
            plan(ComputePathToPose, 'compute_path_to_pose', [pose(70, 10)],
                 start=pose(50, 10), planner=planner)
            assert_applied(planner, True, offset)

        # Another plugin rejecting a batch must not partially apply the VO values.
        before = get_values(['VORRTStar.step_size', 'Plain.cost_weight'])
        set_values({'VORRTStar.step_size': 7.0, 'Plain.cost_weight': -1.0}, accepted=False)
        assert get_values(['VORRTStar.step_size', 'Plain.cost_weight']) == before
        set_values({'VORRTStar.colregs_anchor_max_dist': 10.0}, accepted=False)
        for planner in ('Skeleton', 'VOSkeleton'):
            keys = ['step_size', 'goal_bias', 'eta', 'node_limit', 'path_limit', 'near_limit',
                    'connector_limit', 'recovery_near_limit', 'global_iterations',
                    'local_iterations', 'refine_iterations', 'max_work', 'time_limit',
                    'goal_tolerance', 'allow_recovery', 'allow_skip', 'reuse_iterations',
                    'switch_margin', 'prune_period', 'safety_dist', 'cost_weight']
            request = DescribeParameters.Request(names=[planner + '.' + key for key in keys])
            descriptors = parameter_request(DescribeParameters, 'describe_parameters', request)
            assert all(d.read_only and d.description for d in descriptors.descriptors)
            assert get_values([planner + '.max_work'])[0].integer_value == 3000000000
            set_values({planner + '.step_size': 5.0}, accepted=False)
            set_values({planner + '.prune_period': 20}, accepted=False)
            plan(ComputePathToPose, 'compute_path_to_pose', [pose(30, 10)], planner=planner,
                 expected_calls=1 if planner == 'VOSkeleton' else 0)

        for planner in ('VORRTStar', 'VOSkeleton'):
            for status in (GetAvoidancePoint.Response.NO_DATA,
                           GetAvoidancePoint.Response.INESCAPABLE,
                           GetAvoidancePoint.Response.STALE_STATE,
                           GetAvoidancePoint.Response.INVALID_REQUEST):
                behavior['avoidance_status'] = status
                before = len(calls['barrier'])
                decision_plan(planner, GoalStatus.STATUS_ABORTED)
                assert len(calls['barrier']) == before
            behavior['avoidance_status'] = GetAvoidancePoint.Response.NO_THREAT
            before = len(calls['barrier'])
            decision_plan(planner, GoalStatus.STATUS_SUCCEEDED)
            assert len(calls['barrier']) == before
            behavior.clear()
            for key, value in [('barrier_status', GetBarrierLines.Response.STALE_STATE),
                               ('barrier_status', GetBarrierLines.Response.SNAPSHOT_MISMATCH),
                               ('empty_barrier', True), ('mismatched_snapshot', True),
                               ('enclosed_avoidance', True),
                               ('bad_point', True), ('bad_frame', True),
                               ('avoidance_delay', 1.2), ('barrier_delay', 1.2)]:
                behavior[key] = value
                decision_plan(planner, GoalStatus.STATUS_ABORTED)
                behavior.clear()
            behavior['crossing_barrier'] = True
            decision_plan(planner, GoalStatus.STATUS_SUCCEEDED)
            behavior.clear()
            decision_plan(planner, GoalStatus.STATUS_SUCCEEDED)

        transition(Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE)
        set_values({'VORRTStar.max_iterations': -1}, accepted=False)
        set_values({'VORRTStar.max_iterations': 120, 'VORRTStar.use_informed_sampling': False})
        transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE)
        offset = len(log_path.read_text())
        plan(ComputePathToPose, 'compute_path_to_pose', [pose(30, 10)], expected_calls=1)
        assert_applied('VORRTStar', False, offset)
        transition(Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE)
        transition(Transition.TRANSITION_CLEANUP, State.PRIMARY_STATE_UNCONFIGURED)
        transition(Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE)
        transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE)
        plan(ComputePathToPose, 'compute_path_to_pose', [pose(30, 10)], expected_calls=1)
        transition(Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE)
        transition(Transition.TRANSITION_CLEANUP, State.PRIMARY_STATE_UNCONFIGURED)
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5.0)
        for client in clients:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        if log_path.exists():
            print(log_path.read_text(), flush=True)


@pytest.mark.parametrize('planner,key,value', [
    ('VORRTStar', 'safety_dist', -1.0),
    ('VORRTStar', 'max_iterations', 2147483648),
    ('Plain', 'goal_bias', 1.1),
    ('Plain', 'eta', 0.0),
    ('Skeleton', 'node_limit', 2147483648),
    ('Skeleton', 'max_work', 0),
    ('VOSkeleton', 'cost_weight', -1.0),
    ('VOSkeleton', 'time_limit', float('nan')),
])
def test_invalid_planner_startup(tmp_path, monkeypatch, planner, key, value):
    """Reject invalid startup values through the real planner lifecycle service.

    :param tmp_path: Evidence directory.
    :param monkeypatch: Environment fixture.
    :param planner: Plugin ID to configure.
    :param key: Invalid parameter name.
    :param value: Invalid value.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('NTP_TEST_ROS_DOMAIN_ID', '91'))
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    data = yaml.safe_load(PARAMS)
    settings = data['/anchor_test/planner_server']['ros__parameters']
    settings['planner_plugins'] = [planner]
    settings[planner][key] = value
    params = tmp_path / 'invalid.yaml'
    params.write_text(yaml.safe_dump(data))
    executable = Path(get_package_prefix('nav2_planner')) / 'lib/nav2_planner/planner_server'
    log_path = tmp_path / 'invalid_planner.log'
    rclpy.init()
    node = rclpy.create_node('invalid_planner_parameters')
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/anchor_test',
                                 '--params-file', str(params)],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        client = node.create_client(ChangeState, '/anchor_test/planner_server/change_state')
        assert client.wait_for_service(timeout_sec=5.0)
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.done() and future.result() is not None
        assert not future.result().success
        print(f'Startup rejected: {planner}.{key}={value}', flush=True)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        print(log_path.read_text(), flush=True)
