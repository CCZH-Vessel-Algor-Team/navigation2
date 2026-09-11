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

from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_colregs_msgs.srv import GetAvoidancePoint, GetBarrierLines
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from tf2_ros import StaticTransformBroadcaster


PARAMS = """
/anchor_test/planner_server:
  ros__parameters:
    use_sim_time: false
    planner_plugins: [VORRTStar]
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
    log_path = tmp_path / 'planner.log'
    print(f'Planner log: {log_path}', flush=True)

    def avoidance(request, response):
        calls['avoidance'].append(request)
        response.has_feasible_angle = True
        response.point = Point(x=20.0, y=6.0)
        return response

    def barrier(request, response):
        calls['barrier'].append(request)
        response.barriers.points = [Point(x=90.0, y=80.0), Point(x=90.0, y=90.0)]
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

    def plan(action_type, action_name, goals, start=None, expected_calls=0):
        before = {key: len(value) for key, value in calls.items()}
        client = ActionClient(node, action_type, '/anchor_test/' + action_name)
        clients.append(client)
        assert client.wait_for_server(timeout_sec=10.0)
        request = action_type.Goal()
        request.planner_id = 'VORRTStar'
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
        print(f'{action_name}: SUCCEEDED, poses={len(path.poses)}, '
              f'calls per service={expected_calls}', flush=True)

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
