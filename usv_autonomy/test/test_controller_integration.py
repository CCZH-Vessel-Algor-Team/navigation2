"""Exercise the real controller producer and its loop-exit heartbeat."""
import signal
import subprocess
from time import monotonic
import yaml
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from usv_interfaces.msg import ExecutionEvidence, ManagedMotion


def test_real_controller_binds_goal_context_and_reports_idle(tmp_path):
    rclpy.init(args=['--ros-args', '-r', '__ns:=/producer_v2'])
    peer = Node('controller_test_peer')
    broadcaster = TransformBroadcaster(peer)
    odom_pub = peer.create_publisher(Odometry, 'odom', 1)
    motions, evidence = [], []
    peer.create_subscription(ManagedMotion, 'autonomy/motion_raw', motions.append, 10)
    peer.create_subscription(ExecutionEvidence, 'autonomy/producer', evidence.append, 10)
    def publish():
        stamp = peer.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id, t.header.stamp = 'map', 'boat/base_link', stamp
        t.transform.rotation.w = 1.
        broadcaster.sendTransform(t)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'map'
        odom.child_frame_id = 'boat/base_link'
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
    peer.create_timer(.03, publish)
    params = {
        'controller_server': {'ros__parameters': {
            'require_managed_execution': True, 'controller_frequency': 20.,
            'controller_plugins': ['FollowPath'], 'goal_checker_plugins': ['goal_checker'],
            'FollowPath': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                           'desired_linear_vel': .5},
            'goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker',
                             'xy_goal_tolerance': .5, 'yaw_goal_tolerance': 6.28}}},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'global_frame': 'map', 'robot_base_frame': 'boat/base_link',
            'rolling_window': True, 'width': 30, 'height': 30, 'resolution': .5,
            'robot_radius': .2, 'update_frequency': 20.,
            'plugins': ['inflation'], 'inflation': {'plugin': 'nav2_costmap_2d::InflationLayer'}}}},
    }
    path = tmp_path / 'controller.yaml'
    path.write_text(yaml.safe_dump({'producer_v2': params}))
    log = (tmp_path / 'controller.log').open('w+')
    process = subprocess.Popen([
        get_package_prefix('nav2_controller') + '/lib/nav2_controller/controller_server',
        '--ros-args', '-r', '__ns:=/producer_v2', '--params-file', str(path)],
        stdout=log, stderr=subprocess.STDOUT)
    def until(predicate, timeout=10):
        end = monotonic() + timeout
        while monotonic() < end:
            rclpy.spin_once(peer, timeout_sec=.01)
            if predicate():
                return
        log.flush()
        log.seek(0)
        raise AssertionError(log.read())
    try:
        lifecycle = peer.create_client(ChangeState, 'controller_server/change_state')
        until(lifecycle.service_is_ready)
        for transition in (1, 3):
            req = ChangeState.Request()
            req.transition.id = transition
            future = lifecycle.call_async(req)
            until(future.done)
            assert future.result().success
        until(lambda: evidence and not evidence[-1].active)
        client = ActionClient(peer, FollowPath, 'follow_path')
        until(client.server_is_ready)
        request = FollowPath.Goal()
        request.controller_id, request.goal_checker_id = 'FollowPath', 'goal_checker'
        request.usv_execution_id, request.usv_attempt_id, request.usv_boot_id, request.usv_epoch = 'execution', 'attempt', 'coordinator', 7
        request.path.header.frame_id = 'map'
        for x in (0., 2., 5.):
            pose = PoseStamped()
            pose.header.frame_id, pose.pose.orientation.w, pose.pose.position.x = 'map', 1., x
            request.path.poses.append(pose)
        future = client.send_goal_async(request)
        until(future.done)
        handle = future.result()
        assert handle.accepted
        until(lambda: motions and evidence[-1].active)
        assert (motions[-1].boot_id, motions[-1].execution_id, motions[-1].attempt_id, motions[-1].epoch) == (
            'coordinator', 'execution', 'attempt', 7)
        assert motions[-1].sequence > 0
        # A conflicting execution must not preempt a still-active controller.
        request.usv_execution_id = 'conflict'
        pending = client.send_goal_async(request)
        until(pending.done)
        pending_result = pending.result().get_result_async()
        until(pending_result.done)
        assert pending_result.result().status != 4
        assert all(m.execution_id == 'execution' for m in motions)
        canceled = handle.cancel_goal_async()
        until(canceled.done)
        result = handle.get_result_async()
        until(result.done)
        until(lambda: evidence and not evidence[-1].active)
        assert evidence[-1].execution_id == 'execution' and evidence[-1].epoch == 7
        assert result.result().status != 4
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        peer.destroy_node()
        rclpy.shutdown()
        log.close()
