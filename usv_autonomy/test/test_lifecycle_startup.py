"""A completed transition with a delayed/lost reply must not hang bringup."""
import signal
import subprocess
from time import monotonic
import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State, Transition
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future


def test_manager_confirms_actual_state_after_transition_reply_timeout(tmp_path):
    rclpy.init(args=['--ros-args', '-r', '__ns:=/late_lifecycle_reply'])
    node = Node('backend_test')
    group = ReentrantCallbackGroup()
    current = [State.PRIMARY_STATE_UNCONFIGURED]
    configured_at, activated_at = [], []
    configure_reply_sent = [False]
    async def change(request, response):
        if request.transition.id == Transition.TRANSITION_CONFIGURE:
            current[0] = State.PRIMARY_STATE_INACTIVE
            configured_at.append(monotonic())
            # Transition already completed, but its response is delayed.
            future = Future()
            timer = node.create_timer(3., lambda: future.set_result(True) if not future.done() else None,
                                      callback_group=group)
            try:
                await future
            finally:
                node.destroy_timer(timer)
            configure_reply_sent[0] = True
        elif request.transition.id == Transition.TRANSITION_ACTIVATE:
            current[0] = State.PRIMARY_STATE_ACTIVE
            activated_at.append(monotonic())
        elif request.transition.id == Transition.TRANSITION_DEACTIVATE:
            current[0] = State.PRIMARY_STATE_INACTIVE
        elif request.transition.id == Transition.TRANSITION_CLEANUP:
            current[0] = State.PRIMARY_STATE_UNCONFIGURED
        response.success = True
        return response
    def get_state(request, response):
        response.current_state.id = current[0]
        return response
    node.create_service(ChangeState, 'backend/change_state', change, callback_group=group)
    node.create_service(GetState, 'backend/get_state', get_state, callback_group=group)
    log = (tmp_path / 'manager.log').open('w+')
    binary = get_package_prefix('nav2_lifecycle_manager') + '/lib/nav2_lifecycle_manager/lifecycle_manager'
    process = subprocess.Popen([binary, '--ros-args', '-r', '__ns:=/late_lifecycle_reply',
        '-p', 'node_names:=[backend]', '-p', 'autostart:=true', '-p', 'bond_timeout:=0.0',
        '-p', 'transition_timeout:=1'], stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = monotonic() + 8.
        while not activated_at and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        log.flush(); log.seek(0)
        assert activated_at, log.read()
        assert len(configured_at) == 1  # No blind duplicate configure request.
        assert activated_at[0] - configured_at[0] < 2.5
        assert current[0] == State.PRIMARY_STATE_ACTIVE
    finally:
        # Do not destroy a service while its deliberately delayed coroutine
        # still owns the request handle (rclpy destruction waits for that handle).
        finish = monotonic() + 4.
        while configured_at and not configure_reply_sent[0] and monotonic() < finish:
            rclpy.spin_once(node, timeout_sec=.02)
        process.send_signal(signal.SIGINT)
        end = monotonic() + 8.
        while process.poll() is None and monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if process.poll() is None:
            process.kill(); process.wait()
        node.destroy_node(); rclpy.shutdown(); log.close()
