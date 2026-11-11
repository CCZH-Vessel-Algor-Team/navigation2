"""Observe accepted Nav2 goals independently of a halted BT leaf's executor.

GetResult is the action protocol's public result service. A terminal response
retires only the registered identity; it never substitutes for producer-idle
or output-revocation evidence.
"""
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses, FollowPath


class ChildResults:
    ACTIONS = {'compute_path_to_pose': ComputePathToPose,
               'compute_path_through_poses': ComputePathThroughPoses,
               'follow_path': FollowPath}

    def __init__(self, node, coordinator):
        self.node, self.core = node, coordinator
        self.clients = {name: node.create_client(action.Impl.GetResultService,
            name + '/_action/get_result') for name, action in self.ACTIONS.items()}
        self.cancel_clients = {name: node.create_client(CancelGoal,
            name + '/_action/cancel_goal') for name in self.ACTIONS}
        self.pending = {}

    def watch(self, request):
        key = request.key
        if key in self.pending or key not in self.core.records:
            return
        action = key.partition(':')[0]
        text = request.goal_id.replace('-', '')
        try:
            if action not in self.ACTIONS or len(text) != 32:
                raise ValueError('expected a canonical 16-byte action UUID')
            uuid = bytes.fromhex(text)
        except ValueError as exc:
            self.core.audit('child_result_query_invalid', {'key': key, 'reason': str(exc)})
            return  # An invalid/missing UUID is not evidence of exit.
        self.pending[key] = dict(action=action, uuid=uuid,
            execution=request.execution_id, attempt=request.attempt_id,
            epoch=request.epoch, future=None, cancel_sent=False)

    def tick(self):
        for key, record in list(self.pending.items()):
            if key not in self.core.records:
                self.pending.pop(key, None)
                continue
            cancel = self.cancel_clients[record['action']]
            if self.core.frozen and not record['cancel_sent'] and cancel.service_is_ready():
                request = CancelGoal.Request()
                request.goal_info.goal_id.uuid = list(record['uuid'])
                cancel.call_async(request)
                record['cancel_sent'] = True
                # This is only a cancel request. GetResult still proves terminal.
            if record['future'] is not None or not self.clients[record['action']].service_is_ready():
                continue
            request = self.ACTIONS[record['action']].Impl.GetResultService.Request()
            request.goal_id.uuid = list(record['uuid'])
            future = self.clients[record['action']].call_async(request)
            record['future'] = future
            future.add_done_callback(lambda result, k=key, r=record: self.finished(k, r, result))

    def finished(self, key, record, future):
        self.pending.pop(key, None)
        try:
            response = future.result()
        except Exception as exc:
            self.node.get_logger().error('Child result confirmation failed: ' + str(exc))
            # Keep the execution record unresolved. Never infer successful cancel.
            return
        try:
            outcomes = {GoalStatus.STATUS_SUCCEEDED: 0, GoalStatus.STATUS_ABORTED: 1,
                        GoalStatus.STATUS_CANCELED: 2}
            if response.status not in outcomes:
                self.core.audit('child_result_query_unknown', {'key': key, 'status': response.status})
                return
            self.core.navigation_control('terminal', record['execution'], record['attempt'],
                record['epoch'], key, outcomes[response.status])
        except Exception as exc:
            self.node.fail_storage(exc)
