"""Independent result evidence when a halted BT leaf cannot spin callbacks."""
from concurrent.futures import Future
from types import SimpleNamespace
import pytest
from action_msgs.msg import GoalStatus
from usv_autonomy.child_results import ChildResults
from usv_autonomy.lifecycle import Coordinator
from usv_autonomy.store import Store


class Client:
    def __init__(self):
        self.requests = []
        self.future = Future()
    def service_is_ready(self):
        return True
    def call_async(self, request):
        self.requests.append(request)
        return self.future


class Node:
    def __init__(self):
        self.clients = {}
    def create_client(self, service, name):
        self.clients[name] = Client()
        return self.clients[name]
    def get_logger(self):
        return SimpleNamespace(error=lambda _: None)
    def fail_storage(self, error):
        raise error


@pytest.fixture
def tracked(tmp_path):
    store = Store(tmp_path / 'task.db')
    core = Coordinator(store, lambda: 0.)
    core.execution, core.attempt, core.epoch = 'execution', 'attempt', 1
    key = 'follow_path:late-result'
    core.records[key] = ('execution', 'attempt', 1, 0.)
    request = SimpleNamespace(key=key, execution_id='execution', attempt_id='attempt',
                              epoch=1, goal_id='000102030405060708090a0b0c0d0e0f')
    node = Node()
    watcher = ChildResults(node, core)
    yield core, watcher, node.clients['follow_path/_action/get_result'], request
    store.close()


@pytest.mark.parametrize('status', [GoalStatus.STATUS_CANCELED,
                                   GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED])
def test_late_action_result_retires_exact_record_without_leaf_callback(tracked, status):
    core, watcher, client, request = tracked
    watcher.watch(request)
    watcher.tick()
    assert list(client.requests[0].goal_id.uuid) == list(range(16))
    assert request.key in core.records  # Sending GetResult does not prove exit.
    client.future.set_result(SimpleNamespace(status=status))
    assert request.key not in core.records
    assert not core.quiet()  # Still requires idle producer and revocation ACK.


def test_unknown_result_is_not_cancel_confirmation(tracked):
    core, watcher, client, request = tracked
    watcher.watch(request)
    watcher.tick()
    client.future.set_result(SimpleNamespace(status=GoalStatus.STATUS_UNKNOWN))
    assert request.key in core.records


def test_stale_result_cannot_retire_a_different_execution(tracked):
    core, watcher, client, request = tracked
    watcher.watch(request)
    watcher.tick()
    core.records[request.key] = ('new-execution', 'new-attempt', 2, 0.)
    client.future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert core.records[request.key][0] == 'new-execution'


def test_ambiguous_uuid_is_kept_unresolved(tracked):
    core, watcher, client, request = tracked
    request.goal_id = '1234'  # Old non-padded byte conversion must not be guessed.
    watcher.watch(request)
    watcher.tick()
    assert not client.requests and request.key in core.records


def test_late_acceptance_after_halt_gets_cancelled_by_exact_uuid(tracked):
    core, watcher, client, request = tracked
    assert core.frozen
    watcher.watch(request)
    watcher.tick()
    cancel = watcher.cancel_clients['follow_path']
    assert list(cancel.requests[0].goal_info.goal_id.uuid) == list(range(16))
    assert request.key in core.records  # cancellation request alone is not terminal
