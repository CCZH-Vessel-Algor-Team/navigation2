"""Fault-oriented contract tests, runnable without ROS or generated messages."""
import itertools
import pytest
from usv_autonomy.gate import Gate, DISABLED, NEUTRAL, VELOCITY_CONTROL
from usv_autonomy.lifecycle import Coordinator, SUCCESS, FAILURE, RUNNING
from usv_autonomy.rules import MaritimeRules
from usv_autonomy.store import Store
from usv_autonomy.health import HealthPolicy


class Clock:
    def __init__(self):
        self.t = 0.0
    def __call__(self):
        return self.t
    def advance(self, value):
        self.t += value


@pytest.fixture
def lifecycle(tmp_path):
    clock = Clock()
    store = Store(tmp_path / 'tasks.sqlite3')
    c = Coordinator(store, clock)
    c.blockers.clear()
    c.set_authority(True)
    c.producer_update('', '', 0, False, 'producer')
    c.acknowledge_gate(c.boot, c.permit_revision, False)
    spec = dict(mode='NAVIGATION', goals=[{'frame': 'map', 'position': [5, 0, 0],
                                          'orientation': [0, 0, 0, 1]}], hold={})
    assert c.manage('start', 'mission', spec, 'start')[0]
    yield c, clock, store
    store.close()


def begin(c):
    assert c.activate() == SUCCESS
    c.outer_terminal = False
    assert c.navigation_control('begin', c.execution) == SUCCESS


def acknowledge_stop(c, terminal=True):
    c.acknowledge_gate(c.boot, c.permit_revision, False)
    c.producer_update(c.execution, c.attempt, c.epoch, False, 'producer')
    c.outer_terminal = terminal


def test_outer_terminal_is_not_child_exit(lifecycle):
    c, clock, _ = lifecycle
    begin(c)
    old = c.execution
    assert c.navigation_control('register', old, c.attempt, c.epoch, 'child') == SUCCESS
    c.stop()
    acknowledge_stop(c)
    assert not c.quiet()
    clock.advance(3.1)
    c.tick()
    assert c.fault and not c.permitted and c.activate() != SUCCESS
    assert c.execution == old


def test_pending_registration_blocks_reentry_and_late_terminal_is_scoped(lifecycle):
    c, _, _ = lifecycle
    begin(c)
    args = c.execution, c.attempt, c.epoch
    assert c.navigation_control('register', *args, 'pending') == SUCCESS
    c.stop()
    acknowledge_stop(c)
    c.navigation_control('terminal', 'wrong', args[1], args[2], 'pending')
    assert not c.quiet()
    c.navigation_control('terminal', *args, 'pending')
    assert c.quiet()
    c.tick()
    assert not c.execution


def test_local_recovery_does_not_cancel_outer_or_timeout_during_wait(lifecycle):
    c, clock, _ = lifecycle
    begin(c)
    old = c.execution
    c.navigation_control('failure', old, c.attempt, c.epoch, retryable=True)
    acknowledge_stop(c, terminal=False)
    c.tick()
    assert c.execution == old and c.cancel_since is None
    assert c.navigation_control('retry', old, c.attempt, c.epoch) == SUCCESS
    clock.advance(5.0)
    c.producer_update(old, c.attempt, c.epoch, False, 'producer')
    c.acknowledge_gate(c.boot, c.permit_revision, False)
    c.tick()
    assert not c.fault
    epoch = c.epoch
    assert c.navigation_control('begin', old, c.attempt, c.epoch) == SUCCESS
    assert c.epoch > epoch


@pytest.mark.parametrize('events', list(itertools.permutations(['risk_clear', 'authority_return', 'protection_clear'])))
def test_automatic_events_never_clear_manual_pause(lifecycle, events):
    c, _, _ = lifecycle
    begin(c)
    c.manage('pause', 'mission', request_id='pause')
    acknowledge_stop(c)
    c.tick()
    for event in events:
        if event == 'authority_return':
            c.set_authority(True)
        else:
            c.set_blocker(event, False)
    assert c.task['intent'] == 'PAUSE' and not c.runnable()
    assert c.activate() != SUCCESS


def test_unavailable_mode_does_not_destroy_active_context(lifecycle):
    c, _, _ = lifecycle
    begin(c)
    old, epoch = c.execution, c.epoch
    assert c.manage('switch', 'dock', {'mode': 'DOCK'}, 'dock')[1] == 501
    assert c.execution == old and c.epoch == epoch and c.permitted


def test_survey_mode_accepted_with_geometry(lifecycle):
    c, _, _ = lifecycle
    # Quiesce the fixture NAVIGATION execution before mode switch.
    c.stop()
    acknowledge_stop(c)
    c.tick()
    ok, code, _ = c.manage(
        'start', 'survey-1',
        {'mode': 'SURVEY', 'goals': [], 'hold': {}, 'survey': {'leg_length_m': 300.0}},
        'survey-start')
    assert ok and code == 202
    assert c.task['spec']['mode'] == 'SURVEY'
    bad, bad_code, _ = c.manage(
        'start', 'survey-bad', {'mode': 'SURVEY', 'goals': []}, 'survey-bad')
    assert not bad and bad_code == 400


@pytest.mark.parametrize('late_register', [False, True])
@pytest.mark.parametrize('key', ['compute_path_to_pose:unsent', 'follow_path:unsent'])
def test_proven_unsent_registration_cannot_leak_or_reappear(lifecycle, late_register, key):
    c, _, _ = lifecycle
    producer = c.producer
    begin(c)
    args = c.execution, c.attempt, c.epoch
    if not late_register:
        assert c.navigation_control('register', *args, key) == SUCCESS
    assert c.navigation_control('not_sent', *args, key) == SUCCESS
    if late_register:
        assert c.navigation_control('register', *args, key) == FAILURE
    c.stop()
    acknowledge_stop(c)
    # An unsent FollowPath never ran a controller under the new identity.
    c.producer_update(*producer[:3], False, producer[4])
    assert c.quiet()
    c.tick()
    assert not c.execution and not c.fault


def test_unsent_evidence_cannot_retire_accepted_or_wrong_identity(lifecycle):
    c, _, _ = lifecycle
    begin(c)
    args = c.execution, c.attempt, c.epoch
    assert c.navigation_control('register', *args, 'follow_path:accepted') == SUCCESS
    assert c.navigation_control('accepted', *args, 'follow_path:accepted', goal_id='1' * 32) == SUCCESS
    assert c.navigation_control('not_sent', *args, 'follow_path:accepted') == FAILURE
    assert c.navigation_control('not_sent', args[0], 'wrong', args[2], 'follow_path:accepted') == FAILURE
    c.stop()
    acknowledge_stop(c)
    assert not c.quiet()
    c.navigation_control('terminal', *args, 'follow_path:accepted')
    assert c.quiet()


def test_multiple_unsent_follow_registrations_restore_only_unstarted_producer(lifecycle):
    c, _, _ = lifecycle
    begin(c)
    args = c.execution, c.attempt, c.epoch
    for key in ('follow_path:first', 'follow_path:second'):
        assert c.navigation_control('register', *args, key) == SUCCESS
    for key in ('follow_path:first', 'follow_path:second'):
        assert c.navigation_control('not_sent', *args, key) == SUCCESS
    assert c.producer_exit_expected is None
    assert c.navigation_control('register', *args, 'follow_path:ran') == SUCCESS
    c.navigation_control('accepted', *args, 'follow_path:ran', goal_id='1' * 32)
    c.navigation_control('terminal', *args, 'follow_path:ran')
    c.navigation_control('register', *args, 'follow_path:never-sent')
    c.navigation_control('not_sent', *args, 'follow_path:never-sent')
    assert c.producer_exit_expected == args  # Earlier actual controller still needs its own idle.


def test_navigation_mode_switch_restores_source_route_across_restart(lifecycle):
    c, clock, store = lifecycle
    route = c.task
    route['task_retries'], route['leg_retries'] = 2, {'0:0': 2}
    # A newer route exists, but HOLD was entered from the older resumed route.
    assert c.manage('start', 'newer', route['spec'], 'newer')[0]
    assert c.manage('resume_task', 'mission', request_id='select-original')[0]
    acknowledge_stop(c)
    c.tick()
    assert c.task['id'] == 'mission'
    assert c.manage('switch', 'hold', {'mode': 'HOLD', 'goals': [], 'hold': {}}, 'hold')[0]
    assert c.task['spec']['mode'] == 'HOLD'
    c2 = Coordinator(store, clock)
    c2.blockers.clear()
    c2.set_authority(True)
    c2.producer_update('', '', 0, False, 'producer')
    c2.acknowledge_gate(c2.boot, c2.permit_revision, False)
    assert c2.manage('resume_task', 'hold', request_id='restore-hold')[0]
    c2.tick()
    assert c2.manage('resume_navigation', request_id='navigation-button')[0]
    c2.tick()
    assert c2.task['id'] == 'mission'
    assert c2.task['spec'] == route['spec']
    assert c2.task['task_retries'] == 2 and c2.task['leg_retries'] == {'0:0': 2}
    assert c2.task['intent'] == 'PAUSE' and not c2.runnable()
    assert c2.manage('resume_navigation', request_id='navigation-button')[0]  # Idempotent.
    assert c2.manage('stop', 'mission', request_id='end-source')[0]
    assert c2.manage('resume_task', 'hold', request_id='select-hold')[0]
    c2.tick()
    assert c2.manage('resume_navigation', request_id='source-ended')[1] == 409
    assert c2.task['id'] == 'hold'  # Never silently substitute the unrelated newer route.


def test_mode_round_trip_selects_paused_context_and_requires_continue(lifecycle):
    c, _, store = lifecycle
    route = c.task
    route['completed'], route['index'], route['lap'] = [], 0, 2
    route['task_retries'] = 2
    hold_spec = dict(mode='HOLD', goals=[], hold=dict(
        anchor=dict(frame='map', position=[3., 4., 0.]), inner=3., outer=10.))
    assert c.manage('switch', 'hold-one', hold_spec, 'hold-one')[0]
    hold = c.task
    hold['hold_returns'], hold['hold_elapsed'], hold['task_retries'] = 2, 17., 1
    assert c.manage('resume_navigation', request_id='select-navigation')[0]
    acknowledge_stop(c)
    c.tick()
    assert c.task is route and route['intent'] == 'PAUSE'
    assert not c.runnable() and c.activate() != SUCCESS and not c.permitted
    assert route['lap'] == 2 and route['task_retries'] == 2
    assert c.manage('resume', route['id'], request_id='continue-navigation')[0]
    assert c.runnable()
    new_hold_spec = dict(mode='HOLD', goals=[], hold=dict(
        anchor=dict(frame='map', position=[8., 9., 0.], orientation=[0., 0., 0., 1.]),
        inner=3., outer=10., use_current_pose=True))
    assert c.manage('switch', 'new-ui-id', new_hold_spec, 'select-hold')[0]
    acknowledge_stop(c)
    c.tick()
    assert c.task is hold and hold['intent'] == 'PAUSE' and not c.runnable()
    assert 'new-ui-id' not in c.tasks
    assert hold['hold_anchor']['position'] == [8., 9., 0.]
    assert hold['hold_returns'] == 2 and hold['hold_elapsed'] == 17.
    assert store.tasks()[hold['id']]['task_retries'] == 1
    retransmit = dict(mode='HOLD', goals=[], hold=dict(new_hold_spec['hold']))
    retransmit['hold']['anchor'] = dict(frame='map', position=[50., 60., 0.],
                                      orientation=[0., 0., 0., 1.])
    assert c.manage('switch', 'new-ui-id', retransmit, 'select-hold')[0]
    assert hold['hold_anchor']['position'] == [8., 9., 0.]  # One request, one capture.
    assert c.manage('resume', hold['id'], request_id='continue-hold')[0]
    assert c.runnable()


def test_manual_hold_reanchor_same_task_is_paused_and_idempotent(lifecycle):
    c, _, store = lifecycle
    anchor = dict(frame='map', position=[3., 4., 0.], orientation=[0., 0., 0., 1.])
    spec = dict(mode='HOLD', goals=[], hold=dict(anchor=anchor, inner=3., outer=10.))
    assert c.manage('switch', 'hold-one', spec, 'hold-one')[0]
    hold = c.task
    hold['hold_returns'], hold['hold_elapsed'], hold['task_retries'] = 2, 17., 1
    manual = dict(mode='HOLD', goals=[], hold=dict(anchor=dict(
        frame='map', position=[-12., 8., 0.], orientation=[0., 0., 0., 1.]), use_current_pose=False))
    assert c.manage('switch', 'ignored-new-id', manual, 'manual-anchor')[0]
    acknowledge_stop(c)
    c.tick()
    assert c.task is hold and hold['intent'] == 'PAUSE' and not c.runnable()
    assert hold['hold_anchor']['position'] == [-12., 8., 0.]
    assert not hold['spec']['hold']['use_current_pose']
    assert hold['spec']['hold']['inner'] == 3. and hold['spec']['hold']['outer'] == 10.
    assert hold['hold_returns'] == 2 and hold['hold_elapsed'] == 17. and hold['task_retries'] == 1
    assert c.manage('switch', 'ignored-new-id', manual, 'manual-anchor')[0]
    assert store.tasks()['hold-one']['hold_anchor'] == manual['hold']['anchor']


def test_navigation_mode_without_saved_route_does_not_replace_hold(lifecycle):
    c, _, _ = lifecycle
    assert c.manage('stop', 'mission', request_id='finish-route')[0]
    assert c.manage('switch', 'hold', {'mode': 'HOLD', 'goals': [], 'hold': {}}, 'hold')[0]
    assert c.manage('resume_navigation', request_id='navigation-button')[1] == 409
    assert c.task['id'] == 'hold' and c.task['intent'] == 'RUN'


def test_retry_reservation_is_idempotent_across_pause_and_restart(lifecycle):
    c, _, store = lifecycle
    begin(c)
    c.navigation_control('failure', c.execution, c.attempt, c.epoch, retryable=True)
    acknowledge_stop(c, terminal=False)
    c.tick()
    assert c.reserve_retry() == SUCCESS
    assert c.reserve_retry() == SUCCESS
    assert c.task['task_retries'] == 1
    assert c.manage('pause', 'mission', request_id='pause')[0]
    c2 = Coordinator(store, Clock())
    restored = c2.tasks['mission']
    assert restored['task_retries'] == 1
    assert restored['intent'] == 'PAUSE'
    assert not c2.execution


def test_commit_barrier_and_persistent_completion(lifecycle):
    c, _, store = lifecycle
    begin(c)
    old = c.execution
    c.result = 'SUCCEEDED'
    c.stop()
    assert c.commit_waypoint(old) == FAILURE
    assert store.tasks()['mission']['index'] == 0
    acknowledge_stop(c)
    c.tick()
    c.activate()
    c.result = 'SUCCEEDED'
    assert c.commit_waypoint(c.execution) == SUCCESS
    assert store.tasks()['mission']['completed'] == [0]
    assert store.tasks()['mission']['index'] == 1


def test_stop_is_terminal_and_request_is_idempotent(lifecycle):
    c, _, _ = lifecycle
    first = c.manage('stop', 'mission', request_id='stop')
    assert c.manage('stop', 'mission', request_id='stop') == first
    assert c.manage('resume', 'mission', request_id='resume')[1] == 409
    c.set_authority(False)
    assert not c.task['return_confirmation']


def test_resume_does_not_ack_success_while_cancel_evidence_is_missing(lifecycle):
    c, clock, _ = lifecycle
    begin(c)
    args = c.execution, c.attempt, c.epoch
    c.navigation_control('register', *args, 'compute_path_to_pose:unresolved')
    c.manage('pause', 'mission', request_id='pause')
    acknowledge_stop(c)
    clock.advance(3.1)
    acknowledge_stop(c)
    c.tick()
    result = c.manage('resume', 'mission', request_id='resume')
    assert not result[0] and result[1] == 409 and 'compute_path_to_pose:unresolved' in result[2]
    assert c.task['intent'] == 'PAUSE' and not c.permitted


def test_explicit_resume_can_reenter_after_late_terminal_proves_cleanup(lifecycle):
    c, clock, _ = lifecycle
    begin(c)
    old = c.execution
    args = c.execution, c.attempt, c.epoch
    c.navigation_control('register', *args, 'compute_path_to_pose:late')
    c.manage('pause', 'mission', request_id='pause')
    clock.advance(3.1)
    acknowledge_stop(c)
    c.tick()
    assert c.fault
    c.navigation_control('terminal', *args, 'compute_path_to_pose:late', outcome=2)
    acknowledge_stop(c)
    c.tick()
    assert c.task['intent'] == 'PAUSE' and c.fault and not c.execution
    assert c.manage('resume', 'mission', request_id='resume')[0]
    assert not c.fault and c.activate() == SUCCESS and c.execution != old
    assert c.task['index'] == 0 and c.task['task_retries'] == 0


def test_immediate_continue_is_pending_until_old_execution_really_exits(lifecycle):
    c, _, _ = lifecycle
    begin(c)
    old = c.execution
    c.manage('pause', 'mission', request_id='pause')
    c.producer_update(c.execution, c.attempt, c.epoch, True, 'producer')
    result = c.manage('resume', 'mission', request_id='resume')
    assert result[:2] == (True, 202)
    assert not c.permitted and c.activate() == RUNNING and c.execution == old
    acknowledge_stop(c)
    c.tick()
    assert c.activate() == SUCCESS and c.execution != old


def test_resume_reports_safety_blocker_without_erasing_manual_pause(lifecycle):
    c, _, _ = lifecycle
    c.manage('pause', 'mission', request_id='pause')
    c.set_blocker('regular_safety', True)
    result = c.manage('resume', 'mission', request_id='resume')
    assert result[1] == 409 and 'regular_safety' in result[2]
    c.set_blocker('regular_safety', False)
    assert c.task['intent'] == 'PAUSE'


def permitted_gate():
    clock = Clock()
    g = Gate(clock)
    g.authority = (True, clock())
    g.safety = (True, 1.0, clock())
    g.heartbeat = clock()
    assert g.update_permit('boot', 'exec', 'try', 1, 1, True, .5)
    return g, clock


def test_gate_rejects_late_commands_replayed_sequence_and_revoked_renewal():
    g, _ = permitted_gate()
    assert g.accept('boot', 'exec', 'try', 1, 1, (1., .2), 0, .25)
    assert g.output()[0] == VELOCITY_CONTROL
    assert not g.accept('boot', 'exec', 'try', 1, 1, (1., .2), 0, .25)
    assert not g.accept('boot', 'exec', 'try', 1, 2, (1., .2), .3, .25)
    assert g.update_permit('boot', 'exec', 'try', 1, 2, False, .5)
    assert not g.update_permit('boot', 'exec', 'try', 1, 1, True, .5)
    assert not g.update_permit('boot', 'exec', 'try', 1, 2, True, .5)
    assert g.output()[0] == NEUTRAL


def test_gate_has_independent_timeouts_and_external_authority():
    g, clock = permitted_gate()
    g.accept('boot', 'exec', 'try', 1, 1, (1., .2), 0, .25)
    clock.advance(.3)
    assert g.output()[0] == NEUTRAL
    g.authority = (False, clock())
    assert g.output()[0] == DISABLED


def test_expired_epoch_requires_new_authorization_identity():
    g, clock = permitted_gate()
    clock.advance(.6)
    assert not g.update_permit('boot', 'exec', 'try', 1, 2, True, .5)
    assert g.update_permit('boot', 'exec', 'new-try', 2, 3, True, .5)


def test_command_timeout_does_not_reset_epoch_sequence():
    g, clock = permitted_gate()
    assert g.accept('boot', 'exec', 'try', 1, 10, (1., 0.), 0., .25)
    clock.advance(.3)
    assert g.output()[0] == NEUTRAL
    assert not g.accept('boot', 'exec', 'try', 1, 10, (1., 0.), 0., .25)


def test_old_attempt_stop_cannot_revoke_new_attempt(lifecycle):
    c, clock, _ = lifecycle
    begin(c)
    old = c.attempt, c.epoch
    c.navigation_control('failure', c.execution, *old, retryable=True)
    acknowledge_stop(c, terminal=False)
    c.tick()
    c.reserve_retry()
    clock.advance(5.)
    acknowledge_stop(c, terminal=False)
    assert c.navigation_control('begin', c.execution, *old) == SUCCESS
    assert c.navigation_control('stop', c.execution, *old) == FAILURE
    assert c.permitted


def test_idle_evidence_must_match_producer_attempt_and_follow_terminal(lifecycle):
    c, clock, _ = lifecycle
    begin(c)
    args = c.execution, c.attempt, c.epoch
    c.navigation_control('register', *args, 'follow_path:test')
    c.stop()
    c.outer_terminal = True
    c.acknowledge_gate(c.boot, c.permit_revision, False)
    c.navigation_control('terminal', *args, 'follow_path:test')
    assert not c.quiet()  # startup's idle evidence does not prove this loop exited
    c.producer_update('unrelated', c.attempt, c.epoch, False, 'producer')
    assert not c.quiet()
    clock.advance(.1)
    c.producer_update(*args, False, 'producer')
    assert c.quiet()


def test_unknown_failure_does_not_blindly_retry(lifecycle):
    c, _, _ = lifecycle
    begin(c)
    c.navigation_control('failure', c.execution, c.attempt, c.epoch, retryable=False)
    acknowledge_stop(c, terminal=False)
    c.tick()
    assert c.reserve_retry() == FAILURE
    assert c.task['task_retries'] == 0
    assert c.blockers['nonretryable_failure']


def test_request_id_payload_conflict_is_rejected(lifecycle):
    c, _, _ = lifecycle
    c.manage('pause', 'mission', request_id='same-request')
    assert c.manage('resume', 'mission', request_id='same-request')[1] == 409


def test_old_epoch_cannot_be_relabelled_when_new_permit_opens():
    g, _ = permitted_gate()
    g.update_permit('boot', 'new-exec', 'new-try', 2, 2, True, .5)
    assert not g.accept('boot', 'exec', 'try', 1, 100, (1., 0.), 0, .25)
    assert g.output()[0] == NEUTRAL


def test_multi_target_wait_does_not_clear_on_missing_or_invalid_cpa():
    clock = Clock()
    rules = MaritimeRules(clock, observe=1., clear_time=1.)
    for _ in range(3):
        rules.update([('left', True, True, 5., 20., 2), ('second', True, True, 4., 20., 2)])
        clock.advance(.5)
    assert rules.decision()[0]
    rules.update([('left', True, False, 100., 0., 0)])
    assert rules.decision()[0]
    rules.update([])
    assert rules.decision()[0]
    clock.advance(2.)
    assert rules.decision() == (True, False, False)


def test_stable_passed_target_releases_but_transient_low_risk_does_not():
    clock = Clock()
    rules = MaritimeRules(clock, observe=1., clear_time=1.)
    for _ in range(3):
        rules.update([('ship', True, True, 5., 10., 2)])
        clock.advance(.5)
    rules.update([('ship', True, True, 100., 10., 0)])
    assert rules.decision()[0]
    for _ in range(3):
        rules.update([('ship', False, True, 100., -10., 0)])
        clock.advance(.5)
    assert rules.decision() == (False, True, True)


def test_passed_target_uses_current_range_not_past_dcpa():
    clock = Clock()
    rules = MaritimeRules(clock, observe=1., clear_time=1.)
    for _ in range(3):
        rules.update([('ship', True, True, 5., 10., 2, 30.)])
        clock.advance(.5)
    assert rules.decision()[0]
    restored = MaritimeRules(clock, observe=1., clear_time=1.)
    restored.restore_waiting(rules.waiting_ids())
    restored.update([])
    assert restored.decision() == (True, False, False)
    for distance in (80., 90., 100.):
        restored.update([('ship', False, True, 5., -10., 0, distance)])
        clock.advance(.5)
    assert restored.decision() == (False, True, True)


def test_invalid_assessment_is_not_an_empty_clear_scene():
    rules = MaritimeRules(Clock())
    rules.update([], valid=False)
    assert rules.decision() == (False, False, False)


def test_missing_target_latches_handover_at_default_timeout():
    clock = Clock()
    rules = MaritimeRules(clock, observe=1.)
    for _ in range(3):
        rules.update([('ship', True, True, 5., 10., 2)])
        clock.advance(.5)
    assert rules.waiting_ids() == ['ship']
    rules.update([])
    for _ in range(9):
        clock.advance(.5)
        rules.update([])
    assert rules.decision() == (True, False, False)
    clock.advance(.5)
    rules.update([])
    assert rules.decision() == (True, False, False)
    assert rules.event_id and rules.handover_reason == 'target_missing'
    assert rules.waiting_ids() == ['ship']
    # Reappearance never acknowledges a manual takeover/reset event.
    rules.update([('ship', True, True, 5., 10., 2)])
    assert rules.decision()[0] and rules.event_id


def test_restored_target_gets_full_configured_absence_window():
    clock = Clock()
    rules = MaritimeRules(clock, missing_target_timeout=3.)
    rules.restore_waiting(['ship'])
    clock.advance(100.)
    assert rules.decision() == (True, False, False)
    rules.update([])
    for _ in range(2):
        clock.advance(1.)
        rules.update([])
        assert rules.waiting_ids() == ['ship']
    clock.advance(1.)
    rules.update([])
    assert rules.decision() == (True, False, False)
    assert rules.event_id


@pytest.mark.parametrize('interruption', ['invalid', 'silent', 'reappeared'])
def test_missing_target_timeout_restarts_after_interruption(interruption):
    clock = Clock()
    rules = MaritimeRules(clock)
    rules.restore_waiting(['ship'])
    rules.update([])
    for _ in range(4):
        clock.advance(1.)
        rules.update([])
    if interruption == 'invalid':
        rules.update([], valid=False)
    elif interruption == 'silent':
        clock.advance(2.)
        assert rules.decision() == (True, False, False)
    else:
        # Even invalid CPA means the target is present, not disappeared.
        rules.update([('ship', True, False, 5., 10., 0)])
    rules.update([])
    for _ in range(4):
        clock.advance(1.)
        rules.update([])
        assert rules.waiting_ids() == ['ship']
    clock.advance(1.)
    rules.update([])
    assert rules.decision() == (True, False, False)
    assert rules.event_id


def test_missing_target_timeout_does_not_clear_other_present_waiting_targets():
    clock = Clock()
    rules = MaritimeRules(clock)
    rules.restore_waiting(['missing', 'present'])
    present = [('present', True, True, 5., 10., 2)]
    for _ in range(6):
        rules.update(present)
        clock.advance(1.)
    assert rules.waiting_ids() == ['missing', 'present']
    assert rules.decision() == (True, False, False)
    assert rules.event_id


def test_invalid_reports_and_silence_never_retire_waiting_targets():
    clock = Clock()
    rules = MaritimeRules(clock)
    rules.restore_waiting(['ship'])
    rules.update([])
    for _ in range(10):
        clock.advance(1.)
        rules.update([], valid=False)
    clock.advance(100.)
    assert rules.decision() == (True, False, False)
    assert rules.waiting_ids() == ['ship']


@pytest.mark.parametrize('timeout', [0., -1., float('nan'), float('inf')])
def test_missing_target_timeout_must_be_finite_and_positive(timeout):
    with pytest.raises(ValueError, match='missing_target_timeout'):
        MaritimeRules(Clock(), missing_target_timeout=timeout)


def test_regular_health_localization_quality_and_stable_recovery():
    clock = Clock()
    health = HealthPolicy(clock, stable_time=.5)
    health.localization(1., True)
    assert not health.evaluate()[0]
    clock.advance(.5)
    health.localization(5., True)
    assert health.evaluate()[:2] == (True, .3)
    health.localization(10., True)
    assert not health.evaluate()[0]
    health.localization(1., True)
    assert not health.evaluate()[0]
    clock.advance(.5)
    health.localization(1., True)
    assert health.evaluate()[:2] == (True, 1.)


def test_regular_sources_are_aggregated_not_last_writer_wins():
    clock = Clock()
    health = HealthPolicy(clock, stable_time=0., required_sources=('weather', 'perception'))
    health.localization(0., True)
    health.source('weather', False)
    health.source('perception', True)
    assert not health.evaluate()[0]
    health.source('weather', True)
    assert health.evaluate()[0]
    clock.advance(1.1)
    health.localization(0., True)
    health.source('perception', True)
    assert not health.evaluate()[0]  # missing weather freshness cannot be cleared by perception


def test_regular_health_rejects_nonfinite_or_invalid_pose():
    health = HealthPolicy(Clock(), stable_time=0.)
    health.localization(float('nan'), True)
    assert not health.evaluate()[0]
    health.localization(0., False)
    assert not health.evaluate()[0]
