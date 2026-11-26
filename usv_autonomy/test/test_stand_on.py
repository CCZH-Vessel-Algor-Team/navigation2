"""Single-target policy facts; workflow remains in the existing task BT."""
import pytest
from usv_autonomy.rules import MaritimeRules


class Clock:
    def __init__(self):
        self.t = 0.
    def __call__(self):
        return self.t


def ship(left=True, cpa=True, dcpa=5., tcpa=40., risk=2, distance=80., target='ship'):
    return (target, left, cpa, dcpa, tcpa, risk, distance)


def feed(rules, clock, reports, seconds):
    for _ in range(seconds * 2):
        rules.update(reports)
        clock.t += .5
    rules.update(reports)


def test_left_crossing_stand_on_confirmation_and_danger_threshold():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True, observe=2.)
    feed(rules, clock, [ship()], 3)
    assert rules.stand_on_target == 'ship' and not rules.decision()[0]
    rules.update([ship(tcpa=20.)])
    assert rules.decision() == (True, True, False)


@pytest.mark.parametrize('report', [ship(left=False), ship(cpa=False)])
def test_other_or_unknown_encounters_do_not_get_stand_on(report):
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    feed(rules, clock, [report], 6)
    assert not rules.stand_on_target and not rules.decision()[0]


def test_multiple_targets_do_not_enter_single_target_policy():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    feed(rules, clock, [ship(tcpa=10.), ship(target='second')], 6)
    assert not rules.stand_on_target and not rules.decision()[0]


def test_critical_target_skips_history_window():
    rules = MaritimeRules(Clock(), stand_on_enabled=True)
    rules.update([ship(tcpa=3.)])
    assert rules.decision() == (True, True, False)


def test_configured_danger_limits_are_not_capped_by_monitor_warning_thresholds():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True, observe=2., wait_tcpa=60., wait_dcpa=50.)
    feed(rules, clock, [ship(dcpa=40., tcpa=50., risk=1)], 3)
    assert rules.decision() == (True, True, False)


def test_quiescent_low_speed_unknown_geometry_does_not_oscillate_back_to_vo():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True, observe=2.)
    feed(rules, clock, [ship()], 2)
    assert rules.stand_on_target == 'ship'
    rules.update([ship(left=False, tcpa=20.) + (0,)])
    assert rules.stand_on_target == 'ship'
    assert rules.decision() == (True, True, False)


def test_new_known_encounter_before_wait_returns_to_ordinary_planning():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    feed(rules, clock, [ship()], 2)
    rules.update([ship(left=False) + (1,)])
    assert not rules.stand_on_target and not rules.decision()[0]


def test_known_safe_passage_clears_wait_without_handover():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True, clear_time=2.)
    rules.update([ship(tcpa=3.)])
    feed(rules, clock, [ship(left=False, tcpa=-10., risk=0)], 2)
    assert rules.decision() == (False, True, True)
    assert not rules.event_id


@pytest.mark.parametrize('failure', ['invalid', 'silent'])
def test_observation_fault_requires_takeover_not_clear(failure):
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True, fault_timeout=3.)
    rules.update([ship(tcpa=3.)])
    if failure == 'invalid':
        rules.update([], False)
    clock.t = 5.
    assert rules.decision() == (True, False, False)
    assert rules.event_id and rules.handover_reason == 'observation_fault'


def test_handover_requires_event_scoped_reset_even_after_safe_target_return():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    rules.update([ship(tcpa=3.)])
    feed(rules, clock, [], 5)
    event = rules.event_id
    assert rules.event_target == 'ship'
    feed(rules, clock, [ship(left=False, tcpa=-10., risk=0)], 6)
    assert rules.decision() == (True, True, False)
    assert rules.event_target == 'ship'
    assert not rules.reset_event('old-event')
    assert rules.reset_event(event)
    assert not rules.encounters and rules.decision() == (False, True, True)


def head_on(rules, clock):
    feed(rules, clock, [ship(left=False, tcpa=20.) + (1,)], 2)
    assert rules.responsibilities() == {'ship': 1}


@pytest.mark.parametrize('stand_on_enabled', [False, True])
def test_confirmed_head_on_turning_into_left_never_enters_stand_on_or_left_wait(stand_on_enabled):
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=stand_on_enabled)
    head_on(rules, clock)
    feed(rules, clock, [ship(tcpa=20.) + (3,)], 7)
    assert rules.responsibilities() == {'ship': 1}
    assert not rules.stand_on_target and rules.decision() == (False, True, True)
    # Urgency must not change the original responsibility into a left-crossing
    # wait. VO and the independent collision gate remain responsible for it.
    rules.update([ship(tcpa=2., distance=5.) + (3,)])
    assert rules.decision() == (False, True, True) and not rules.event_id


def test_head_on_responsibility_is_per_target_not_a_global_left_rule_disable():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    head_on(rules, clock)
    feed(rules, clock, [ship(tcpa=20., target='second') + (3,)], 6)
    assert rules.responsibilities() == {'ship': 1}
    assert rules.waiting_ids() == ['second']


def test_head_on_safe_end_requires_passed_separated_opening_observations():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    head_on(rules, clock)
    for i in range(6):
        # Past DCPA remains small; the growing CURRENT range is the evidence.
        rules.update([ship(left=False, dcpa=1., tcpa=-10., risk=0, distance=80. + i) + (0,)])
        if i < 5:
            assert rules.responsibilities() == {'ship': 1}
        clock.t += 1.
    assert not rules.responsibilities()
    feed(rules, clock, [ship() + (3,)], 2)
    assert rules.stand_on_target == 'ship'  # A new encounter may have a new role.


@pytest.mark.parametrize('report', [
    ship(left=False, tcpa=10., risk=0, distance=80.),
    ship(left=False, tcpa=-10., risk=0, distance=39.),
    ship(left=False, cpa=False, tcpa=-10., risk=0, distance=80.),
    ship(left=False, tcpa=-10., risk=2, distance=80.),
    ship(left=False, tcpa=-10., risk=0, distance=80.),  # constant, not opening
])
def test_low_risk_or_a_passed_time_alone_does_not_end_head_on(report):
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    head_on(rules, clock)
    feed(rules, clock, [report + (0,)], 7)
    assert rules.responsibilities() == {'ship': 1}


@pytest.mark.parametrize('interruption', ['invalid', 'missing', 'silent', 'closing'])
def test_head_on_safe_end_window_restarts_after_interruption(interruption):
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    head_on(rules, clock)
    for i in range(4):
        rules.update([ship(left=False, tcpa=-10., risk=0, distance=80. + i) + (0,)])
        clock.t += 1.
    if interruption == 'invalid':
        rules.update([], valid=False)
    elif interruption == 'missing':
        rules.update([])
    elif interruption == 'silent':
        clock.t += 2.
    else:
        rules.update([ship(left=False, tcpa=-10., risk=0, distance=70.) + (0,)])
    for i in range(6):
        rules.update([ship(left=False, tcpa=-10., risk=0, distance=90. + i) + (0,)])
        if i < 5:
            assert rules.responsibilities() == {'ship': 1}
        clock.t += 1.
    assert not rules.responsibilities()


def test_missing_or_invalid_head_on_data_cannot_create_a_new_left_encounter():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    head_on(rules, clock)
    feed(rules, clock, [], 7)
    assert rules.responsibilities() == {'ship': 1}
    assert rules.decision() == (False, True, True) and not rules.event_id
    rules.update([], valid=False)
    clock.t += 10.
    feed(rules, clock, [ship(tcpa=10.) + (3,)], 7)
    assert rules.responsibilities() == {'ship': 1} and not rules.decision()[0]


def test_head_on_confirmation_does_not_accumulate_across_stale_observations():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    rules.update([ship(left=False) + (1,)])
    clock.t += 2.
    rules.update([ship(left=False) + (1,)])
    assert not rules.responsibilities()


def test_early_avoidance_turn_cannot_erase_pending_head_on_confirmation():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True, stand_on_confirm_time=.1)
    rules.update([ship(left=False, tcpa=20.) + (1,)])
    for i in range(1, 5):
        clock.t = i * .25
        rules.update([ship(tcpa=3.) + (3,)])
        assert not rules.stand_on_target and not rules.decision()[0]
    assert rules.responsibilities() == {'ship': 1}


def test_restored_responsibility_restarts_release_timer_and_never_clears_handover():
    clock = Clock()
    rules = MaritimeRules(clock, stand_on_enabled=True)
    rules.restore_waiting(['ship'])
    rules.restore_responsibilities({'ship': 1})
    rules.latch_handover('target_missing', 'ship')
    event = rules.event_id
    clock.t += 100.
    for i in range(6):
        rules.update([ship(left=False, tcpa=-10., risk=0, distance=80. + i) + (0,)])
        if i < 5:
            assert rules.responsibilities() == {'ship': 1}
        clock.t += 1.
    assert not rules.responsibilities() and rules.event_id == event
    assert rules.decision() == (True, True, False)
    assert rules.reset_event(event) and not rules.responsibilities()


@pytest.mark.parametrize('roles', [[], {'ship': 3}, {'ship': True}, {'': 1}])
def test_invalid_responsibility_checkpoint_is_rejected(roles):
    with pytest.raises(ValueError, match='responsibilities'):
        MaritimeRules(Clock()).restore_responsibilities(roles)


@pytest.mark.parametrize('parameter', ['encounter_confirm_time', 'encounter_end_time', 'encounter_release_range'])
def test_invalid_encounter_threshold_is_rejected(parameter):
    with pytest.raises(ValueError, match=parameter):
        MaritimeRules(Clock(), **{parameter: 0.})
