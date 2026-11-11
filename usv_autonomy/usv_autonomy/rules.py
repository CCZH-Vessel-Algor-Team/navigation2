"""History-based left-crossing rule. Unknown observations never imply clear.

This module judges risk trend, not another vessel's mental intent. Thresholds
are configurable engineering policy, requiring scenario validation.
"""
from collections import deque
from dataclasses import dataclass, field
import math
import uuid


@dataclass
class Encounter:
    samples: deque = field(default_factory=deque)
    waiting: bool = False
    clear_since: float = None
    last_seen: float = 0.0
    missing_since: float = None
    responsibility: int = 0
    head_on_since: float = None
    responsibility_clear_since: float = None
    release_start_range: float = None
    release_last_range: float = None

    def reset_confirmation(self):
        """Break observation windows without discarding a confirmed responsibility."""
        self.head_on_since = None
        self.responsibility_clear_since = None
        self.release_start_range = self.release_last_range = None


class MaritimeRules:
    def __init__(self, clock, observe=5.0, clear_time=5.0, stale=1.5,
                 improvement=10.0, release_dcpa=40.0, wait_risk=2,
                 missing_target_timeout=5.0, stand_on_enabled=False,
                 stand_on_confirm_time=1.0, wait_tcpa=30.0, wait_dcpa=20.0,
                 critical_range=10.0, critical_tcpa=5.0, fault_timeout=5.0,
                 encounter_confirm_time=1.0, encounter_end_time=5.0,
                 encounter_release_range=40.0):
        for name, value in dict(missing_target_timeout=missing_target_timeout,
                stand_on_confirm_time=stand_on_confirm_time, wait_tcpa=wait_tcpa,
                wait_dcpa=wait_dcpa, critical_range=critical_range,
                critical_tcpa=critical_tcpa, fault_timeout=fault_timeout,
                encounter_confirm_time=encounter_confirm_time, encounter_end_time=encounter_end_time,
                encounter_release_range=encounter_release_range).items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(name + ' must be finite and positive')
        self.clock = clock
        self.observe, self.clear_time, self.stale = observe, clear_time, stale
        self.improvement, self.release_dcpa = improvement, release_dcpa
        self.wait_risk = wait_risk
        self.missing_target_timeout = missing_target_timeout
        self.stand_on_enabled = stand_on_enabled
        self.confirm_time, self.wait_tcpa, self.wait_dcpa = stand_on_confirm_time, wait_tcpa, wait_dcpa
        self.critical_range, self.critical_tcpa = critical_range, critical_tcpa
        self.fault_timeout = fault_timeout
        self.encounter_confirm_time = encounter_confirm_time
        self.encounter_end_time = encounter_end_time
        self.encounter_release_range = encounter_release_range
        self.stand_on_target = ''
        self.stand_on_engaged = ''
        self.observed_target = ''
        self.left_since = None
        self.event_id = ''
        self.event_target = ''
        self.handover_reason = ''
        self.fault_since = self.clock()
        self.encounters = {}
        self.last_array = float('-inf')
        self.revision = 0
        self.input_valid = False

    def update(self, reports, valid=True):
        now = self.clock()
        self.input_valid = valid
        if not valid:
            self.stand_on_target = ''
            self.left_since = None
            if self.fault_since is None:
                self.fault_since = now
            for encounter in self.encounters.values():
                encounter.clear_since = None
                encounter.missing_since = None
                encounter.reset_confirmation()
            return
        self.fault_since = None
        # A report after a watchdog gap starts a new absence window. Silence
        # and invalid observations are not evidence that a target disappeared.
        if now - self.last_array > self.stale:
            self.left_since = None
            for encounter in self.encounters.values():
                encounter.missing_since = None
                encounter.reset_confirmation()
        self.last_array = now
        reports = list(reports)
        # Confirm responsibility BEFORE selecting a planner or evaluating the
        # left-crossing rule. A right turn changes geometry, not the original
        # head-on obligation. This is an encounter fact, not a workflow FSM.
        for report in reports:
            target = report[0]
            e = self.encounters.setdefault(target, Encounter())
            if now - e.last_seen > self.stale:
                e.reset_confirmation()
            self.update_responsibility(e, report, now)
        self.stand_on_target = ''
        if self.stand_on_enabled and len(reports) == 1:
            target, left, cpa_valid = reports[0][:3]
            tcpa = reports[0][4]
            encounter = reports[0][7] if len(reports[0]) > 7 else (3 if left else 0)
            if (self.encounters[target].responsibility == 1 or
                    self.encounters[target].head_on_since is not None):
                left = False
                self.left_since = None
                if target == self.stand_on_engaged:
                    self.stand_on_engaged = ''
            if (target == self.stand_on_engaged and encounter in (1, 2, 4) and tcpa >= 0
                    and not self.encounters.get(target, Encounter()).waiting):
                self.stand_on_engaged = ''
            if target == self.stand_on_engaged:
                # Quiescing the old planner can reduce own speed to zero and
                # make the geometry UNKNOWN. Do not oscillate back to VO just
                # because that safe planner-switch stop erased own course.
                self.stand_on_target = target
            elif left and cpa_valid and tcpa >= 0:
                if self.left_since is None or target != self.observed_target:
                    self.left_since = now
                self.observed_target = target
                if now - self.left_since >= self.confirm_time:
                    self.stand_on_target = target
                    self.stand_on_engaged = target
            else:
                self.left_since = None
        else:
            self.left_since = None
        seen = set()
        for report in reports:
            target, left, cpa_valid, dcpa, tcpa, risk = report[:6]
            left = left or (self.stand_on_enabled and len(reports) == 1
                            and target == self.stand_on_engaged)
            if (self.encounters[target].responsibility == 1 or
                    self.encounters[target].head_on_since is not None):
                left = False
            seen.add(target)
            separation = report[6] if len(report) > 6 else dcpa
            e = self.encounters.setdefault(target, Encounter())
            e.missing_since = None
            if now - e.last_seen > self.stale:
                e.samples.clear()
                e.clear_since = None
            e.last_seen = now
            previous_range = e.samples[-1][6] if e.samples else separation
            e.samples.append((now, dcpa, tcpa, risk, left, cpa_valid, separation))
            while len(e.samples) > 1 and e.samples[1][0] < now - self.observe:
                e.samples.popleft()
            span = now - e.samples[0][0]
            stable_left = all(s[4] and s[5] for s in e.samples)
            improving = dcpa - e.samples[0][1] >= self.improvement
            danger = 0.0 <= tcpa <= self.wait_tcpa and dcpa <= self.wait_dcpa
            critical = (separation <= self.critical_range or
                        (0.0 <= tcpa <= self.critical_tcpa and dcpa <= self.wait_dcpa))
            eligible = not self.stand_on_enabled or len(reports) == 1
            dangerous = danger if self.stand_on_enabled else risk >= self.wait_risk
            if eligible and left and cpa_valid and ((critical and self.stand_on_enabled) or
                    (dangerous and span >= self.observe and stable_left and not improving)):
                e.waiting = True
            # Past DCPA can stay small forever after a safe passage. Use current
            # separation plus passed/opening geometry, not past closest distance.
            clear = (cpa_valid and tcpa < 0 and separation > self.release_dcpa
                     and separation >= previous_range - .1 and risk < self.wait_risk)
            if (e.waiting or target == self.stand_on_engaged) and clear:
                if e.clear_since is None:
                    e.clear_since = now
                if now - e.clear_since >= self.clear_time:
                    e.waiting = False
                    if target == self.stand_on_engaged:
                        self.stand_on_engaged = self.stand_on_target = ''
                        self.left_since = None
            else:
                e.clear_since = None
        for target, encounter in list(self.encounters.items()):
            if target in seen:
                continue
            encounter.clear_since = None
            encounter.reset_confirmation()
            if encounter.missing_since is None:
                encounter.missing_since = now
            if now - encounter.missing_since >= self.missing_target_timeout:
                if encounter.waiting:
                    self.latch_handover('target_missing', target)
                elif not encounter.responsibility:
                    del self.encounters[target]
                    if target == self.stand_on_engaged:
                        self.stand_on_engaged = ''
        self.revision += 1

    def update_responsibility(self, e, report, now):
        target, left, cpa_valid, dcpa, tcpa, risk = report[:6]
        separation = report[6] if len(report) > 6 else dcpa
        kind = report[7] if len(report) > 7 else (3 if left else 0)
        if not e.responsibility:
            if (cpa_valid and tcpa >= 0.0
                    and (kind == 1 or e.head_on_since is not None)):
                # VO may start turning before this confirmation window ends.
                # Keep the first head-on evidence through that geometry change;
                # confirmation needs continuous valid tracking, not a frozen
                # bearing that our own avoidance action would immediately erase.
                if e.head_on_since is None:
                    e.head_on_since = now
                if now - e.head_on_since >= self.encounter_confirm_time:
                    e.responsibility = 1
            else:
                e.head_on_since = None
            return
        passed = (cpa_valid and tcpa < 0.0 and separation > self.encounter_release_range
                  and risk < self.wait_risk)
        if (not passed or (e.release_last_range is not None
                           and separation < e.release_last_range - .1)):
            e.responsibility_clear_since = None
            e.release_start_range = e.release_last_range = None
            return
        if e.responsibility_clear_since is None:
            e.responsibility_clear_since = now
            e.release_start_range = separation
        e.release_last_range = separation
        if (now - e.responsibility_clear_since >= self.encounter_end_time
                and separation > e.release_start_range + .1):
            e.responsibility = 0
            e.reset_confirmation()
            e.samples.clear()  # A later encounter needs new trend evidence.

    def responsibilities(self):
        return {target: e.responsibility for target, e in sorted(self.encounters.items())
                if e.responsibility}

    def restore_responsibilities(self, responsibilities):
        if (not isinstance(responsibilities, dict) or any(
                not isinstance(target, str) or not target or type(kind) is not int or kind != 1
                for target, kind in responsibilities.items())):
            raise ValueError('invalid maritime responsibilities checkpoint')
        for target, kind in responsibilities.items():
            e = self.encounters.setdefault(target, Encounter(last_seen=float('-inf')))
            e.responsibility = kind
            e.reset_confirmation()  # Offline time never counts as a safe passage.

    def decision(self):
        now = self.clock()
        # Stream failure is separate from a valid snapshot with a missing ID.
        fault_start = (self.fault_since if self.fault_since is not None
                       else self.last_array + self.stale)
        if self.waiting_ids() and now - fault_start >= self.fault_timeout:
            self.latch_handover('observation_fault')
        waiting = [k for k, e in self.encounters.items() if e.waiting]
        unknown = not self.input_valid or now - self.last_array > self.stale or any(
            now - self.encounters[k].last_seen > self.stale for k in waiting)
        return bool(waiting) or bool(self.event_id), not unknown, not waiting and not unknown and not self.event_id

    def latch_handover(self, reason, target=''):
        if not self.event_id:
            self.event_id = uuid.uuid4().hex
            self.event_target = target or next(iter(self.waiting_ids()), '')
            self.handover_reason = reason
            self.revision += 1

    def reset_event(self, event_id):
        if not event_id or event_id != self.event_id:
            return False
        self.encounters.clear()
        self.event_id = self.event_target = self.handover_reason = self.stand_on_target = ''
        self.stand_on_engaged = ''
        self.left_since = None
        self.revision += 1
        return True

    def waiting_ids(self):
        return sorted(k for k, e in self.encounters.items() if e.waiting)

    def restore_waiting(self, targets):
        for target in targets:
            self.encounters[target] = Encounter(waiting=True, last_seen=float('-inf'))
