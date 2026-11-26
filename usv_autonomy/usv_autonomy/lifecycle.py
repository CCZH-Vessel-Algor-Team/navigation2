"""Small lifecycle/lease coordinator; navigation and mode workflows live in XML.

All methods run on one event-loop writer. Wall/ROS stamps never measure timeouts.
"""
import copy
import json
import math
import uuid

FAILURE, SUCCESS, RUNNING = 0, 1, 2
TASK_BLOCKERS = {'retry_exhausted', 'execution_failed', 'nonretryable_failure', 'hold_timeout'}


def identity():
    return uuid.uuid4().hex


class Coordinator:
    def __init__(self, store, clock, cancel_timeout=3.0, retries=3,
                 task_retries=12, recovery_timeout=120.0, auto_return=False, retry_wait=5.0):
        self.store, self.clock = store, clock
        self.boot = identity()
        self.revision = 1
        self.tasks = store.tasks()
        self.task = None
        self.pending_task = None
        self.execution = ''
        self.attempt = ''
        self.epoch = 0
        self.permit_revision = 1
        self.permitted = False
        self.frozen = True
        self.records = {}
        self.unsent = set()
        self.accepted = set()
        self.registration_previous_producer = {}
        self.producer_confirmed = set()
        self.cancel_since = None
        self.cancel_scope = 'EXECUTION'
        self.fault = ''
        self.authority = False
        self.was_authority = False
        self.auto_return = auto_return
        self.blockers = {'startup': True}
        self.producer = None
        self.producer_exit_expected = None
        self.producer_barrier = float('-inf')
        self.gate_ack = None
        self.outer_pending = False
        self.outer_terminal = True
        self.cancel_timeout = cancel_timeout
        self.max_retries, self.max_task_retries = retries, task_retries
        self.recovery_timeout = recovery_timeout
        self.retry_wait = retry_wait
        self.recovery_started = None
        self.result = None
        self.last_failure_retryable = False
        self.reason = 'startup: waiting for fresh inputs and idle producer'
        # Restart never revives an execution or silently restores user RUN intent.
        with self.store.transaction():
            for task in self.tasks.values():
                if task['state'] not in ('COMPLETED', 'CANCELED'):
                    task['intent'] = 'PAUSE'
                    task['state'] = 'SUSPENDED'
                    # Unknown, unflushed active recovery time cannot be set to zero.
                    if task.get('recovery_open'):
                        task['budget_uncertain'] = True
                    self.store.save(task, 'restart_suspended')

    def _persist(self, kind):
        if self.task:
            with self.store.transaction():
                self.store.save(self.task, kind)

    def audit(self, kind, data=None):
        with self.store.transaction():
            self.store.event(kind, dict(boot_id=self.boot, execution_id=self.execution,
                attempt_id=self.attempt, epoch=self.epoch, revision=self.revision,
                monotonic_time=self.clock(), data=data or {}))

    def set_blocker(self, key, active):
        before = bool(self.blockers.get(key))
        if active:
            self.blockers[key] = True
        else:
            self.blockers.pop(key, None)
        if before != bool(active):
            if key in TASK_BLOCKERS and self.task:
                self.task['blocked_reasons'] = sorted(k for k in TASK_BLOCKERS if self.blockers.get(k))
                self._persist('task_blocker_changed')
            self.revision += 1
            if active:
                self.stop('EXECUTION', key)

    def set_authority(self, allowed):
        allowed = bool(allowed)
        if (self.authority and not allowed and self.task and not self.auto_return
                and self.task['intent'] != 'END'
                and self.task['state'] not in ('COMPLETED', 'CANCELED')):
            self.task['return_confirmation'] = True
            self._persist('authority_lost')
        self.authority = allowed
        self.set_blocker('authority', not allowed)

    def runnable(self):
        return bool(self.task and self.task['intent'] == 'RUN'
                    and self.task['state'] not in ('COMPLETED', 'CANCELED', 'FAILED')
                    and not self.task.get('return_confirmation')
                    and not self.task.get('budget_uncertain')
                    and self.authority and not self.blockers and not self.fault)

    def producer_update(self, execution, attempt, epoch, active, boot):
        self.producer = (execution, attempt, epoch, bool(active), boot, self.clock())

    def producer_idle(self):
        return bool(self.producer and self.clock() - self.producer[5] <= 1.0
                    and not self.producer[3] and self.producer[5] >= self.producer_barrier
                    and (self.producer_exit_expected is None
                         or self.producer[:3] == self.producer_exit_expected))

    def acknowledge_gate(self, boot, revision, permitted):
        self.gate_ack = (boot, revision, permitted, self.clock())

    def revoked(self):
        return bool(self.gate_ack and self.gate_ack[0] == self.boot
                    and self.gate_ack[1] >= self.permit_revision
                    and not self.gate_ack[2]
                    and self.clock() - self.gate_ack[3] <= 1.0)

    def quiet(self, whole=True):
        return (not self.records and self.producer_idle() and self.revoked()
                and (not whole or (not self.outer_pending and self.outer_terminal)))

    def exit_obstacles(self, whole=True):
        reasons = []
        if self.records:
            reasons.append('等待子动作终态: ' + ', '.join(sorted(self.records)))
        if not self.producer_idle():
            reasons.append('等待对应控制循环退出/新鲜退出心跳')
        if not self.revoked():
            reasons.append('等待门控撤权确认')
        if whole and (self.outer_pending or not self.outer_terminal):
            reasons.append('等待外层导航终态/发送确认')
        return reasons

    def resume_denials(self, task):
        reasons = []
        if task.get('budget_uncertain'):
            reasons.append('budget_uncertain')
        if not self.authority:
            reasons.append('authority')
        reasons.extend(sorted(self.blockers))
        if self.fault:
            reasons.append(self.fault)
        return list(dict.fromkeys(reasons))

    def stop(self, scope='EXECUTION', cause='stop'):
        changed = self.permitted or (self.cancel_since is None and bool(self.execution))
        if self.permitted:
            self.permitted = False
            self.permit_revision += 1
        self.frozen = True
        if self.cancel_since is None and (self.execution or self.outer_pending):
            self.cancel_since = self.clock()
            self.cancel_scope = scope
        elif scope == 'EXECUTION':
            self.cancel_scope = scope
        self.reason = cause
        if scope == 'EXECUTION':
            self.result = None
            self.finish_recovery()
        if changed:
            self.audit('output_revoked', {'scope': scope, 'cause': cause})

    def tick(self):
        if self.cancel_since is not None:
            if self.quiet(whole=self.cancel_scope == 'EXECUTION'):
                self.audit('execution_quiescent', {'scope': self.cancel_scope})
                self.cancel_since = None
                if self.cancel_scope == 'EXECUTION':
                    self.execution = self.attempt = ''
                    self.producer_exit_expected = None
                    self.revision += 1
                    if self.pending_task:
                        self.select_task(self.tasks[self.pending_task])
                        self.pending_task = None
                    self._persist('switch_activated')
            elif self.clock() - self.cancel_since > self.cancel_timeout:
                self.fault = 'cancel_timeout: missing child/result/producer/gate evidence'
                self.permitted = False
        if self.recovery_started is not None and self.task:
            if self.clock() - self.recovery_started + self.task['recovery_seconds'] > self.recovery_timeout:
                self.set_blocker('retry_exhausted', True)

    def select_task(self, task):
        for key in TASK_BLOCKERS:
            self.blockers.pop(key, None)
        self.task = task
        for key in task.get('blocked_reasons', []):
            self.blockers[key] = True

    def remember_request(self, key, result):
        self.store.remember(key, {'fingerprint': self.request_fingerprint, 'result': result})

    def manage(self, command, task_id='', spec=None, request_id='', expected=0):
        if not request_id:
            return False, 400, 'request_id required'
        prior = self.store.request(request_id)
        fingerprint_spec = spec
        if spec and spec['mode'] == 'HOLD' and spec.get('hold', {}).get('use_current_pose'):
            # The observed pose is resolved input, not part of the caller's
            # payload. Retrying one request must not capture a second anchor.
            fingerprint_spec = copy.deepcopy(spec)
            fingerprint_spec['hold']['anchor'] = None
        fingerprint = json.dumps([command, task_id, fingerprint_spec, expected],
                                 sort_keys=True, allow_nan=False)
        if prior:
            if not isinstance(prior, dict) or prior.get('fingerprint') != fingerprint:
                return False, 409, 'request_id reused with a different payload'
            return tuple(prior['result'])
        self.request_fingerprint = fingerprint
        if expected and expected != self.revision:
            return False, 409, 'revision conflict'
        selected_hold = None
        if command == 'switch' and spec and spec['mode'] == 'HOLD' and self.task:
            # Reuse task progress/budgets, but this explicit mode entry chooses
            # its anchor anew (a captured current pose or a manual map point).
            hold = (self.task if self.task['spec']['mode'] == 'HOLD' else
                    self.tasks.get(self.task.get('hold_task_id', '')))
            if not hold and self.task['spec']['mode'] == 'NAVIGATION':
                matches = [t for t in self.tasks.values() if t['spec']['mode'] == 'HOLD'
                           and t.get('navigation_task_id') == self.task['id']
                           and t['state'] not in ('COMPLETED', 'CANCELED')]
                hold = matches[-1] if matches else None
            if hold and hold['state'] not in ('COMPLETED', 'CANCELED'):
                selected_hold = copy.deepcopy(spec['hold'])
                command, task_id = 'select_task', hold['id']
        if command in ('start', 'switch'):
            if not spec or not task_id:
                return False, 400, 'task_id and spec required'
            mode = spec['mode']
            if mode in ('DOCK', 'UNDOCK', 'BERTHING'):
                return False, 501, 'docking executor unavailable'
            if mode not in ('NAVIGATION', 'HOLD', 'SURVEY'):
                return False, 400, 'unsupported mode'
            if mode == 'NAVIGATION' and not spec.get('goals'):
                return False, 400, 'empty goals'
            if mode == 'SURVEY' and not spec.get('goals') and not spec.get('survey'):
                return False, 400, 'survey requires goals or survey geometry'
            if task_id in self.tasks:
                return False, 409, 'task ID already exists; use resume_task or a new ID'
            task = dict(id=task_id, spec=copy.deepcopy(spec), intent='RUN', state='READY',
                        index=0, lap=0, completed=[], checkpoint=1, leg_retries={}, task_retries=0,
                        reservation=None, recovery_seconds=0.0, recovery_open=False,
                        budget_uncertain=False, return_confirmation=False,
                         hold_anchor=spec.get('hold', {}).get('anchor'), hold_returns=0, hold_elapsed=0.0)
            if mode == 'HOLD' and self.task:
                task['navigation_task_id'] = (self.task['id'] if self.task['spec']['mode'] == 'NAVIGATION'
                                              else self.task.get('navigation_task_id', ''))
            # Preflight completed before disturbing the active recoverable task.
            with self.store.transaction():
                if self.task and self.task['state'] not in ('COMPLETED', 'CANCELED'):
                    if mode == 'HOLD' and self.task['spec']['mode'] == 'NAVIGATION':
                        self.task['hold_task_id'] = task_id
                    self.task['state'] = 'SUSPENDED'
                    self.store.save(self.task, 'switch_suspended')
                self.store.save(task, 'task_created')
                self.remember_request(request_id, [True, 202, 'transition accepted'])
            self.tasks[task_id] = task
            self.pending_task = task_id
            if not self.execution and self.quiet():
                self.select_task(task)
                self.pending_task = None
            else:
                self.stop(cause='mode_switch')
                if self.cancel_since is None:
                    self.cancel_since = self.clock()
            self.revision += 1
            return True, 202, 'transition accepted'
        if command == 'resume_navigation':
            candidates = [task for task in self.tasks.values()
                          if task['spec']['mode'] == 'NAVIGATION'
                          and task['state'] not in ('COMPLETED', 'CANCELED')]
            if not candidates:
                return False, 409, '没有可接续的导航任务；请先新建导航任务'
            previous = self.tasks.get(self.task.get('navigation_task_id', '')) if self.task else None
            if (self.task and self.task.get('navigation_task_id') and previous not in candidates):
                return False, 409, '原导航任务已结束或不可接续；请先新建导航任务'
            target = (self.task if self.task in candidates else
                      previous if previous in candidates else candidates[-1])
            task_id = target['id']
            command = 'select_task'
        if command in ('resume_task', 'select_task'):
            target = self.tasks.get(task_id)
            if not target or target['state'] in ('COMPLETED', 'CANCELED'):
                return False, 409, 'task not resumable'
            if target.get('budget_uncertain'):
                return False, 409, 'uncertain recovery interval requires budget reconciliation or a new task'
            if selected_hold is not None:
                anchor = selected_hold.get('anchor')
                if anchor is None:
                    return False, 400, 'HOLD mode selection requires a captured anchor'
                validate_pose(anchor)
                target['hold_anchor'] = copy.deepcopy(anchor)
                target['spec']['hold']['anchor'] = copy.deepcopy(anchor)
                target['spec']['hold']['use_current_pose'] = selected_hold.get('use_current_pose', True)
            if (self.task and self.task is not target
                    and self.task['state'] not in ('COMPLETED', 'CANCELED')):
                self.task['state'] = 'SUSPENDED'
                self._persist('switch_suspended')
            selecting = command == 'select_task'
            target['intent'] = 'PAUSE' if selecting else 'RUN'
            if not selecting:
                target['return_confirmation'] = False
            target['state'] = 'PAUSED' if selecting else 'READY'
            message = ('mode selected; click Continue to execute remaining task' if selecting
                       else 'transition accepted')
            with self.store.transaction():
                self.store.save(target, 'mode_selected' if selecting else 'resume_requested')
                self.remember_request(request_id, [True, 202, message])
            self.pending_task = task_id
            self.stop(cause=command)
            if self.cancel_since is None:
                self.cancel_since = self.clock()
            self.revision += 1
            return True, 202, message
        if not self.task or (task_id and task_id != self.task['id']):
            return False, 409, 'no matching active task'
        if command == 'reset_fault':
            if not self.quiet() or self.fault.startswith('persistence'):
                return False, 409, 'cleanup evidence incomplete; persistence errors require restart'
            self.fault = ''
            for key in ('execution_failed', 'nonretryable_failure', 'hold_timeout'):
                self.blockers.pop(key, None)
            self.task['blocked_reasons'] = sorted(k for k in TASK_BLOCKERS if self.blockers.get(k))
            self._persist('operator_fault_reset')
            with self.store.transaction():
                self.remember_request(request_id, [True, 200, 'fault reset; retry budgets preserved'])
            self.revision += 1
            return True, 200, 'fault reset; retry budgets preserved'
        if command not in ('pause', 'resume', 'stop'):
            return False, 400, 'unknown command'
        if self.task['state'] in ('COMPLETED', 'CANCELED'):
            return False, 409, 'terminal task requires a new start'
        waiting = False
        if command == 'resume':
            # A late, independently confirmed terminal result can complete the
            # cleanup. Only an explicit continue clears this specific timeout.
            if self.fault.startswith('cancel_timeout:') and self.quiet():
                self.audit('cancel_timeout_resolved_by_resume')
                self.fault = ''
            denials = self.resume_denials(self.task)
            if denials:
                details = self.exit_obstacles() if self.fault else []
                return False, 409, '恢复受阻: ' + '; '.join(denials + details)
            waiting = self.cancel_since is not None and not self.quiet()
            self.task['intent'] = 'RUN'
            self.task['return_confirmation'] = False
            self.task['state'] = 'READY'
        else:
            self.stop(cause=command)
            self.task['intent'] = 'PAUSE' if command == 'pause' else 'END'
            self.task['state'] = 'PAUSED' if command == 'pause' else 'CANCELED'
            if command == 'stop':
                self.task['return_confirmation'] = False
        with self.store.transaction():
            self.store.save(self.task, command)
            result = ([True, 202, '恢复请求已接收，等待旧执行退出']
                      if command == 'resume' and waiting else [True, 200, command + ' accepted'])
            self.remember_request(request_id, result)
        self.revision += 1
        return tuple(result)

    def activate(self):
        if self.execution:
            return SUCCESS if not self.frozen else RUNNING
        if not self.runnable() or not self.quiet():
            return RUNNING
        self.execution = identity()
        self.unsent.clear()
        self.accepted.clear()
        self.registration_previous_producer.clear()
        self.producer_confirmed.clear()
        self.result = None
        self.frozen = False
        self.task['state'] = 'RUNNING'
        self._persist('execution_activated')
        return SUCCESS

    def navigation_control(self, op, execution, attempt='', epoch=0, key='', outcome=0,
                            retryable=False, goal_id=''):
        token = (execution, attempt, epoch, key)
        if op == 'not_sent':
            # The sender proves async_send_goal was never called. This is not a
            # cancel acknowledgement, and cannot retire an accepted child Goal.
            if token in self.accepted or not key or execution != self.execution:
                return FAILURE
            record = self.records.get(key)
            if record and record[:3] != token[:3]:
                return FAILURE
            self.unsent.add(token)
            if record:
                del self.records[key]
                if (key.startswith('follow_path:') and self.producer_exit_expected == token[:3]
                        and token[:3] not in self.producer_confirmed
                        and not any(k.startswith('follow_path:') and r[:3] == token[:3]
                                    for k, r in self.records.items())):
                    self.producer_exit_expected = self.registration_previous_producer.get(token[:3])
            self.audit('child_not_sent', {'key': key})
            return SUCCESS
        # terminal evidence may arrive after freeze, but may only retire its own record.
        if op == 'terminal':
            record = self.records.get(key)
            if record and record[:3] == (execution, attempt, epoch):
                del self.records[key]
                self.accepted.discard(token)
                if key.startswith('follow_path:'):
                    if outcome != 3:
                        self.producer_confirmed.add(token[:3])
                    self.producer_barrier = self.clock()
                self.audit('child_terminal', {'key': key, 'outcome': outcome})
                # A superseded FollowPath can be aborted during normal path
                # replacement. Only the current BT action reports failure;
                # terminal evidence itself does not classify business failure.
            return SUCCESS
        if op == 'accepted':
            record = self.records.get(key)
            if record and record[:3] == (execution, attempt, epoch):
                self.accepted.add(token)
                if key.startswith('follow_path:'):
                    self.producer_confirmed.add(token[:3])
                self.audit('child_accepted', {'key': key, 'goal_id': goal_id})
                return SUCCESS
            return FAILURE
        if execution != self.execution or not execution:
            return FAILURE
        if op in ('stop', 'quiet', 'retry', 'failure') and (attempt != self.attempt or epoch != self.epoch):
            return FAILURE
        if op == 'begin' and not ((not attempt and not self.attempt and epoch == 0)
                                  or (attempt == self.attempt and epoch == self.epoch)):
            return FAILURE
        if op == 'failure':
            self.last_failure_retryable = retryable
            self.stop('ATTEMPT', 'retryable_failure' if retryable else 'nonretryable_failure')
            return SUCCESS
        if op == 'stop':
            self.stop('ATTEMPT', 'navigation_recovery')
            self.result = None
            return SUCCESS
        if op == 'quiet':
            return SUCCESS if self.quiet(False) else (FAILURE if self.fault else RUNNING)
        if op == 'retry':
            if not self.task or not self.runnable() or not self.quiet(False):
                return FAILURE
            return self.reserve_retry()
        if op == 'begin':
            if not self.runnable() or self.outer_terminal:
                return FAILURE
            if self.permitted and not self.frozen:
                return SUCCESS
            if not self.quiet(False):
                return RUNNING
            reservation = self.task.get('reservation')
            if reservation and not reservation['started']:
                if self.recovery_started is None:
                    self.recovery_started = self.clock()
                    self.task['recovery_open'] = True
                    self._persist('recovery_resumed')
                if self.clock() - self.recovery_started < reservation.get('wait_remaining', self.retry_wait):
                    return RUNNING
            self.attempt = identity()
            self.epoch += 1
            self.permit_revision += 1
            self.frozen, self.permitted = False, True
            self.cancel_since = None
            self.last_failure_retryable = False
            self.audit('attempt_authorized')
            if self.task.get('reservation'):
                self.task['reservation']['started'] = True
                self._persist('retry_started')
            self.finish_recovery()
            return SUCCESS
        if op == 'register':
            if self.frozen or not self.runnable() or attempt != self.attempt or epoch != self.epoch:
                return FAILURE
            if token in self.unsent:
                return FAILURE
            if key not in self.records:
                self.records[key] = (execution, attempt, epoch, self.clock())
                if key.startswith('follow_path:'):
                    self.registration_previous_producer.setdefault(token[:3], self.producer_exit_expected)
                    self.producer_exit_expected = (execution, attempt, epoch)
                    self.producer_barrier = self.clock()
                self.audit('child_registered', {'key': key})
            return SUCCESS
        if op == 'passed':
            if (self.frozen or not self.runnable() or attempt != self.attempt or epoch != self.epoch
                    or self.task['spec'].get('policy') != 'VIA'):
                return FAILURE
            index = int(key)
            if index < self.task['index']:
                return SUCCESS if index in self.task['completed'] else FAILURE
            if index != self.task['index'] or index >= len(self.task['spec']['goals']) - 1:
                return FAILURE
            self.task['completed'].append(index)
            self.task['index'] += 1
            self.task['checkpoint'] += 1
            self.task['reservation'] = None
            self._persist('via_waypoint_committed')
            return SUCCESS
        return FAILURE

    def reserve_retry(self):
        task = self.task
        if not self.last_failure_retryable:
            self.set_blocker('nonretryable_failure', True)
            return FAILURE
        if task['reservation'] and not task['reservation']['started']:
            return SUCCESS
        leg = str(task.get('lap', 0)) + ':' + str(task['index'])
        used = task['leg_retries'].get(leg, 0)
        limit = self.max_retries
        if task['spec']['mode'] == 'HOLD':
            leg = 'hold_return:' + str(task['hold_returns'])
            used = task['leg_retries'].get(leg, 0)
            limit = min(limit, task['spec']['hold']['retries'])
        if used >= limit or task['task_retries'] >= self.max_task_retries:
            self.set_blocker('retry_exhausted', True)
            self.reason = 'navigation retry budget exhausted'
            return FAILURE
        task['leg_retries'][leg] = used + 1
        task['task_retries'] += 1
        task['reservation'] = {'id': identity(), 'started': False, 'wait_remaining': self.retry_wait}
        task['recovery_open'] = True
        self._persist('retry_reserved')
        if self.recovery_started is None:
            self.recovery_started = self.clock()
        return SUCCESS

    def finish_recovery(self):
        if self.recovery_started is not None and self.task:
            elapsed = self.clock() - self.recovery_started
            self.task['recovery_seconds'] += elapsed
            reservation = self.task.get('reservation')
            if reservation:
                reservation['wait_remaining'] = max(0.0, reservation.get('wait_remaining', self.retry_wait) - elapsed)
            self.recovery_started = None
            self.task['recovery_open'] = False
            self._persist('recovery_checkpoint')

    def commit_waypoint(self, execution):
        if execution != self.execution or self.frozen or self.result != 'SUCCEEDED':
            return FAILURE
        index = self.task['index']
        if index not in self.task['completed']:
            self.task['completed'].append(index)
            self.task['index'] += 1
            self.task['checkpoint'] += 1
            self.task['reservation'] = None
            self._persist('waypoint_committed')
        self.stop(cause='waypoint_complete')
        self.revision += 1
        return SUCCESS


def validate_pose(pose):
    values = pose['position'] + pose['orientation']
    if not pose['frame'] or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid pose/frame')
    norm = sum(v * v for v in pose['orientation'])
    if abs(norm - 1.0) > 0.02:
        raise ValueError('orientation must be a unit quaternion')
