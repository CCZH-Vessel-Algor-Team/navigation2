# USV autonomy v2

Task workflow is implemented by BehaviorTree.CPP in `usv_autonomy_bt/trees`.
This package supplies single-writer lifecycle/checkpoint primitives, an
independent output-gate process, a read-only FCU authority adapter and maritime
history evaluation. It does not embed navigation workflow in a dispatcher.

## Processes and interfaces

| Process | Input / API | Output |
| --- | --- | --- |
| `mission` | `autonomy/manage` (`ManageMission`), `manage_task` (`ManageTask` Action), task-BT and Nav2 contract services | `autonomy/state`, `autonomy/permit`, `task_progress`, native navigation Actions |
| `task_tree` (`usv_autonomy_bt`) | state snapshots, `trees/task.xml` | bounded primitive calls and `autonomy/bt_heartbeat` |
| `motion_gate` | identity-bound commands, permit, FCU authority, direct odometry / raw radar sector / point cloud / emergency inputs, BT heartbeat | `control/actuator_command`, `autonomy/safety`, `autonomy/gate_state` |
| `regular_safety` | odometry pose/covariance and configured `health/input` sources | `autonomy/regular_safety`, with stable recovery and capability/speed restrictions |
| `maritime_rules` | complete stamped `SituationReportArray` | aggregate `autonomy/maritime` decision |
| `authority_adapter` | `mavros/state` | `control/authority`; never arms or switches FCU mode |

All relative names are scoped by vessel namespace. TF in the launch file is
remapped to the vessel's `tf`/`tf_static`. Radar is transformed into the base
frame at observation time; missing TF, nonfinite data and oversized input make
observations invalid. The raw `RadarSector` is the primary independent input
because it publishes continuously (an empty sector means clear), while the
filtered point cloud only appears when there are returns. `require_radar=false` is for tests/deployments with
an explicitly different independent protection input, not the normal profile.

## Launch

After the v2 overlay is built into the workspace `install/` prefix and sourced:

```bash
ros2 launch usv_autonomy autonomy.launch.py \
  namespace:=usv_1 nav2_params:=/absolute/path/to/radar_nav2_param.yaml
```

The launch owns planner, controller, navigator and Wait-only behavior server.
Localization, map/keepout data, TS services, odometry, radar, maritime situation
assessment and FCU state are deployment inputs. Do not also launch the old
Nav2 stack or a bare-Twist actuator bridge. V2 bypasses the old smoother chain;
identity and epoch reach the command producer and final gate directly.

The current simulator entry is
`ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py`.
CCS enables v2 and the managed RViz panel by default, preserving the existing
three-vision/mmwave scene, localization/map readiness and PID parameters.
Its optional simulation controls begin in manual authority; click the panel's
simulation-autonomy button before starting a task. Disabling
`enable_autonomy_test_controls` requires an external authority source.
Generic `nav2_sim_full_bringup` remains opt-in with `autonomy_v2:=true`.
The Gazebo thrust bridge is not a real-FCU neutral-thrust implementation.

The RViz panel and waypoint tool are documented in
[`nav2_rviz_plugins/README.md`](../nav2_rviz_plugins/README.md).
Simulation test inputs use separate blockers and a startup-read-only enable
flag; they never publish over the real maritime or regular-safety decision.

## Management

Read `autonomy/state` to obtain `boot_id` and optionally `revision`. Every
`ManageMission`/`ManageTask` request must name that boot; `request_id` is an
idempotency key and cannot be reused with a different payload. Map-frame
poses require unit quaternions. Example (replace `BOOT`):

```bash
ros2 service call /usv_1/autonomy/manage usv_interfaces/srv/ManageMission \
"{command: start, request_id: route-001-start, expected_boot_id: BOOT,
  task_id: route-001, mode: NAVIGATION, waypoint_policy: REQUIRED,
  goals: [{header: {frame_id: map}, pose: {position: {x: 20.0, y: 0.0},
    orientation: {w: 1.0}}}]}"
```

- `start`/`switch`: 202 acknowledges an accepted transition, not physical task
  completion. Watch the state and `task_progress` for activation/completion.
- `pause`/`resume`: explicit user intent. Automatic risk/health recovery never
  writes RUN intent over a manual pause.
  Resume returns 409 with concrete admission/exit obstacles, or 202 while an
  old execution is still exiting. A resolved cancel timeout can only be cleared
  by explicit resume after complete exit evidence; other faults remain latched.
- `stop`: terminal cancellation; resume is refused. A new run needs a new ID.
- `resume_task`: selects a stored suspended task, preserving original goals,
  progress, HOLD anchor and budget. Restarted tasks require this explicit step.
- `resume_navigation`: switches to the navigation route saved when entering
  HOLD in PAUSE, preserving goals/progress/budgets. Explicit `resume` is required
  before it executes. Falls back to the latest resumable
  navigation task for older records; refuses 409 if no route exists. Creating
  a new route remains an explicit start, not an effect of the mode button.
- Switching back to an existing HOLD associated with that navigation task selects
  it in PAUSE, preserving options, return count, elapsed time and budget, but
  explicitly updates the anchor from this mode-entry request. `use_current_pose`
  captures a fresh current pose on every entry; false selects a manual map point.
  Continue and stored-task resume do not recapture the anchor. Retransmitting one
  request ID cannot capture a different anchor.
- `reset_fault`: only succeeds after all execution/gate/producer evidence is
  quiet. It does not replenish budgets or resolve uncertain persistence.
- DOCK/UNDOCK/BERTHING requests return 501 before disturbing the active task.
- `SURVEY` runs FollowReferencePath lawnmower surveying (default geometry from
  `survey.*` parameters, or `goals` as an explicit reference path). Wait timeout /
  stuck / `MANUAL_HANDOVER` latches `maritime_handover` so `MaritimeManualTakeover`
  preempts the survey branch. Events append to
  `~/.ros/survey_journals/<run_id>.jsonl` with a `*_summary.json` skip summary.

An independent child Action GetResult watcher continues collecting terminal
results after BT halt, using exact UUID and execution/attempt/epoch identities.
Unknown results and cancellation acknowledgements are not exit evidence.
If registration acknowledgement times out before `async_send_goal` was called,
the sender reports exact `not_sent` evidence; an accepted Goal cannot be retired
this way, and a tombstone rejects any delayed registration of that same send.
Navigation admission also requires a fresh ACTIVE `bt_navigator/get_state`
response (`navigation_backend` blocker while unavailable).

Regional HOLD returns stop as soon as fresh position is within the inner
radius; they do not wait for Nav2's point/yaw goal checker. Each completed
return revokes/cancels the old execution and resumes region monitoring. The
anchor and cumulative retry budget persist; a normal return is not a retry.

`autonomy/hold_region` publishes a transient-local MarkerArray at 5 Hz: green inner
circle, orange outer circle, anchor and radius text. It uses the selected task's
saved anchor/options, remains visible while paused/suspended and deletes its four
markers when leaving HOLD or ending the task. CCS adds its RViz display automatically.

The VO planner throws a typed `PlannerTransientException` for explicitly missing/
stale COLREGS snapshots and service timeouts. The planner server returns
`usv_retryable=true`; navigation stops, proves exit and uses bounded Wait recovery.
Malformed or mismatched responses and inescapable decisions remain nonretryable.

`REQUIRED` navigates and confirms goals individually. `VIA` uses native
NavigateThroughPoses with transactional passed-goal commits before filtering;
the final goal is always explicitly confirmed. `via_radius` defaults to 8 m.
Looping stores a lap number and preserves task-level retry totals. Speeds are
nonnegative limits; zero/empty uses configured safe speed.

`ManageTask` and the `ExecuteAutoTask` compatibility adapter convert geographic
waypoints through the actual `fromLL` localization service. Conversion is
bounded and never invents a second map origin. The compatibility Action's
result acknowledges management (202/200); physical completion is reported on
`task_progress`. Use the v2 management API when boot fencing is required.

## HOLD and safety

Regular safety and independent emergency protection publish separate
constraints. The task service and final gate merge them by capability
intersection and minimum speed, not by latest-writer-wins. Localization
covariance/validity/age is monitored by default. Additional GNSS, perception
or environment monitors publish `NavigationInputState` on `health/input`;
enable the comma-separated source names in `required_health_sources`.
A missing/invalid/stale enabled source inhibits navigation and HOLD until
stable recovery; source status enums are never guessed from raw device bytes.

HOLD is regional: inside the outer boundary use neutral thrust, outside it
plan a return to the saved anchor, then stop. Inner radius must cover the
configured navigation goal tolerance. Return timeout, per-return and task
budgets bound failure recovery; duration counts active HOLD time. It is not
continuous dynamic positioning or a hard geofence.

Maritime waiting always means neutral propulsion and permitted drift. Waiting
targets persist across rule-process restart; missing targets, invalid CPA,
invalid assessment and ID changes cannot silently clear them. A passed,
separating target must remain beyond the configured release range for the
clear interval. There is no speculative thrust pulse to test resumption.
After release, normal planning is rerun from actual position.

Final `ActuatorCommand` distinguishes DISABLED, NEUTRAL and VELOCITY_CONTROL.
NEUTRAL must bypass speed/yaw PID; DISABLED means release the autonomous FCU
channel. The included Gazebo bridge writes simulated thrust. Actual FCU input
selection, neutral protocol and hardware watchdog must be implemented and
verified against the vessel interface before real actuation is connected.

## Persistence and failure recovery

SQLite WAL/full-synchronous transactions save tasks, completion checkpoints,
retry reservations and an execution journal. DB and maritime-checkpoint paths
must be distinct for each vessel. The mission DB has a single-writer lock.
Rule waiting IDs are atomically stored before the decision is published.

Cancel acknowledgement is not exit acknowledgement. Pending child sends,
outer result, current producer loop exit and gate revocation are checked
separately. Cancel timeout stays inhibited. If terminal evidence cannot be
recovered, keep the gate restricted, stop/restart the affected Nav2 stack,
then restart the coordinator and explicitly resume the stored task only after
fresh idle/authority/safety evidence. Never clear unresolved records by timer.

An open recovery interval at an unclean restart is marked budget-uncertain;
it cannot silently become zero. A new explicit task or a separately reviewed
budget-reconciliation procedure is needed. `reset_fault` does not hide it.

Workspace scripts `build_autonomy_v2.sh`, `test_autonomy_v2.sh` and
`report_autonomy_v2.py` build, verify and export the journal. The workspace's
`docs/usv_behavior_tree_design/bt-status/实施记录.md` records exact validation
and deployment limitations for this implementation.
