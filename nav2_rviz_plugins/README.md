# USV BT v2 RViz controls

This package retains standard Nav2 plugins and adds:

- `nav2_rviz_plugins/USV Autonomy`: managed task panel, displayed as **USV BT v2**.
- `nav2_rviz_plugins/USV Goal`: **USV Test Goal**, shortcut **U**, map position/heading collection.

The CCS simulation entry generates a configuration from its existing
three-vision/mmwave RViz template, preserving its panels, tools, views and saved
window layout. USV BT v2 is an additional tab next to the original Navigation 2
panel. With `Managed Task: true`, the original G goal tool, waypoint controls
and Pause/Resume/Cancel use ManageMission rather than direct navigation Actions.
The additional U goal tool publishes `/<vessel>/autonomy/goal_pose` as a draft.

## Usage

1. Wait for the task state and services; in CCS click **仿真自主权** (simulation autonomy).
2. Use the U tool to click/drag waypoints, or enter X/Y/yaw-degrees/speed in the panel.
3. Select REQUIRED/VIA and optional looping, then create a navigation task.
4. The always-visible mode bar contains peer **NAVIGATION / HOLD / DOCK / UNDOCK**
   buttons. NAVIGATION selects the saved source route in PAUSE (progress/budgets intact);
   click Continue to execute. HOLD uses the mode tab's anchor source: current pose
   (captured anew on every entry), or manually entered map-frame X/Y in metres.
   Switching back selects the existing HOLD in PAUSE and updates its anchor while
   preserving options/progress/budgets. The Apply Anchor button also works in HOLD.
   Click Continue to resume monitoring/returning. Creating a
   new route is a separate button on the waypoint tab.
5. Pause/resume/stop are on the common tab. Enter a suspended task ID on the
   mode-parameters tab to resume a specific stored task. DOCK/UNDOCK report 501
   while their executor is unavailable and leave the current mode selected.

HOLD continuously monitors its saved anchor: crossing the outer radius starts
a return, entering the inner radius cancels that return and revokes motion, and
monitoring continues for subsequent drifts. Normal returns do not spend retry
budgets. Duration zero means indefinite HOLD until stopped/switched.

CCS also adds a MarkerArray display named `HOLD Region (inner / outer)` on
`/<vessel>/autonomy/hold_region`: green inner radius, orange outer radius and saved
anchor. Paused/suspended HOLD keeps the reference; leaving HOLD or ending deletes it.

The panel fills the current boot/revision and a unique request ID. Pending
requests are serialized. Code 202 is management acceptance, not task completion;
watch the state and final gate intent. A stale snapshot disables management.
Resume reports 409 with a specific reason when blocked, or 202 while waiting
for the old execution to exit. Only complete child/result/producer/gate evidence
allows a resolved cancellation timeout to be cleared by explicit resume.
Changing vessel namespace reconnects and clears local drafts; also update the
tool's Topic property if selecting another active vessel.

CCS-only buttons call `autonomy/test/{authority,maritime_wait,emergency,localization_failure}`
SetBool services. Clearing a test wait does not clear real maritime risk.
Clearing the emergency input does not reset its protection latch; use the
separate protection-reset button after stable clear. These controls are hidden
when the configuration's Simulation Tests field is false and unavailable
when their servers are absent.

## Verification

Build with `-DUSV_PANEL_TESTING=ON` (enabled by the workspace v2 build script), then:

```bash
QT_QPA_PLATFORM=offscreen ros2 run nav2_rviz_plugins test_usv_autonomy_panel
```

The tests load the actual plugin and click real widgets against ROS services,
checking request contents, lifecycle/mode controls, test services, stale state,
boot changes, invalid drafts and the original Navigation 2 managed adapter.
Actual CCS G-tool picking and both panels' pause/resume controls are also
checked interactively; evidence is recorded in the workspace implementation log.
