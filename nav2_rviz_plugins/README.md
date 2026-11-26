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

1. Wait for task state/services and FCU authority. With `authority_source=cc_state`,
   autonomy is observed from the actual connected/armed OFFBOARD/GUIDED FCU;
   only the explicit `authority_source=simulation` profile permits **仿真自主权**.
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
SetBool services. The test page and all button labels remain visible. With real FCU
authority, **仿真自主权/仿真人工接管** are disabled with an explanation, and their
service and publisher do not exist in the fault-injection node. Other fault
buttons can remain enabled with `enable_autonomy_test_controls=true`.
The RViz `Simulation Tests` field enables fault injection; the separate
`Simulation Authority` field additionally permits synthetic authority and defaults
to false, including for older saved configs that lack the field.
Clearing a test wait does not clear real maritime risk.
Clearing the emergency input does not reset its protection latch. The
separate `autonomy/test_emergency` input is OR-ed with the real `safety/estop`, so
clearing the test cannot mask a real emergency. Use the
separate protection-reset button after stable clear. When tests are disabled or
servers absent, controls stay visible but disabled. Task/mode controls and the
original window/dock layout are unchanged.

Chinese labels require a CJK font in the RViz container (`fonts-noto-cjk`, already
listed in the project Dockerfile). The panel prefers Noto Sans CJK SC locally,
preserving the existing font size/DPI. Older running containers may need
`apt-get update && apt-get install -y fonts-noto-cjk`; restart RViz to refresh its
font database. Installing fonts does not enable disabled actions.

## Verification

Build with `-DUSV_PANEL_TESTING=ON` (enabled by the workspace v2 build script), then:

```bash
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 QT_QPA_PLATFORM=offscreen \
  ros2 run nav2_rviz_plugins test_usv_autonomy_panel
```

The tests load the actual plugin and click real widgets against ROS services,
checking request contents, lifecycle/mode controls, test services, stale state,
boot changes, invalid drafts and the original Navigation 2 managed adapter.
Actual CCS G-tool picking and both panels' pause/resume controls are also
checked interactively; evidence is recorded in the workspace implementation log.
