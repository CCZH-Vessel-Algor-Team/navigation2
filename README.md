# navigation2 — COLREGS / Maritime Extensions

This repository extends the official [navigation2](https://github.com/ros-navigation/navigation2) `humble` branch with experimental COLREGS-oriented maritime navigation capabilities: velocity-obstacle RRT* planning, persistent skeleton-based replanning, target-ship state management, and LOS/ALOS guidance controllers.

For upstream Nav2 documentation, see [docs.nav2.org](https://docs.nav2.org/). This README covers only the packages added in this fork.

## Added Packages

### Global Planner Plugins

| Plugin ID | Package | Summary |
|---|---|---|
| `nav2_rrt_star_planner/RRTStarPlanner` | `nav2_rrt_star_planner` | Standard RRT* with corrected optimize phase, subtree cost propagation, goal competition on total path cost, and informed ellipsoidal sampling. |
| `nav2_colregs_vo_rrt_star_planner/VORRTStarPlanner` | `nav2_colregs_vo_rrt_star_planner` | COLREGS VO-RRT*: two-segment plan (deterministic leg to avoidance point + informed RRT* to goal), barrier-line collision constraints, decision marker visualization. |
| `nav2_skeleton_planner/SkeletonRRTPlanner` | `nav2_skeleton_planner` | Standalone skeleton RRT* — persistent goal-rooted tree with skeleton/prefix separation. No COLREGS service dependencies; query is the passed-in start pose. |
| `nav2_colregs_vo_skeleton_planner/VOSkeletonPlanner` | `nav2_colregs_vo_skeleton_planner` | COLREGS skeleton planner. The core search (Space, BoundedInformedRRT, SkeletonPlanner) is exported from this package and reused by `nav2_skeleton_planner`. |

**Skeleton planner mechanism** (shared by both skeleton plugins):

- Goal-rooted persistent tree with bounded node capacity and leaf recycling
- Skeleton (anchor chain) / prefix (query-to-active-anchor) separation; 3% switch margin (hysteresis locks the corridor)
- Recovery ladder: reanchor → direct connect → skip-ahead → tree near-connect → global recovery
- Random ancestor shortcuts (equal-cost non-increasing reparenting, gated by cost certification)
- Per-stage atomic adoption — budget interruption never leaves inconsistent state
- Per-query costmap snapshot under mutex; world-revision-driven incremental tree pruning with cost refresh

**RRT* correctness fixes** (apply to `RRTStarPlanner` and `VORRTStarPlanner`):

- Optimize phase: `iter >= max_iterations_` was unreachable — the post-first-solution refinement loop never ran
- Rewire: subtree `cost_from_root` was not propagated after re-parenting
- Goal competition: compared bare root costs without the final candidate→goal edge
- Performance: rate-limited LOS scans with centerline pre-filter, incremental children adjacency

### Controller Plugins

| Plugin ID | Package | Summary |
|---|---|---|
| `nav2_colregs_los_controller::LOSController` | `nav2_colregs_los_controller` | Line-of-sight guidance controller. |
| `nav2_colregs_alos_controller::ALOSController` | `nav2_colregs_alos_controller` | Adaptive LOS (Fossen 2023) with sideslip estimation (β̂). |

### Supporting Components

| Component | Package | Summary |
|---|---|---|
| TS State Manager | `nav2_colregs_ts_manager` | `ts_state_manager` (TS snapshots, CPA/TCPA, collision cones), `avoidance_point_node`, `barrier_node`. Also embeddable as `colregs_ts_state` lifecycle child. |
| TS Projection Layer | `nav2_colregs_ts_projection_layer` | Costmap layer projecting TS positions as occupied regions. |
| Vector Object Server | `nav2_colregs_vector_object_server` | RViz-based target-ship placement. |
| Costmap Layers | `nav2_colregs_costmap_layers` | COLREGS-specific costmap layer plugins. |
| Maritime Situation Monitor | `nav2_maritime_situation_monitor` | Standalone CPA/TCPA / encounter classification reporting. |
| Messages | `nav2_colregs_msgs` | `TrackedShip`/`TrackedShipList` messages; `GetAvoidancePoint`, `GetBarrierLines`, `GetPrimaryThreat` services. |

## Build

Prerequisites: ROS 2 Humble (apt or RoboStack), colcon, and the standard Nav2 dependencies.

**Important**: this repo is a full fork of navigation2. Do NOT clone it into a workspace that already contains upstream navigation2 packages (duplicate package names). Either use a clean workspace, or source upstream Nav2 as an underlay and build this fork as an overlay.

```bash
# clean workspace
mkdir -p ~/colregs_ws/src && cd ~/colregs_ws/src
git clone -b fix/parameter-contracts \
  git@github.com:CCZH-Vessel-Algor-Team/navigation2.git

cd ~/colregs_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

To build only the planner packages (skips controllers, costmap layers, sim-related packages):

```bash
colcon build --symlink-install \
  --packages-up-to nav2_skeleton_planner nav2_colregs_vo_skeleton_planner
```

Note: `--packages-up-to` on the skeleton planners pulls in the core search plus `nav2_colregs_msgs` and `nav2_colregs_ts_manager` (service definitions). It does NOT build the LOS/ALOS controllers, TS projection layer, or vector object server — add those explicitly if needed.

## Integration Guide

This repository does not ship a standalone simulation launch. Integrate the plugins into your own Nav2 bringup as follows.

### Register a planner plugin

In your `planner_server` parameters (merge the relevant sections from `nav2_colregs_bringup/params/nav2_colregs_params_humble_minimal.yaml` into your own params):

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["VOSkeleton"]
    VOSkeleton:
      plugin: "nav2_colregs_vo_skeleton_planner/VOSkeletonPlanner"
      step_size: 4.0
      goal_bias: 0.1
      eta: 50.0
      safety_dist: 1.5
      cost_weight: 0.3
      goal_tolerance: 2.0
      reuse_iterations: 64
      switch_margin: 0.03
      # ... full parameter list below
```

Point your BT XML at the registered planner and configure bt_navigator to load it:

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="VOSkeleton"/>
```

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "/path/to/your_tree.xml"
```

### NavigateThroughPoses with VORRTStar

The standard Humble `planner_server` calls `createPlan(start, goal)` for each segment. `VORRTStarPlanner` compares each segment start with the live robot pose from `Costmap2DROS`. It calls the COLREGS services only when that distance is within `colregs_anchor_max_dist` (default **3.0 m**, finite and nonnegative, startup-only). More distant starts use plain informed RRT* without barrier constraints. If the live pose cannot be obtained, planning fails rather than silently skipping COLREGS.

This is a distance-based gate, not a segment-index check: a later waypoint within the threshold can also enable COLREGS. `configure()` retains the `Costmap2DROS` object until cleanup; this fixes a null-pointer crash on the first planning request, including ordinary single-goal navigation.

Example planner settings (the iteration budgets are tuned USV values, not plugin defaults):

```yaml
planner_server:
  ros__parameters:
    planner_plugins: [VORRTStar]
    VORRTStar:
      plugin: "nav2_colregs_vo_rrt_star_planner::VORRTStarPlanner"
      colregs_anchor_max_dist: 3.0
      max_iterations: 1500
      max_optimize_iters: 500
      use_informed_sampling: true
```

The iteration budget is **per segment**, not per NavigateThroughPoses request or a wall-clock deadline. Once a solution is found, the loop can extend to `max_iterations + max_optimize_iters`. The example reduces the previous USV budget of 2000 + 2000 to 1500 + 500; each COLREGS service can still add up to 1 s of waiting.

Bringup requirements:

- Set `bt_navigator.default_nav_through_poses_bt_xml` to a tree using `ComputePathThroughPoses` with `planner_id="VORRTStar"`.
- Give `RemovePassedGoals` explicit frames matching the vehicle, e.g. `global_frame="map" robot_base_frame="usv_1/base_link"`. Its default `base_link` does not inherit the navigator's parameter.
- For the standard planner server, use `global_costmap/clear_entirely_global_costmap` in recovery nodes.
- Add `nav2_rviz_plugins/GoalTool` to the RViz Tools list. Select **Waypoint / Nav Through Poses Mode**, collect poses using **Nav2 Goal**, then click **Start Nav Through Poses**. The ordinary **2D Goal Pose** tool publishes a single goal and does not populate the panel's waypoint list.

The tested local companion setup in `USV_Simulation` (`feat/colregs-local-planner-bringup`) uses `usv_sim_full/config/navigate_through_poses_vorrt_star.xml` and the corresponding `radar_nav2_param.yaml` / `three_vision_one_mmwave.rviz` configuration. Those simulation changes are maintained separately from this Nav2 repository. Reload the bringup after changing its default tree or RViz configuration.

### Skeleton planner parameters

| Group | Parameters | Default |
|---|---|---|
| Search | `step_size` `goal_bias` `eta` `goal_tolerance` | 4.0 / 0.1 / 50.0 / 2.0 |
| Capacity | `node_limit` `path_limit` `near_limit` `connector_limit` `recovery_near_limit` | 1024 / 256 / 16 / 128 / 16 |
| Budget | `global_iterations` `local_iterations` `refine_iterations` `reuse_iterations` `max_work` `time_limit` | 2400 / 600 / 200 / 64 / 12M / 0 |
| Behaviour | `allow_recovery` `allow_skip` `switch_margin` `safety_dist` `cost_weight` | true / true / 0.03 / 1.5 / 0.3 |

### COLREGS service chain (VO plugins only)

`VORRTStarPlanner` and `VOSkeletonPlanner` query two services from `nav2_colregs_ts_manager`. Only an explicit **NO_THREAT** result permits ordinary planning for an anchored query. Infeasible directions, missing/stale state, either service timing out (1 s), or malformed/mismatched responses fail the planning action instead of bypassing COLREGS. Heading candidates are checked against all snapshot targets; the primary threat (minimum TCPA) still determines the avoidance-point extension and barrier.

- `/get_avoidance_point` (`nav2_colregs_msgs/srv/GetAvoidancePoint`) — VO collision-cone safe heading and avoidance point
- `/get_barrier_lines` (`nav2_colregs_msgs/srv/GetBarrierLines`) — barrier segments around the primary threat

The services share a snapshot UUID and explicit frame/status contract. Rebuild/restart producers and consumers together after this interface update. Consumers must inspect `status`; `has_feasible_angle=false` alone does not distinguish no threat from an invalid or infeasible decision.

**Safety scope:** radius inflation certifies candidate headings for the initial straight VO leg under the stated constant-speed model. The later AP-to-goal search uses the costmap and static U barriers; it does **not** guarantee the same inflated TS clearance along the whole returned path. This gap was independently reviewed and reproduced even for a stationary TS. Full-trajectory clearance remains a separate required enhancement.

**Open-water encounter scope:** both VO planners prepend the initial OS-to-AP leg without checking current costmap occupancy. A moving ship's current occupied cells do not veto a future-safe VO heading. Static-obstacle handling along that leg is outside this stage and is reserved for future BT/controller collision integration. AP-to-goal search, ordinary no-threat planning and generic endpoint validation retain their costmap constraints.

Launch the TS subsystem (standalone nodes):

```bash
ros2 launch nav2_colregs_bringup ts_subsystem_launch.py \
  tracked_ship_topic:=/your_ts_topic \
  robot_base_frame:=base_link \
  odom_topic:=odom
```



### TS conservative-avoidance tuning

Use `ts_subsystem_launch.py ts_params_file:=/path/to/ts_subsystem.yaml`; the default is `nav2_colregs_bringup/params/ts_subsystem.yaml`. The parent Nav2 `params_file` is independent. Algorithm values come from YAML; launch overrides only simulation-time and input/base-frame wiring.

- **Both nodes** interpret `safety_factor >= 1` as radius inflation: the effective collision radius is `safety_factor * (os_radius + ts_radius)`. Manager uses its factor for detection and diagnostic cones; avoidance uses its factor to validate the actual selected heading. Current YAML settings use physical radii 5m, manager factor 3 (30m detection domain), avoidance factor 1.5 (15m heading clearance), and a manager `tcpa_horizon` of 40s.
- **Avoidance node** `point_extension_distance` independently sets point range to `distance(OS,TS) + point_extension_distance` (default 20m). Changing safety_factor changes angular conservatism, not this radial extension. Inside/on the avoidance node's inflated domain, no heading satisfies the strict clearance constraint, so planning fails; it does not silently shrink back to physical radii.
- Manager/barrier application parameters and skeleton planner settings are startup-only and read-only. Avoidance parameters are validated and used on the next successful service call. RRT numeric/bool updates are validated as a batch and applied from committed values at the next planning boundary, including updates made while inactive.
- RRT `tolerance` and skeleton `prune_period` are deprecated, ignored compatibility parameters with startup warnings; remove them from configurations.

Parameter descriptions are available through `ros2 param describe`. Changing a startup-only setting requires restarting the node with the updated YAML; accepted dynamic settings take effect at the decision/planning boundary described above.

### Controller registration

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_colregs_alos_controller::ALOSController"
```

Reference parameter sets: `nav2_colregs_bringup/params/*.yaml` (merge the sections you need into your own Nav2 params — none of them is a standalone drop-in file).

### Known Limitations

- **Skeleton planners** (`SkeletonRRTPlanner`, `VOSkeletonPlanner`) do not support NavigateThroughPoses: the persistent goal-rooted tree anchors to a single final goal, and per-segment calls would repeatedly rebuild it, defeating the warm-start mechanism.
- **`VORRTStarPlanner` under NavigateThroughPoses** uses the start-to-live-pose distance gate described above; it does not guarantee that only segment zero uses COLREGS.
- **VO motion model** assumes constant speeds and instantaneous heading changes. Maneuver persistence and safe resumption of the goal course across controller braking/turning are not yet implemented as a stateful encounter policy.

## Tests

Each added package ships GTest regressions (supercover traversal, capacity recycling, recovery invariants, budget-interruption consistency, resolution-change safety distance, shortcut certification, etc.):

```bash
colcon test --packages-select nav2_colregs_vo_skeleton_planner nav2_skeleton_planner
colcon test-result --verbose
```

For the VORRTStar planner, build the plugin and standard planner server, source the workspace, then run:

```bash
colcon build --symlink-install --packages-up-to nav2_colregs_vo_rrt_star_planner nav2_planner
source install/setup.bash
colcon test --packages-select nav2_colregs_vo_rrt_star_planner
colcon test-result --test-result-base build/nav2_colregs_vo_rrt_star_planner --verbose
```

Coverage includes core and anchor-distance GTests plus `test_planner_actions.py`: a real planner-server subprocess with TF, costmap, counted mock COLREGS services, and parameter/lifecycle regressions for all four planners. It uses ROS domain 91 by default (override with `NTP_TEST_ROS_DOMAIN_ID`). The bringup tests exercise the real TS nodes, parameter loading, radius inflation, snapshot/failure handling, first-leg occupancy acceptance and AP-to-goal obstacle rejection. These tests use synthetic sensor inputs and do not replace full vessel-control trials.

After building and sourcing the workspace, run the node/action suites with:

```bash
python3 -m pytest -v \
  nav2_colregs_bringup/test/test_ts_decisions.py \
  nav2_colregs_bringup/test/test_ts_parameter_contract.py \
  nav2_colregs_vo_rrt_star_planner/test/test_planner_actions.py
```

Run the command from this repository's root; the TS suites use ROS domains 93 and 92 by default.
