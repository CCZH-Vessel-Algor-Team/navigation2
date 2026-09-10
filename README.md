# navigation2 — COLREGS / Maritime Extensions

This repository extends the official [navigation2](https://github.com/ros-navigation/navigation2) `humble` branch with COLREGS-compliant maritime navigation capabilities: velocity-obstacle RRT* planning, persistent skeleton-based replanning, target-ship state management, and LOS/ALOS guidance controllers.

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
git clone -b feat/rrt-star-informed-humble \
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

### Skeleton planner parameters

| Group | Parameters | Default |
|---|---|---|
| Search | `step_size` `goal_bias` `eta` `goal_tolerance` | 4.0 / 0.1 / 50.0 / 2.0 |
| Capacity | `node_limit` `path_limit` `near_limit` `connector_limit` `recovery_near_limit` | 1024 / 256 / 16 / 128 / 16 |
| Budget | `global_iterations` `local_iterations` `refine_iterations` `reuse_iterations` `max_work` `time_limit` | 2400 / 600 / 200 / 64 / 12M / 0 |
| Behaviour | `allow_recovery` `allow_skip` `switch_margin` `safety_dist` `cost_weight` | true / true / 0.03 / 1.5 / 0.3 |

### COLREGS service chain (VO plugins only)

`VORRTStarPlanner` and `VOSkeletonPlanner` query two services from `nav2_colregs_ts_manager`. If either service times out (1 s), the planner falls back to plain RRT* from the robot pose — COLREGS avoidance is skipped but planning still succeeds. Decision is made against the **primary threat only** (minimum TCPA); multi-vessel simultaneous avoidance is not yet supported.

- `/get_avoidance_point` (`nav2_colregs_msgs/srv/GetAvoidancePoint`) — VO collision-cone safe heading and avoidance point
- `/get_barrier_lines` (`nav2_colregs_msgs/srv/GetBarrierLines`) — barrier segments around the primary threat

Launch the TS subsystem (standalone nodes):

```bash
ros2 launch nav2_colregs_bringup ts_subsystem_launch.py \
  tracked_ship_topic:=/your_ts_topic \
  robot_base_frame:=base_link \
  odom_topic:=odom
```



### TS conservative-avoidance tuning

The avoidance point conservatism is governed by three parameters on `ts_state_manager` / `avoidance_point_node` (set in `ts_subsystem_launch.py` or your params file):

- `safety_factor` — multiplies the combined keep-out radius `(os_radius + ts_radius)`: controls both the DCPA threat-trigger threshold and the avoidance-point placement distance
- `os_radius` — own-ship safety radius; the only parameter that widens the collision cone (lateral clearance)
- `tcpa_horizon` — time window for threat qualification

### Controller registration

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_colregs_alos_controller::ALOSController"
```

Reference parameter sets: `nav2_colregs_bringup/params/*.yaml` (merge the sections you need into your own Nav2 params — none of them is a standalone drop-in file).

## Tests

Each added package ships GTest regressions (supercover traversal, capacity recycling, recovery invariants, budget-interruption consistency, resolution-change safety distance, shortcut certification, etc.):

```bash
colcon test --packages-select nav2_colregs_vo_skeleton_planner nav2_skeleton_planner
colcon test-result --verbose
```
