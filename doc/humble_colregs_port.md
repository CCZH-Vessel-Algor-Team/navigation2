# Humble COLREGS Port Status

This branch ports the COLREGS packages from the Jazzy development branch onto
the upstream Humble baseline. The Humble baseline is tracked separately so it
can be rebased or fast-forwarded with official Humble updates.

## Branches

- Baseline: `humble` tracking `origin/humble`
- Port branch: `feat/colregs-humble`

## Ported and Build-Verified Packages

Verified on the Humble apt test machine at `192.168.1.54`:

- `nav2_colregs_msgs`
- `nav2_rrt_star_planner`
- `nav2_colregs_ts_manager`
- `nav2_colregs_vo_rrt_star_planner`
- `nav2_colregs_costmap_layers`
- `nav2_colregs_vector_object_server`
- `nav2_colregs_los_controller`
- `nav2_colregs_alos_controller`
- `nav2_colregs_bringup` with params and behavior tree XML only

Focused build command:

```bash
cd ~/navigation2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select \
  nav2_colregs_msgs \
  nav2_rrt_star_planner \
  nav2_colregs_ts_manager \
  nav2_colregs_vo_rrt_star_planner \
  nav2_colregs_costmap_layers \
  nav2_colregs_vector_object_server \
  nav2_colregs_los_controller \
  nav2_colregs_alos_controller \
  nav2_colregs_bringup
```

Full workspace builds may fail at `nav2_system_tests` if Gazebo test packages
such as `gazebo_ros_pkgs` are not installed. That failure is outside the COLREGS
plugin port scope.

## Humble Compatibility Changes

- Humble `nav2_core::GlobalPlanner::createPlan()` does not take the Jazzy
  `cancel_checker` argument. The RRT* and VO-RRT* planners use the Humble
  two-argument signature.
- Humble `nav2_core/exceptions.hpp` exposes `PlannerException`; Jazzy-specific
  planner/controller exception subclasses were replaced with `PlannerException`.
- Humble RPP does not expose the Jazzy `PathHandler` helper header used by the
  COLREGS LOS/ALOS controllers. The Humble controllers now store the global plan
  internally and transform/prune it using the Humble RPP transform pattern.
- `nav2_colregs_vector_object_server` replaces Jazzy-only
  `declare_or_get_parameter()` calls with a package-local Humble-compatible
  declaration/get helper, uses `create_wall_timer()`, and links `uuid` explicitly.

## Intentionally Not Ported

- Full Gazebo / TS simulation launch files, worlds, models, scripts, bridge
  configs, maps, and RViz assets.
- Full keepout/vector-object validation launch chain.
- `nav2_colregs_local_path_bt_nodes` and `nav2_colregs_local_path_behavior`.

`nav2_colregs_local_path_bt_nodes` was a behavior-validation bridge for a
passthrough `CreateLocalPath` action. In Jazzy it logged path size/endpoints and
returned the original path unchanged. It is not required for the Humble plugin
MVP and remains out of scope unless the behavior-validation BT pipeline is
needed later.

## Minimal Params

Use `nav2_colregs_bringup/params/nav2_colregs_params_humble_minimal.yaml` as a
plugin wiring reference. It is a mergeable configuration snippet, not a complete
standalone Nav2 bringup file.
