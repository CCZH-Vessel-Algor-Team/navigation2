# PerceivedObstacleLayer

`nav2_colregs_costmap_layers::PerceivedObstacleLayer` consumes
`usv_interfaces/msg/TrackedObstacleList` on `/tracked_obstacles`. It marks cells
intersecting each object's circular footprint as lethal. Put the plugin before
InflationLayer in both global and local costmaps.

The package requires a version of `usv_interfaces` containing `TrackedObstacle`
and `TrackedObstacleList` (the CCS interface branch is `feature/tracked_obstacle`).
This plugin does not perform COLREGS decisions or velocity prediction; `type` and
`twist` do not change the geometry. TSProjectionLayer remains a separate input.

## Configuration

Merge this block into each costmap's `ros__parameters`, and add
`perceived_obstacle_layer` to its `plugins` list **before** `inflation_layer`:

```yaml
perceived_obstacle_layer:
  plugin: "nav2_colregs_costmap_layers::PerceivedObstacleLayer"
  enabled: true
  tracked_obstacle_topic: "/tracked_obstacles"
  observation_timeout: 3.0
  tracking_frame: "map"
```

All four parameters are startup/read-only parameters. Restart the costmap after
changing configuration. Timeout is a finite, positive number of seconds.
The tracking frame must be stable (e.g. map), not a moving sensor/base frame.

## Input and clearing contract

- IDs are globally unique UUIDs. Each publisher may supply a partial list:
  observations update by ID; absent IDs and empty lists do not delete another
  publisher's objects. This supports the independent buoy and storm publishers.
- `header.stamp` is the **observation time** in the same ROS clock domain as the
  costmap. Duplicate/out-of-order observations do not renew the lifetime. Upstream
  must not relabel cached, unobserved objects with fresh measurement timestamps.
- Zero/negative, already expired, and more-than-one-timeout future timestamps are
  rejected. Small future clock-delivery skew is tolerated. ROS simulation time
  pauses also pause expiry; a clock rewind clears the old observation cache.
- Sensor coordinates are transformed at observation time into the tracking frame.
  Each costmap update transforms a consistent snapshot into its own global frame.
  Missing observation TF rejects that batch without refreshing old objects. No
  identity fallback is used for missing TF. Missing tracking-to-costmap TF makes
  the layer non-current until that transform is available again.
- Expiration is checked in `updateBounds`, including when no messages arrive.
  Old and new rendered regions are both reported so the layered costmap can
  remove this layer's old marks while rebuilding contributions from other layers.
- Radius must be finite/nonnegative. Positions must be finite. Circle-to-cell
  intersection is conservative, including small buoys and circles crossing the
  map boundary. Writes are restricted to the costmap update window.
- `reset()` drops the cache and schedules withdrawal of the previous region.
  This direct `Layer` returns `isClearable=false`: rectangular CostmapLayer clear
  operations do not apply; full costmap reset and automatic expiry are supported.
- Subscriber caching is mutex-protected. Input callbacks cannot change the render
  snapshot between `updateBounds` and `updateCosts`.

## Verification

In a sourced ROS 2 Humble workspace:

```bash
colcon build --packages-up-to nav2_colregs_costmap_layers --symlink-install
source install/setup.bash
colcon test --packages-select nav2_colregs_costmap_layers
colcon test-result --test-result-base build/nav2_colregs_costmap_layers --verbose
```

- C++ regressions cover multi-publisher retention, silent expiry, movement/shrink
  clearing, overlapping layers, stale/invalid data, measurement-time TF, rotated
  frames, rolling/resize, edge clipping, update-window limits, consistent render
  snapshots, reset/clock rewind, parameter validation and plugin/ROS discovery.
- The Python integration test runs two real planner-server/Costmap2DROS processes
  (map/global and rotated odom/rolling-local), with two ROS object publishers and
  InflationLayer. It checks occupancy, movement, independent storm retention and
  silent expiry. It sends no navigation goals and cleans up its own processes.
- GTest uses ROS domain 92. The Python test defaults to domain 93; override with
  `PERCEIVED_TEST_DOMAIN_ID` if needed. These domains must be reserved for tests.

Current CCS simulation wiring lives separately in
`usv_sim_full/config/radar_nav2_param.yaml`. A running costmap must be restarted
to load the newly built plugin and updated `plugins` list.
