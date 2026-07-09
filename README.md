# navigation2（Humble + COLREGS 插件级移植）

本仓库基于官方 `humble` 分支，移植了 `feat/colregs` 中与 COLREGS 插件链路相关的核心功能包。

当前 Humble 分支定位为 **COLREGS plugin MVP**：保留消息、规划器、控制器、TS 状态管理、TSProjectionLayer 代价图层，以及最小 bringup 参数/BT XML；不移植完整 Gazebo/目标船仿真链路。

Jazzy 完整开发分支见 `feat/colregs`。Humble 移植状态详见 `doc/humble_colregs_port.md`。

## 一、相对官方新增的包

### 1) `nav2_colregs_msgs`
- 作用：定义 COLREGS 扩展消息与服务接口。
- 消息：
  - `TrackedShip.msg`：单条目标船（含 `target_id: UUID`、位姿、速度、半径）。
  - `TrackedShipList.msg`：批量目标船列表（`Header` + `TrackedShip[]`），统一发布到 `/tracked_ship`。
  - `ProcessedTS.msg`：经 TS State Manager 处理后的目标船快照（含 `target_id`、CPA/TCPA、威胁标识）。
  - `CircleObject.msg`、`PolygonObject.msg`：矢量障碍物形状接口，保留给 vector-object/keepout 链路。
- 服务：
  - `AddShapes.srv`、`GetShapes.srv`、`RemoveShapes.srv`：矢量对象增删查接口。

### 2) `nav2_rrt_star_planner`
- 作用：RRT* 全局规划器插件。
- Humble 适配：使用 Humble `nav2_core::GlobalPlanner::createPlan(start, goal)` 接口，不使用 Jazzy `cancel_checker` 参数。

### 3) `nav2_colregs_vo_rrt_star_planner`
- 作用：COLREGS VO-RRT* 全局规划器插件。
- 特点：在 RRT* 基础上接入目标船状态/速度障碍逻辑，用于生成符合 COLREGS 约束的候选路径。
- Humble 适配：同样使用 Humble two-argument planner API，并使用 Humble `nav2_core::PlannerException`。

### 4) `nav2_colregs_costmap_layers`
- 作用：自定义 Costmap Layer 插件。
- `TSProjectionLayer`：订阅 `/tracked_ship`（`TrackedShipList`），逐 `target_id` 维护状态，在 `master_grid` 上标注目标船 LETHAL 圆。
- 特性：支持多船、超时清理（默认 3s）、移动目标尾迹 bounds 累积清除、TF 坐标变换。
- 对标 `ObstacleLayer` 的 "topic -> 标记 master_grid" 模式，不通过 keepout mask 中转。

### 5) `nav2_colregs_vector_object_server`
- 作用：发布矢量对象栅格化后的 keepout mask（默认 `/keepout_filter_mask`）。
- 特点：LifecycleNode，支持 `circle` / `polygon`，内置指数梯度膨胀，可通过 `AddShapes` / `GetShapes` / `RemoveShapes` 服务动态维护矢量对象。
- Humble 适配：替换 Jazzy 参数 helper，使用 Humble timer API，补充 `uuid` 链接和 component 注册。
- 注意：包已移植并可 build，但 Humble 分支尚未恢复 Jazzy 的完整 keepout/vector-object validation launch 链路。

### 6) `nav2_colregs_ts_manager`
- 作用：TS 状态管理节点（LifecycleNode）。订阅 `/tracked_ship`（`TrackedShipList`）和 `/odom`，逐 `target_id` 维护多 TS 状态，计算 CPA/TCPA，发布 `/processed_ts_list` topic。
- TS 位姿通过 TF 从消息 `frame_id` 变换到 `global_frame`（通常为 `map`），确保多坐标系兼容。
- 与 `TSProjectionLayer` 配合：前者判断 TS 是否危险，后者将 TS 位姿画入 costmap。
- 超时参数：`ts_timeout: 3.0`（自动清除失联船舶）。

### 7) `nav2_colregs_los_controller`
- 作用：最简 LOS 制导 Controller 插件。
- 算法：沿路径找前视点，使用 `atan2` 计算目标艏向，再做角/线速度限制和 footprint 碰撞检测。
- Humble 适配：移除 Jazzy RPP `PathHandler` 依赖，内部保存并变换/prune 全局路径。
- 注意：`/lookahead_point` 发布的是原始全局路径上的前视点，仅做坐标变换，不包含 COLREGS 修正。

### 8) `nav2_colregs_alos_controller`
- 作用：Adaptive LOS（ALOS）制导 Controller 插件，在 LOS 基础上加入侧滑角自适应估计。
- 算法：基于 Fossen (2023)，找最近点和前推点，计算路径切线角、侧偏和自适应侧滑估计，输出目标航向和速度指令。
- 关键参数：`forward_dist`, `gamma`, `beta_hat0`, `reset_beta_on_new_path`。
- Humble 适配：移除 Jazzy RPP `PathHandler` 依赖，使用 Humble RPP 风格的路径变换逻辑。
- 注意：`/lookahead_point` 和 `/closest_point` 是原始全局路径上的调试/可视化点。COLREGS/ALOS 修正只作用于速度指令输出。

### 9) `nav2_colregs_bringup`
- 作用：Humble 移植分支中仅保留参数和 Behavior Tree XML 资源。
- 当前安装内容：
  - `params/`
  - `behavior_trees/`
- 重点文件：
  - `params/nav2_colregs_params_humble_minimal.yaml`：Humble 插件接线示例，供合并到已有 Humble Nav2 params 使用。
- 注意：该包在 Humble 分支不是完整仿真 bringup 包，不包含 Gazebo worlds/models/scripts/launch/RViz 资源。

## 二、Humble 移植状态

### 已移植并在 Humble apt 环境 build 验证
- `nav2_colregs_msgs`
- `nav2_rrt_star_planner`
- `nav2_colregs_ts_manager`
- `nav2_colregs_vo_rrt_star_planner`
- `nav2_colregs_costmap_layers`
- `nav2_colregs_vector_object_server`
- `nav2_colregs_los_controller`
- `nav2_colregs_alos_controller`
- `nav2_colregs_bringup`（params + behavior tree XML only）

验证命令：

```bash
cd ~/navigation2
source /opt/ros/humble/setup.sh
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

最新验证结果：

```text
Summary: 9 packages finished
```

### 暂未移植
- 完整 Gazebo / 目标船仿真 launch 文件。
- Gazebo worlds、models、bridge config、RViz 配置、地图和仿真脚本。
- keepout / vector-object validation launch chain。
- `nav2_colregs_local_path_behavior`。
- `nav2_colregs_local_path_bt_nodes`。

其中 `nav2_colregs_local_path_bt_nodes` 在 Jazzy 分支中主要用于 behavior-validation：从 blackboard 读取 `{path}`，调用 `CreateLocalPath` action，打印路径信息后透传为 `{local_path}`。它不是 Humble plugin MVP 的必要组件。

### Humble API 适配点
- Humble `nav2_core::GlobalPlanner::createPlan()` 不包含 Jazzy 的 `cancel_checker` 参数。
- Humble `nav2_core/exceptions.hpp` 使用 `PlannerException`，未使用 Jazzy-specific planner/controller exception subclasses。
- Humble RPP 不暴露 Jazzy 分支使用的 `PathHandler` helper，LOS/ALOS 控制器改为内部保存、变换和裁剪路径。

## 三、编译

### 仅编译 COLREGS Humble MVP 包

```bash
colcon build --symlink-install \
  --packages-select \
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

### 加载环境

```bash
source install/setup.bash
```

### 全量编译说明

全 workspace build 可能在 `nav2_system_tests` 处因缺少 Gazebo 测试依赖（例如 `gazebo_ros_pkgs`）失败。该失败不代表 COLREGS Humble plugin MVP 构建失败。

## 四、使用方式

Humble 分支当前不提供完整 COLREGS 仿真 launch。推荐从已有 Humble Nav2 bringup/仿真配置开始，将 `nav2_colregs_bringup/params/nav2_colregs_params_humble_minimal.yaml` 中的片段合并到工作参数文件。

最小接线包含：
- `planner_server`：`RRTStar`、`VORRTStar` 插件配置。
- `controller_server`：默认使用 `nav2_colregs_alos_controller::ALOSController`。
- `local_costmap` / `global_costmap`：`nav2_colregs_costmap_layers::TSProjectionLayer`。
- `ts_state_manager`：CPA/TCPA 和威胁状态参数。
- `bt_navigator`：保留默认 Humble Nav2 BT XML/plugin set，不使用 Jazzy `CreateLocalPath` 诊断 BT 节点。

## 五、关键参数

### TS State Manager
- `ts_timeout: 3.0`：目标船超时（秒），超时后移除。
- `tcpa_horizon: 10.0`：TCPA 预测窗口（秒）。
- `safety_factor: 1.1`：安全距离缩放因子。
- `os_radius: 0.3`：本船半径。
- `global_frame: "map"`：TS 状态统一坐标系。
- `robot_base_frame: "base_link"`：本船 base frame。

### TSProjectionLayer
- `track_timeout: 3.0`：目标船超时（秒），超时后从 costmap 移除。
- `enabled: true`

### Controller
- ALOS: `forward_dist: 2.0`, `gamma: 0.0006`, `beta_hat0: 0.0`, `max_angle_for_motion: 1.047`。
- LOS: 使用 `lookahead_dist` 替代 ALOS 的 `forward_dist/gamma/beta_hat0` 参数。

## 六、常用诊断命令

```bash
# 检查 /tracked_ship 消息格式
ros2 topic echo /tracked_ship --once

# 检查 costmap 中 TS LETHAL 标记
ros2 topic echo /local_costmap/costmap --once

# 检查 TS 处理列表
ros2 topic echo /processed_ts_list --once

# TF 检查
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map ts_virtual_base_link
```

## 七、目录结构（新增/移植包）

```text
nav2_colregs_msgs/
nav2_rrt_star_planner/
nav2_colregs_vo_rrt_star_planner/
nav2_colregs_costmap_layers/
  include/ src/ plugins.xml
nav2_colregs_vector_object_server/
  include/ src/ launch/ params/
nav2_colregs_ts_manager/
  include/ src/
nav2_colregs_los_controller/
  include/ src/ los_controller_plugin.xml
nav2_colregs_alos_controller/
  include/ src/ alos_controller_plugin.xml
nav2_colregs_bringup/
  params/ behavior_trees/
```
