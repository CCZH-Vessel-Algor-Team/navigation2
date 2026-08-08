# navigation2（Humble + COLREGS 插件级移植）

本仓库基于官方 `humble` 分支，移植了 `feat/colregs` 中与 COLREGS 插件链路相关的核心功能包。

当前 Humble 分支定位为 **COLREGS plugin MVP**：保留消息、规划器、控制器、TS 状态管理、TSProjectionLayer 代价图层、vector object server，以及最小 bringup 参数/BT XML/TS 子系统 launch；不移植完整 Gazebo/目标船仿真链路。

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
- `TSProjectionLayer`：订阅 `tracked_ship_topic`（默认 `/dynamic_ship/tracked_ships`，消息类型 `TrackedShipList`），逐 `target_id` 维护状态，在 `master_grid` 上标注目标船 LETHAL 圆。
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
- 作用：Humble 移植分支中保留参数、Behavior Tree XML 资源，以及不依赖 Gazebo 的 TS 子系统 launch。
- 当前安装内容：
  - `params/`
  - `behavior_trees/`
  - `launch/ts_subsystem_launch.py`
- 重点文件：
  - `params/nav2_colregs_params_humble_minimal.yaml`：Humble 插件接线示例，供合并到已有 Humble Nav2 params 使用。
  - `launch/ts_subsystem_launch.py`：启动 `ts_state_manager`、`avoidance_point_node`、`barrier_node`。
- 注意：该包在 Humble 分支不是完整仿真 bringup 包，不包含 Gazebo worlds/models/scripts/RViz 资源。

### 10) `nav2_maritime_situation_msgs`
- 作用：定义独立的海事态势输出接口 `SituationReport` 和 `SituationReportArray`。
- 单船报告包含目标 UUID、`cpa_valid`、DCPA、TCPA、会遇类型和风险等级；数组消息使用 `Header` 标识统一评估坐标系和时间。

### 11) `nav2_maritime_situation_monitor`
- 作用：订阅 `/tracked_ship` (`nav2_colregs_msgs/TrackedShipList`) 和 `/odom` (`nav_msgs/Odometry`)，在 ENU 平面计算所有有效目标的 CPA、风险等级和 COLREGS 会遇类型，并发布 `/maritime_situation` (`nav2_maritime_situation_msgs/SituationReportArray`)。
- 输出仅用于态势展示、记录和上层决策输入，不发送速度、路径或其他控制命令，也不加入 Nav2 lifecycle manager。
- 该包提供独立参数文件和 launch，不启动或 include Nav2 bringup、TS subsystem 或 lifecycle manager。

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
- `nav2_colregs_bringup`（params + behavior tree XML + TS subsystem launch）

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
- `local_costmap` / `global_costmap`：`nav2_colregs_costmap_layers::TSProjectionLayer`，显式配置 `tracked_ship_topic`。
- `ts_state_manager`：CPA/TCPA 和威胁状态参数，显式配置 `tracked_ship_topic`、`robot_base_frame`、`odom_topic`。
- `bt_navigator`：保留默认 Humble Nav2 BT XML/plugin set，不使用 Jazzy `CreateLocalPath` 诊断 BT 节点。

TS 子系统可单独启动：

```bash
ros2 launch nav2_colregs_bringup ts_subsystem_launch.py
```

该 launch 默认使用 `nav2_colregs_params_humble_minimal.yaml`，会启动：
- `ts_state_manager`
- `avoidance_point_node`
- `barrier_node`

`ts_subsystem_launch.py` 暴露 `tracked_ship_topic`、`robot_base_frame`、`odom_topic` 作为便捷参数，只作用于 TS subsystem 进程。TSProjectionLayer 是 Nav2 costmap 插件，不由该 launch 启动；如果需要改目标船 topic，必须同步修改传给 Nav2 的 params 文件中 local/global `ts_projection_layer.tracked_ship_topic`。

### 海事态势监控器

监控器可独立启动，不要求 Nav2 lifecycle 或 TS subsystem launch：

```bash
ros2 launch nav2_maritime_situation_monitor maritime_situation_monitor.launch.py
```

launch 参数为 `params_file`、`use_sim_time` 和可选 `namespace`（默认为空）。默认接口为输入 `/tracked_ship`、`/odom`，输出 `/maritime_situation`；可在 `config/maritime_situation_monitor.yaml` 中修改。节点只发布信息性态势报告，不控制本船。

`cpa_valid` 表示 DCPA/TCPA 是否由有效的非零相对速度预测得到。当相对速度严格小于 `relative_speed_epsilon`（默认 `1e-6 m/s`）时，CPA 预测退化：`cpa_valid=false`、`tcpa=0.0`、`dcpa` 为当前距离，并强制报告 `RISK_SAFE`。消费者必须先检查 `cpa_valid`，不得把该 `tcpa` 当作有效预测时间。

默认风险阈值（DCPA 单位 m，TCPA 单位 s）为：

| 等级 | DCPA | TCPA |
|---|---:|---:|
| INFO | 30.0 | 120.0 |
| WARNING | 20.0 | 30.0 |
| CRITICAL | 10.0 | 10.0 |

默认会遇分类阈值为：船首相遇方位 `6.0 deg`、反向航向容差 `15.0 deg`、追越船尾扇区 `112.5 deg`，航向速度下限为 `0.05 m/s`。输入超时默认值为目标船 `3.0 s`、本船里程计 `1.0 s`，TF 等待上限为 `0.2 s`，发布频率为 `0.5 Hz`。

### 参数文件状态

- `nav2_colregs_params_humble_minimal.yaml`：Humble 当前推荐配置片段，引用的 COLREGS 插件均已移植并 build 验证。
- `vector_object_server_params.yaml` / `vector_object_server_params_behavior_validation.yaml`：vector object server 参数；对应包已移植并 build 验证。
- `nav2_colregs_params_ts_projection_validation.yaml`：Jazzy 主开发配置。ALOS、VO-RRT*、TSProjectionLayer、`ts_state_manager` 对应组件已移植；但文件仍引用未移植的 `CreateLocalPath` Behavior/BT 节点，不能作为完整 Humble runtime 配置直接使用。
- `nav2_colregs_params_behavior_validation.yaml`：Jazzy behavior/keepout 验证配置。ALOS、keepout filter、vector object server 相关组件已移植；但完整 validation launch 与 `CreateLocalPath` Behavior/BT 节点未移植。
- `nav2_colregs_params.yaml` / `nav2_colregs_params_with_keepout.yaml`：Jazzy 基础/keepout 场景配置，保留作参考；未作为 Humble MVP runtime 配置验证。

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
  params/ behavior_trees/ launch/
nav2_maritime_situation_msgs/
  msg/
nav2_maritime_situation_monitor/
  config/ launch/ nav2_maritime_situation_monitor/ test/
```
