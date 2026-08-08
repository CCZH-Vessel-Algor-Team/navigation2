# navigation2（Jazzy + COLREGS 扩展）

本仓库基于 `jazzy` 主干，新增了面向 COLREGS 场景的独立功能包与启动入口。
核心扩展：目标船（TS）仿真、TSProjectionLayer 代价图层、TS State Manager（CPA/TCPA）、LOS/ALOS 制导控制器、自定义 Behavior 插件。

## 一、相对官方新增的包

### 1) `nav2_colregs_msgs`
- 作用：定义 COLREGS 扩展消息与服务接口。
- 消息：
  - `TrackedShip.msg`：单条目标船（含 `target_id: UUID`、位姿、速度、半径）。
  - `TrackedShipList.msg`：**批量目标船列表**（`Header` + `TrackedShip[]`），融合节点统一发布到 `/tracked_ship`。
  - `ProcessedTS.msg`：经 TS State Manager 处理后的目标船快照（含 `target_id`、CPA/TCPA、威胁标识）。
  - `CircleObject.msg`、`PolygonObject.msg`：矢量障碍物形状（vector_object_server 用）。
- 服务：
  - `AddShapes.srv`、`GetShapes.srv`、`RemoveShapes.srv`：矢量对象增删查。

### 2) `nav2_colregs_vector_object_server`
- 作用：发布矢量对象栅格化后的 keepout mask（`/keepout_filter_mask`）。**当前开发链路上不使用**。
- 特点：生命周期节点（LifecycleNode）。支持 `circle` / `polygon`，内置指数梯度膨胀。

### 3) `nav2_colregs_bringup`
- 作用：COLREGS 场景的 launch、参数、地图、世界、模型、脚本。
- 主要 launch：
  - `colregs_ts_simulation_launch.py`：基础 TS 仿真（不含 Nav2 定制）。
  - `colregs_ts_projection_validation_launch.py`：**主开发 launch**（TSProjectionLayer + TS State Manager + ALOS + CreateLocalPath BT 节点）。
  - `colregs_ts_behavior_validation_launch.py`：Behavior plugin 独立验证。
- 脚本：
  - `target_ship_state_publisher.py`：从 Gazebo 位姿生成 `TrackedShipList`，`target_id` 用 UUID5 确定性推导，广播 `map → ts_virtual_base_link`。
  - `target_ship_motion_commander.py`：驱动目标船往返运动。

### 4) `nav2_colregs_costmap_layers`
- 作用：自定义 Costmap Layer 插件。
- `TSProjectionLayer`：订阅 `tracked_ship_topic`（默认 `/tracked_ship`，消息类型 `TrackedShipList`），逐 `target_id` 维护状态，在 `master_grid` 上标注目标船 LETHAL 圆。含超时清理（3s）、尾迹累积 bounds 清除。支持多船。
- 对标 `ObstacleLayer` 的 "topic → 标记 master_grid" 模式，不通过 keepout mask 中转。

### 5) `nav2_colregs_local_path_behavior`
- 作用：自定义 Behavior 插件（`TimedBehavior`），接收全局路径并在 `onRun()` 中打印路径信息。
- Action：`CreateLocalPath.action`，Goal 为 `nav_msgs/Path`。

### 6) `nav2_colregs_local_path_bt_nodes`
- 作用：`CreateLocalPath` 的 BT Action Node，独立编译为 `nav2_create_local_path_action_bt_node` 库。
- 从 blackboard 读取 `{path}`，编码为 action goal 发送到 behavior_server，成功后将路径透传写回 `{local_path}`。

### 7) `nav2_colregs_ts_manager`
- 作用：TS 状态管理节点（LifecycleNode）。订阅 `/tracked_ship`（`TrackedShipList`）和 `/odom`，逐 `target_id` 维护多 TS 状态，计算 CPA/TCPA，发布 `/processed_ts_list` topic。
- TS 位姿通过 TF 从消息 frame_id 变换到 `global_frame`（map），确保多坐标系兼容。
- 与 `TSProjectionLayer` 配合：前者判断"TS 是否危险"，后者将"TS 位姿画入 costmap"。
- 超时参数：`ts_timeout: 3.0`（自动清除失联船舶）。

### 8) `nav2_colregs_los_controller`
- 作用：最简 LOS 制导 Controller 插件。
- 算法：沿路径找前视点 → atan2 算目标艏向 → 角/线速度梯形加速 → footprint 碰撞检测。
- **注意：`/lookahead_point` 发布的是原始全局路径上前视点，仅做 map→base_link 变换，不包含 COLREGS 修正。**

### 9) `nav2_colregs_alos_controller`
- 作用：Adaptive LOS（ALOS）制导 Controller 插件，在 LOS 基础上加入侧滑角自适应估计。
- 算法：基于 Fossen (2023) — 找最近点 + 前推点 → 路径切线角 π_h + 侧偏 y_e → 自适应侧滑估计 β̂ → 目标角度 ψ_d = π_h - β̂ - atan(y_e/Δ)。
- 关键参数：`forward_dist`, `gamma`, `beta_hat0`, `reset_beta_on_new_path`。
- 已知限制：原地转向期间 β̂ 仍会更新。详见代码注释。
- **注意：`/lookahead_point` 和 `/closest_point` 是原始全局路径上未经 COLREGS 修正的点（仅 map→base_link 变换），仅用于制导可视化。COLREGS 修正（β̂）只作用于速度指令输出。**

### 10) `nav2_colregs_local_planner_server`
- 作用：生命周期管理的本地路径规划 Action Server Demo，用于验证 `BT → Action Server → BT → Controller` 接线。
- 生命周期节点：`/colregs_local_planner_server`。
- Action：`/compute_local_path`（`nav2_colregs_msgs/action/ComputeLocalPath`）。
- 发布 topic：`/local_path`（`nav_msgs/msg/Path`）。
- 当前实现仅将 `reference_path` 透传为 `local_path`，保留 frame 和全部 poses，仅刷新路径时间戳；尚不读取 TS 上下文，也不改变路径几何。
- 当前执行模型为串行且不支持抢占。取消采用 group cancellation：执行过程在关键边界检查取消请求，观察到请求后以 `CANCELED` 和 `Local path computation canceled` 终止该 server 的全部 goals，不提供逐 goal 独立取消语义。受 `SimpleActionServer` 非原子检查/完成 API 限制，最终取消检查与发布/成功完成之间仍存在极小竞态窗口。

### 11) `nav2_maritime_situation_msgs`
- 作用：定义独立的海事态势接口 `SituationReport` 和 `SituationReportArray`。
- 单船报告包含目标 UUID、`cpa_valid`、DCPA、TCPA、会遇类型和风险等级；数组消息使用 `Header` 标识统一评估坐标系和时间。

### 12) `nav2_maritime_situation_monitor`
- 作用：订阅 `/tracked_ship`（`nav2_colregs_msgs/TrackedShipList`）和 `/odom`（`nav_msgs/Odometry`），在 ENU 平面评估所有有效目标，并发布 `/maritime_situation`（`nav2_maritime_situation_msgs/SituationReportArray`）。
- 组成：ROS 边界节点调用独立的 ENU 算法与报告构造模块；该包仅发布信息性态势报告，不发送速度或路径命令，也不接入主开发 launch 或 Nav2 lifecycle manager。
- 提供独立参数文件与 standalone launch，不 include Nav2 bringup 或 TS 子系统。

## 二、编译

### 全量编译
```bash
colcon build --symlink-install
```

### 仅编译 COLREGS 包
```bash
colcon build --symlink-install \
  --packages-select \
  nav2_colregs_msgs \
  nav2_colregs_vector_object_server \
  nav2_colregs_costmap_layers \
  nav2_colregs_local_planner_server \
  nav2_colregs_local_path_bt_nodes \
  nav2_colregs_local_path_behavior \
  nav2_colregs_ts_manager \
  nav2_colregs_los_controller \
  nav2_colregs_alos_controller \
  nav2_maritime_situation_msgs \
  nav2_maritime_situation_monitor \
  nav2_colregs_bringup
```

> `reference/` 已放置 `COLCON_IGNORE`，不会被 colcon 扫描。

### 加载环境
```bash
source install/setup.bash
```

## 三、Launch 入口

> gz sim 可能需要 `export GZ_IP=127.0.0.1`

### 1) `colregs_ts_simulation_launch.py`
```bash
ros2 launch nav2_colregs_bringup colregs_ts_simulation_launch.py
```
作用：单船 + 目标船（TS）仿真，含 TS 状态发布与运动控制。不含 Nav2 定制组件。

### 2) `colregs_ts_projection_validation_launch.py` ★ 主开发入口
```bash
ros2 launch nav2_colregs_bringup colregs_ts_projection_validation_launch.py
```
作用：COLREGS 全套开发 launch。组件链：
- **TSProjectionLayer**（costmap 内标记 TS 障碍物）
- **TS State Manager**（CPA/TCPA 计算，`/processed_ts_list` topic）
- **ComputeLocalPath BT 节点**调用 `/compute_local_path`，将 Action result `{local_path}` 交给 `FollowPath`
- **ALOS Controller**（制导）
- 不含 vector_object_server / keepout 链路。

本入口中的 Local Planner Server 是 plumbing Demo，不是 COLREGS 路径规划算法。它只验证生命周期启动、Action 调用、BT result 传递和 Controller 接线；当前不消费 TS 数据，也不修改全局路径几何。

### 3) `colregs_ts_behavior_validation_launch.py`
```bash
ros2 launch nav2_colregs_bringup colregs_ts_behavior_validation_launch.py
```
作用：Behavior plugin 独立验证。在 TS 仿真 + keepout 基础上，插入 `CreateLocalPath` BT 节点。

### 4) `colregs_simulation_launch.py`
```bash
ros2 launch nav2_colregs_bringup colregs_simulation_launch.py
```
作用：单船（TB3）COLREGS 基础仿真，不含目标船。

### 5) `maritime_situation_monitor.launch.py`
```bash
ros2 launch nav2_maritime_situation_monitor maritime_situation_monitor.launch.py
```
作用：独立启动海事态势监控器。launch 参数为 `params_file`、`use_sim_time` 和可选 `namespace`；默认输入为 `/tracked_ship`、`/odom`，输出为 `/maritime_situation`。

## 四、关键参数

### TS 仿真参数
- `ts_x_pose`、`ts_y_pose`、`ts_yaw`：目标船初始位姿。
- `ts_speed`：目标船速度。
- `ts_pingpong_distance`：往返切换距离。
- `enable_ts_motion`：是否启用目标船运动控制。

### TS State Manager
- `tracked_ship_topic: "/tracked_ship"`：目标船输入 topic。若修改 TS 发布源，需同步修改 TS State Manager 与 TSProjectionLayer 的该参数。
- `ts_timeout: 3.0`：目标船超时（秒），超时后移除。
- `tcpa_horizon: 10.0`：TCPA 预测窗口（秒）。
- `safety_factor: 1.1`：安全距离缩放因子。
- `os_radius: 0.3`：本船半径。

### TSProjectionLayer
- `tracked_ship_topic: "/tracked_ship"`：目标船输入 topic，应与 TS State Manager 保持一致。
- `track_timeout: 3.0`：目标船超时（秒），超时后从 costmap 移除。
- `enabled: True`

### Controller
- ALOS: `forward_dist: 2.0`, `gamma: 0.0006`, `max_angle_for_motion: 1.047`
- LOS: `lookahead_dist`（见各 params yaml）

### 海事态势监控器
- `cpa_valid` 表示 DCPA/TCPA 是否由有效的非零相对速度预测得到。相对速度严格小于 `relative_speed_epsilon`（默认 `1e-6 m/s`）时，`cpa_valid=false`、`tcpa=0.0`、`dcpa` 为当前距离，并强制报告 `RISK_SAFE`；消费者必须先检查 `cpa_valid`。
- 默认风险阈值：INFO 为 DCPA `30.0 m` / TCPA `120.0 s`，WARNING 为 `20.0 m` / `30.0 s`，CRITICAL 为 `10.0 m` / `10.0 s`。

## 五、常用诊断命令

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

### Local Planner Server Demo 验证边界

当前开发主机为 Conda/RoboStack 环境，只执行静态检查，不在该环境运行 `colcon`、`ros2` 或 launch。以下构建、测试和运行命令必须在 apt ROS 2 Jazzy 环境执行。

以下 focused build 是增量命令，假定当前 `feat/colregs` 基线及已有自定义运行依赖已在 workspace 中构建并可被 source。它只构建本 Demo 的五个功能包，不会从干净 workspace 构建完整仿真栈：

```bash
colcon build --symlink-install --packages-select \
  nav2_colregs_msgs \
  nav2_colregs_local_planner_server \
  nav2_colregs_local_path_behavior \
  nav2_colregs_local_path_bt_nodes \
  nav2_colregs_bringup
source install/setup.bash
```

主 launch 还使用基线中的 `nav2_colregs_ts_manager`、`nav2_colregs_costmap_layers`、`nav2_colregs_alos_controller`、`nav2_colregs_vo_rrt_star_planner`，以及 Nav2、Gazebo 和 `nav2_minimal_tb3_sim`。若从干净的源码 workspace 构建，应使用 dependency-resolving 的较大范围命令（并确保 apt/system dependencies 已安装）：

```bash
colcon build --symlink-install --packages-up-to \
  nav2_colregs_bringup \
  nav2_colregs_ts_manager \
  nav2_colregs_costmap_layers \
  nav2_colregs_alos_controller \
  nav2_colregs_vo_rrt_star_planner
source install/setup.bash
```

`--packages-up-to` 会构建上述目标及其声明的递归依赖，因此范围显著大于五包 focused build；它用于准备完整 launch 所需的源码依赖，而不是 focused feature rebuild。

```bash
# focused tests
colcon test --packages-select \
  nav2_colregs_local_planner_server \
  nav2_colregs_local_path_bt_nodes \
  --event-handlers console_direct+
colcon test-result --verbose

# launch; retain this terminal output for BT plugin and runtime-error evidence
ros2 launch nav2_colregs_bringup colregs_ts_projection_validation_launch.py
```

发送 NavigateToPose goal 后，在其他已 source workspace 的终端采集：

```bash
ros2 lifecycle get /colregs_local_planner_server
ros2 action list -t | grep '/compute_local_path'
ros2 action info /compute_local_path
ros2 topic echo /local_path --once
ros2 action info /follow_path
ros2 topic echo /cmd_vel --once
```

需要记录实际结果，而不是预先声明通过：focused build/test 结果、生命周期是否为 `active`、`/compute_local_path` 的 action server 数量、`nav2_compute_local_path_action_bt_node` 是否成功加载、是否发布 `/local_path`、`FollowPath`/Controller 是否消费结果，以及所有 launch/runtime errors。由于 Demo 输出与输入路径几何相同，Controller 消费的运行证据应结合成功的 NavigateToPose 执行、`/local_path` 消息及 Controller 输出判断；BT XML 的接线本身仅属于静态证据。

## 六、目录结构（新增包）

```text
nav2_colregs_msgs/
nav2_colregs_vector_object_server/
nav2_colregs_costmap_layers/
  include/ src/ plugins.xml
nav2_colregs_ts_manager/
  include/ src/
nav2_colregs_local_planner_server/
  include/ src/ test/
nav2_colregs_local_path_bt_nodes/
  include/ src/
nav2_colregs_local_path_behavior/
  action/ include/ src/ behavior_plugin.xml
nav2_colregs_los_controller/
  include/ src/ los_controller_plugin.xml
nav2_colregs_alos_controller/
  include/ src/ alos_controller_plugin.xml
nav2_maritime_situation_msgs/
  msg/
nav2_maritime_situation_monitor/
  config/ launch/ nav2_maritime_situation_monitor/ test/
nav2_colregs_bringup/
  launch/ params/ behavior_trees/ maps/ worlds/ models/ rviz/ scripts/
```

## 七、Bug 修复记录

- **Keepout filter 动态跟随与拖影**：见 `doc/keepout_filter_bugfix.md`
- **VoxelLayer 远场残留**：`raytrace_max_range` 必须匹配 `obstacle_max_range`
- **StaticLayer 覆写动态层**：local costmap 必须移除 `static_layer`
