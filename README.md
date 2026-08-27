# navigation2（Jazzy + COLREGS 扩展）

本仓库基于 `jazzy` 主干，新增了面向 COLREGS 场景的独立功能包与启动入口。
核心扩展：目标船（TS）仿真、TSProjectionLayer 代价图层、TS State Manager（CPA/TCPA）、LOS/ALOS 制导控制器、自定义 Behavior 插件。

RRT* 和 COLREGS VO-RRT* 成功规划（包括近似回退）在裁剪和插值前均以请求目标的精确 x/y 坐标结束。

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
  - `colregs_local_planner_demo_launch.py`：**主开发 launch**（自定义 RRT* Server + `colregs_costmap` + TSProjectionLayer + ALOS）。
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
- 作用：TS 状态计算库与 Server 托管的 TS 状态子节点。原三个独立节点（ts_state_manager、avoidance_point_node、barrier_node）与 `/processed_ts_list`、`/get_avoidance_point`、`/get_barrier_lines` 接口已内化删除。
- 组成：`ts_core` 纯计算库（原始/处理后快照类型、`processTs` 超时过滤 + CPA/TCPA + 碰撞锥、`evaluateColregs` 主威胁/安全航向/避让点/屏障线）与 `ColregsTsStateROS` 生命周期子节点。
- `ColregsTsStateROS`（节点名 `colregs_ts_state`）：由 `colregs_local_planner_server` 持有并编排（机制同 `colregs_costmap`），订阅 `/tracked_ship` 与 `/odom`，统一状态互斥锁维护 TS map，10 Hz timer 做超时剔除并发布 `/cpa_markers`（保留的唯一对外接口）。
- TS 位姿/速度经消息 frame→`global_frame`（map）变换旋转；TF 失败跳过该 TS（不回退原始坐标）。OS 速度由 odom body 分量按 TF 旋转到 map，失败时 `velocity_valid=false` 降级。
- 规划期接口：`getPlanningInput(os_x, os_y)` 单锁返回一致快照；阶段 3 起 `computePlan` 经 `processTs`/`evaluateColregs` 消费。
- 超时参数：`ts_timeout: 3.0`（自动清除失联船舶）。
- 兼容共存（过渡期）：三个旧独立节点 `ts_state_manager`/`avoidance_point_node`/`barrier_node` 与 `ts_subsystem_launch.py` 已自 `feat/colregs-humble` 恢复，编译为独立 `ts_manager_core` 库（与 `ts_core`/`colregs_ts_state_ros` 无符号/头重叠），仅供 legacy `planner_server + VORRTStar` 栈（`/processed_ts_list`、`/get_avoidance_point`、`/get_barrier_lines`）使用；legacy 栈迁移到新链路后应整体移除。
- 兼容共存（过渡期）：三个旧独立节点 `ts_state_manager`/`avoidance_point_node`/`barrier_node` 与 `ts_subsystem_launch.py` 已自 `feat/colregs-humble` 恢复，编译为独立 `ts_manager_core` 库（与 `ts_core`/`colregs_ts_state_ros` 无符号/头重叠），仅供 legacy `planner_server + VORRTStar` 栈（`/processed_ts_list`、`/get_avoidance_point`、`/get_barrier_lines`）使用；legacy 栈迁移到新链路后应整体移除。

### 8) `nav2_colregs_los_controller`
- 作用：最简 LOS 制导 Controller 插件。
- 算法：沿路径找前视点 → atan2 算目标艏向 → 角/线速度梯形加速 → footprint 碰撞检测。
- **注意：`/lookahead_point` 发布的是原始全局路径上前视点，仅做 map→base_link 变换，不包含 COLREGS 修正。**

### 9) `nav2_colregs_alos_controller`
- 作用：Adaptive LOS（ALOS）制导 Controller 插件，在 LOS 基础上加入侧滑角自适应估计。
- 算法：基于 Fossen (2023) — 找最近点 + 前推点 → 路径切线角 π_h + 侧偏 y_e → 自适应侧滑估计 β̂ → 目标角度 ψ_d = π_h - β̂ - atan(y_e/Δ)。
- 关键参数：`forward_dist`, `gamma`, `beta_hat0`, `reset_beta_on_new_goal`, `beta_reset_goal_dist_tolerance`。
- β̂ 估计在重规划间保留，仅当 goal 位移超过 `beta_reset_goal_dist_tolerance` 或 frame 变化时复位。
- 线程安全：`setPlan()` 与控制周期共享 controller mutex；控制周期按上游 RPP 模式持有 local costmap mutex，并在路径索引前校验状态、frame、pose 数量与有限坐标。
- 稀疏路径：forward point 按路径弧长插值，重复 segment 被跳过；退化终点仅在机器人已进入 goal tolerance 时返回零速度。
- 已知限制：原地转向期间 β̂ 仍会更新。详见代码注释。
- **注意：`/lookahead_point` 和 `/closest_point` 是原始全局路径上未经 COLREGS 修正的点（仅 map→base_link 变换），仅用于制导可视化。COLREGS 修正（β̂）只作用于速度指令输出。**

### 10) `nav2_colregs_local_planner_server`
- 作用：生命周期管理的 RRT* 路径规划 Server，替代主开发入口中的标准 PlannerServer。
- 生命周期节点：`/colregs_local_planner_server`。
- 标准 Action：`/compute_path_to_pose`（`nav2_msgs/action/ComputePathToPose`）；成功路径发布到 `/plan`。
- Server 内部持有固定 `map` 坐标系、非滚动的 `/colregs_costmap`，不由 Lifecycle Manager 单独管理。
- Server 同时编排 `colregs_ts_state` TS 状态子节点（独立 NodeThread，lifecycle 顺序固定为 costmap → TS）。
- 每次请求在 costmap mutex 内深拷贝快照，释放锁后对静态快照执行确定性 RRT*；接受空 `planner_id` 或 `RRTStar`。
- pruning 后的 RRT* raw nodes 按 `colregs_costmap` resolution 稠密插值，再保留精确 start/goal pose 后发布，避免 Controller 直接消费稀疏树节点。
- 阶段 3 已接入 VO-RRT 语义：每次请求经 `getTsPlanningInput` 取一致快照并做 `processTs`/`evaluateColregs` 决策；存在威胁且安全航向可行时执行两段式规划（start→避让点确定性段不做碰撞检查 + 避让点→goal 的 RRT*，barrier 线段进入 RRT* 碰撞判定），决策不激活或 OS 速度不可用时回退纯 RRT*；两段式 RRT* 段失败直接报错。`avoid_direction` 参数化（默认 `right`，即向右/starboard 过）。
- BT 以 `1 Hz` 重新规划，`FollowPath` 直接消费 `{path}`。当前实现为静态快照 + 决策几何约束，不含 encounter 分类与航行规则推理，不能宣称完整 COLREGS 合规。
- 已知限制：避让点若落在 costmap 膨胀区或图外，两段式将报 `NO_VALID_PATH`（不回退，沿用 VO-RRT 语义）；调参需保证避让距离与膨胀半径兼容。
- 决策可视化：`colregs_decision_markers`（MarkerArray，与 server 同命名空间）随每次决策发布——active 时为避让点 ARROW（ns `avoidance`）与屏障 LINE_LIST（ns `barrier`）的同帧组合，inactive 时发布 DELETE 立即清除；marker lifetime 7 s 自动过期。发布在决策计算后、任何锁外，频率等于重规划频率，不影响规划性能。

### 11) `nav2_maritime_situation_msgs`
- 作用：定义独立的海事态势接口 `SituationReport` 和 `SituationReportArray`。
- 单船报告包含目标 UUID、`cpa_valid`、DCPA、TCPA、会遇类型和风险等级；数组消息使用 `Header` 标识统一评估坐标系和时间。

### 12) `nav2_maritime_situation_monitor`
- 作用：订阅 `/tracked_ship`（`nav2_colregs_msgs/TrackedShipList`）和 `/odom`（`nav_msgs/Odometry`），在 ENU 平面评估所有有效目标，并发布 `/maritime_situation`（`nav2_maritime_situation_msgs/SituationReportArray`）。
- 组成：ROS 边界节点调用独立的 ENU 算法与报告构造模块；该包仅发布信息性态势报告，不发送速度或路径命令，也不接入主开发 launch 或 Nav2 lifecycle manager。
- 提供独立参数文件与 standalone launch，不 include Nav2 bringup 或 TS 子系统。

#### Stage 1 历史接口

2026-07 的 Stage 1 曾使用 `/compute_local_path`、`/local_path` 和 `ComputeLocalPath` BT 节点完成 pass-through 接线验证。该自定义 Action 与 BT 节点已在 Stage 2 删除；`CreateLocalPath` Behavior 仍供其他验证入口使用，不能把 Stage 1 命令当作当前接口。

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

### 2) `colregs_local_planner_demo_launch.py` ★ 主开发入口（colregs server 链）
```bash
ros2 launch nav2_colregs_bringup colregs_local_planner_demo_launch.py
```
作用：COLREGS server 全栈开发 launch（开阔水域场景）。组件链：
- **COLREGS Local Planner Server**（标准 `/compute_path_to_pose`，发布 `/plan`；进程内 VO-RRT 决策与 `colregs_decision_markers`）
- Server 自有 **`/colregs_costmap`**（map-fixed，Static/Obstacle/TSProjection/Inflation 四层）
- **TSProjectionLayer**（在规划与控制 costmap 内标记 TS 障碍物）
- **`colregs_ts_state` TS 状态子节点**（Server 进程内，CPA/TCPA/碰撞锥计算，发布 `/cpa_markers`）
- BT 以 **1 Hz** 调用 `ComputePathToPose`，将 `{path}` 直接交给 `FollowPath`
- **ALOS Controller**（制导）
- 使用专用 RViz：`colregs_local_planner_demo.rviz`
- 不含 vector_object_server / keepout 链路。

场景与便利特性：
- 默认 **放大围墙场景**：`colregs_open_world`（围墙 ±15 m，场地 30×30 m，比原 24×24 m 参考场景略大；3 根外扩特征柱 (-8,6)/(4,-7)/(11,10)）+ 36×36 m 地图 `colregs_open_map`。围墙为 AMCL 提供连续定位特征，柱间中央走廊用于 VO-RRT 避障观测。
- TS 默认 spawn (8, 9)、0.4 m/s、往返 10 m，走廊锁定世界 y 轴（`ts_motion_axis_mode:=y`，x=8 竖向走廊 y∈[-1,9]），不依赖首帧 yaw 捕获，端点距墙 ≥6 m；与东西向巡逻构成交叉会遇。`colregs_ts_simulation_launch.py` 新增 `ts_motion_axis_mode` 参数（默认 `auto` 保持旧行为，可选 `x`/`y`/`angle`）。
- **AMCL 自动初始位姿**：`set_initial_pose` 预置与默认 spawn (6, 0, 0°) 一致，启动后无需手点 2D Pose Estimate；如覆盖 `x_pose/y_pose/yaw`，需同步修改 params 中 `amcl.initial_pose` 或在 RViz 手动指定。

本入口不启动 `/planner_server` 或 `/global_costmap`。RRT* 依据请求时的静态 costmap 快照 + 决策几何约束（避让点/barrier）规划；当前不含 encounter 分类与航行规则推理，不能宣称完整 COLREGS 合规。

### 2b) `colregs_ts_projection_validation_launch.py`（legacy 链，A/B 对照）
```bash
ros2 launch nav2_colregs_bringup colregs_ts_projection_validation_launch.py
```
作用：恢复的 legacy 验证入口——标准 `planner_server`（NavFn + VORRTStar 插件）+ 独立 TS 三节点子系统（`ts_state_manager`/`avoidance_point_node`/`barrier_node`，经 `/processed_ts_list` 与两个 service 交互）+ 标准 Nav2 bringup。用于新旧链路 A/B 对照；参数文件为 `nav2_colregs_params_ts_projection_validation.yaml`（legacy 版）。TS 节点与 server 链互不干扰，可与新入口并行分析，但同一 ROS domain 内不要同时运行两套导航栈。

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

# 检查 TS 状态子节点的 CPA markers
ros2 topic echo /cpa_markers --once

# TF 检查
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map ts_virtual_base_link
```

### RRT* Local Planner Server 验证边界

当前开发主机为 Conda/RoboStack 环境，只执行静态检查，不在该环境运行 `colcon`、`ros2` 或 launch。以下构建、测试和运行命令必须在 apt ROS 2 Jazzy 环境执行。

```bash
colcon build --symlink-install --packages-select \
  nav2_colregs_msgs \
  nav2_colregs_local_path_bt_nodes \
  nav2_colregs_local_planner_server \
  nav2_colregs_bringup
source install/setup.bash

colcon test --packages-select \
  nav2_colregs_msgs \
  nav2_colregs_local_path_bt_nodes \
  nav2_colregs_local_planner_server \
  nav2_colregs_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

分别验证 composed 与 non-composed 模式：

```bash
ros2 launch nav2_colregs_bringup colregs_local_planner_demo_launch.py use_composition:=True
ros2 launch nav2_colregs_bringup colregs_local_planner_demo_launch.py use_composition:=False
```

发送 NavigateToPose goal 后，在其他已 source workspace 的终端采集。以下命令应有输出，并且四个 lifecycle 节点均应为 `active`：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /colregs_local_planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 action list -t | grep -Fx '/compute_path_to_pose [nav2_msgs/action/ComputePathToPose]'
ros2 topic echo /colregs_costmap/costmap --once
ros2 topic echo /plan --once
ros2 action info /follow_path
ros2 topic echo /cmd_vel --once
```

以下命令预期**无输出**；任一输出都表示 obsolete 拓扑仍然存在：

```bash
ros2 node list | grep -E '(^|/)planner_server($|/)' || true
ros2 node list | grep -F '/global_costmap' || true
ros2 topic list | grep -E '^/global_costmap(/|$)' || true
ros2 action list | grep -Fx '/compute_local_path' || true
```

验收需记录两种模式下四个 lifecycle 节点均为 `active`、`/compute_path_to_pose` 类型严格为 `nav2_msgs/action/ComputePathToPose`、`/colregs_costmap/costmap` 与 `/plan` 有输出、Controller 发布速度，并确认不存在任何 PlannerServer 节点、名称含 `/global_costmap` 的嵌套节点、`/global_costmap` topic 树和 `/compute_local_path`。当前 Conda 主机只完成静态验证，apt Jazzy 构建与运行证据仍待用户采集。

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
