# navigation2（Jazzy + COLREGS 扩展）

本仓库基于 `jazzy` 主干，新增了面向 COLREGS 场景的独立功能包与启动入口。  
与官方 `navigation2` 相比，核心差异是新增了目标船（TS）仿真链路、动态 keepout 掩码链路和配套消息/服务定义。

## 一、相对官方新增的包

### 1) `nav2_colregs_msgs`
- 作用：定义 COLREGS 扩展消息与服务接口。
- 主要内容：
  - `msg/TrackedShip.msg`：目标船状态。
  - `msg/CircleObject.msg`、`msg/PolygonObject.msg`：矢量障碍物形状。
  - `srv/AddShapes.srv`、`srv/GetShapes.srv`、`srv/RemoveShapes.srv`：矢量对象增删查。

### 2) `nav2_colregs_vector_object_server`
- 作用：发布矢量对象栅格化后的 keepout mask（`/keepout_filter_mask`）。
- 特点：
  - 生命周期节点（LifecycleNode）。
  - 支持 `circle` / `polygon`。
  - 支持静态地图尺寸或动态尺寸。
  - 内置指数梯度膨胀（复用 Nav2 InflationLayer 模型）：`inflation_radius` / `cost_scaling_factor` / `inscribed_radius` 为全局参数，所有 shape 共享。
  - 与 `nav2_map_server/costmap_filter_info_server` 配套使用。

### 3) `nav2_colregs_bringup`
- 作用：提供 COLREGS 场景的 launch、参数、地图、世界、模型、脚本。
- 主要脚本：
  - `scripts/target_ship_state_publisher.py`：从 Gazebo 世界位姿生成 `/tracked_ship`，并广播 `map -> ts_virtual_base_link`。
  - `scripts/target_ship_motion_commander.py`：发布 `/target_ship/cmd_vel`，驱动目标船往返运动。

### 4) `nav2_colregs_local_path_behavior`
- 作用：自定义 Behavior 插件（`TimedBehavior`），由 `behavior_server` 加载，接收全局路径并在 `onRun()` 中打印路径长度与首末点。
- Action：`action/CreateLocalPath.action`，Goal 为 `nav_msgs/Path`。
- 对应的 BT Action Node 在 `nav2_colregs_local_path_bt_nodes` 中。

### 5) `nav2_colregs_local_path_bt_nodes`
- 作用：`CreateLocalPath` 的 BT Action Node，独立编译为 `nav2_create_local_path_action_bt_node` 库。
- 从 blackboard 读取 `{path}`，编码为 action goal 发送到 behavior_server，成功后将路径透传写回 `{local_path}`。
- 依赖 `nav2_behavior_tree` + `nav2_colregs_local_path_behavior`（action 类型）。

### 6) `nav2_colregs_los_controller`
- 作用：最简 LOS 制导 Controller 插件，搭载于 `controller_server`。
- 算法：沿路径找前视点 → atan2(y,x) 算目标艏向 → 角/线速度梯形加速 → footprint 碰撞检测。
- 关键参数：`desired_linear_vel`, `max_linear_accel`, `max_angular_vel`, `max_angular_accel`, `lookahead_dist`, `max_angle_for_motion`（超阈值原地转向，0=关闭）。

### 7) `nav2_colregs_alos_controller`
- 作用：Adaptive LOS（ALOS）制导 Controller 插件，在 LOS 基础上加入侧滑角自适应估计。
- 算法：基于 Fossen (2023) — 找最近点 + 前推点 → 计算路径切线角 π_h 和侧偏 y_e → 自适应侧滑估计 β̂ → 目标角度 ψ_d = π_h - β̂ - atan(y_e/Δ)。
- 特点：无段追踪（利用稠密 path），β̂ 积分消除 USV 流/风稳态侧偏。
- 关键参数：`desired_linear_vel`, `max_linear_accel`, `max_angular_vel`, `max_angular_accel`, `forward_dist`, `gamma`, `beta_hat0`, `reset_beta_on_new_path`。
- 已知限制：原地转向（`max_angle_for_motion` 触发）期间 β̂ 仍会更新，几何 y_e 可能被误认为侧滑。详见代码注释。

## 二、编译方式

> 建议在 **clean 的、apt 安装的 ROS 2 Jazzy 环境** 中执行。
> 当前使用 KVM 仅是因为宿主机已采用 RoboStack 管理 ROS 栈，不满足该前提。

### 1) 全量编译

```bash
colcon build --symlink-install
```

### 2) 仅编译 COLREGS 新增包

```bash
colcon build --symlink-install \
  --packages-select \
  nav2_colregs_msgs \
  nav2_colregs_vector_object_server \
  nav2_colregs_local_path_bt_nodes \
  nav2_colregs_local_path_behavior \
  nav2_colregs_los_controller \
  nav2_colregs_alos_controller \
  nav2_colregs_bringup
```

> `reference/` 目录已放置 `COLCON_IGNORE` 文件，colcon 不会扫描该目录，避免与 `src/navigation2/` 下同名包冲突。

### 3) 编译后加载环境

```bash
source install/setup.bash
```

## 三、启动方式（launch 入口）

注：gz sim 可能需要额外设置 `export GZ_IP=127.0.0.1`

### 1) `colregs_simulation_launch.py`
- 命令：

```bash
ros2 launch nav2_colregs_bringup colregs_simulation_launch.py
```

- 作用：单船（TB3）COLREGS 基础仿真，不含目标船。

### 2) `colregs_ts_simulation_launch.py`
- 命令：

```bash
ros2 launch nav2_colregs_bringup colregs_ts_simulation_launch.py
```

- 作用：单船 + 目标船（TS）仿真，含 TS 状态发布与运动控制。

### 3) `colregs_ts_keepout_simulation_launch.py`
- 命令：

```bash
ros2 launch nav2_colregs_bringup colregs_ts_keepout_simulation_launch.py
```

- 作用：在 TS 仿真基础上，加载静态 keepout mask + keepout filter info。

### 4) `colregs_ts_vector_keepout_simulation_launch.py`
- 命令：

```bash
ros2 launch nav2_colregs_bringup colregs_ts_vector_keepout_simulation_launch.py
```

- 作用：在 TS 仿真基础上，使用 `nav2_colregs_vector_object_server` 动态生成 keepout mask。

### 5) `colregs_ts_behavior_validation_launch.py`
- 命令：

```bash
ros2 launch nav2_colregs_bringup colregs_ts_behavior_validation_launch.py
```

- 作用：独立验证 launch。在 TS 仿真 + vector keepout 基础上，插入 `CreateLocalPath` BT 节点（1Hz），读取全局路径、打印路径信息并通过 `{local_path}` 透传给 `FollowPath`，用于 behavior 插件端到端验证。

## 四、关键可配置参数

以下为高频参数（完整参数见 `nav2_colregs_bringup/launch/*.py` 与 `params/*.yaml`）。

### 通用参数（多个 launch 共享）
- `use_sim_time`：是否使用仿真时钟（默认 `true`）。
- `autostart`：Nav2 生命周期自动激活（默认 `true`）。
- `headless`：是否关闭 Gazebo GUI（默认 `True` 或 `False`，以各 launch 默认值为准）。
- `params_file`：Nav2 参数文件路径。

### TS 仿真参数（`colregs_ts_simulation_launch.py`）
- `ts_x_pose`、`ts_y_pose`、`ts_yaw`：目标船初始位姿。
- `ts_speed`：目标船速度。
- `ts_pingpong_distance`：往返切换距离。
- `enable_ts_motion`：是否启用目标船运动控制。

### Keepout 参数
- `keepout_mask`（`colregs_ts_keepout_simulation_launch.py`）：静态 keepout mask yaml 路径。
- `vector_keepout_params_file`（`colregs_ts_vector_keepout_simulation_launch.py`）：矢量对象服务参数文件路径。
- `inflation_radius` / `cost_scaling_factor` / `inscribed_radius`（`vector_object_server`）：全局 shape 膨胀参数，复用 Nav2 InflationLayer 的指数衰减模型。

## 五、推荐启动示例

```bash
export GZ_IP=127.0.0.1
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch nav2_colregs_bringup colregs_ts_vector_keepout_simulation_launch.py \
  headless:=False
```

## 六、目录说明（新增包）

```text
nav2_colregs_msgs/
nav2_colregs_vector_object_server/
nav2_colregs_local_path_bt_nodes/
  include/
  src/
nav2_colregs_local_path_behavior/
  action/
  include/
  src/
  behavior_plugin.xml
nav2_colregs_los_controller/
  include/
  src/
  los_controller_plugin.xml
nav2_colregs_alos_controller/
  include/
  src/
  alos_controller_plugin.xml
nav2_colregs_bringup/
  launch/
  params/
  behavior_trees/
  maps/
  worlds/
  models/
  rviz/
  scripts/
```

## 七、Phase 2 运行与诊断

Phase 2 的运行基线、验收标准和 Keepout 分层诊断步骤见：

- `doc/colregs_phase2_runbook.md`

---

## 八、Bug 修复记录

Keepout filter 动态跟随与拖影问题的根因分析及修复详见：

- `doc/keepout_filter_bugfix.md`
