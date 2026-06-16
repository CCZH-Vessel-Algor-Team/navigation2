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
  - `GetPrimaryThreat.srv`：查询主威胁（TCPA 最小的 TS，含 `ProcessedTS`）。
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
- `TSProjectionLayer`：订阅 `/tracked_ship`（`TrackedShipList`），逐 `target_id` 维护状态，在 `master_grid` 上标注目标船 LETHAL 圆。含超时清理（3s）、尾迹累积 bounds 清除。支持多船。
- 对标 `ObstacleLayer` 的 "topic → 标记 master_grid" 模式，不通过 keepout mask 中转。

### 5) `nav2_colregs_local_path_behavior`
- 作用：自定义 Behavior 插件（`TimedBehavior`），接收全局路径并在 `onRun()` 中打印路径信息。
- Action：`CreateLocalPath.action`，Goal 为 `nav_msgs/Path`。

### 6) `nav2_colregs_local_path_bt_nodes`
- 作用：`CreateLocalPath` 的 BT Action Node，独立编译为 `nav2_create_local_path_action_bt_node` 库。
- 从 blackboard 读取 `{path}`，编码为 action goal 发送到 behavior_server，成功后将路径透传写回 `{local_path}`。

### 7) `nav2_colregs_ts_manager`
- 作用：TS 状态管理节点（LifecycleNode）。订阅 `/tracked_ship`（`TrackedShipList`）和 `/odom`，逐 `target_id` 维护多 TS 状态，计算 CPA/TCPA，通过 `/get_primary_threat` 服务返回 TCPA 最紧迫的威胁。
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
  nav2_colregs_local_path_bt_nodes \
  nav2_colregs_local_path_behavior \
  nav2_colregs_ts_manager \
  nav2_colregs_los_controller \
  nav2_colregs_alos_controller \
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
- **TS State Manager**（CPA/TCPA 威胁判定，`/get_primary_threat` 服务）
- **CreateLocalPath BT 节点**（透传路径）
- **ALOS Controller**（制导）
- 不含 vector_object_server / keepout 链路。

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

## 四、关键参数

### TS 仿真参数
- `ts_x_pose`、`ts_y_pose`、`ts_yaw`：目标船初始位姿。
- `ts_speed`：目标船速度。
- `ts_pingpong_distance`：往返切换距离。
- `enable_ts_motion`：是否启用目标船运动控制。

### TS State Manager
- `ts_timeout: 3.0`：目标船超时（秒），超时后移除。
- `tcpa_horizon: 10.0`：TCPA 预测窗口（秒）。
- `safety_factor: 1.1`：安全距离缩放因子。
- `os_radius: 0.3`：本船半径。

### TSProjectionLayer
- `track_timeout: 3.0`：目标船超时（秒），超时后从 costmap 移除。
- `enabled: True`

### Controller
- ALOS: `forward_dist: 2.0`, `gamma: 0.0006`, `max_angle_for_motion: 1.047`
- LOS: `lookahead_dist`（见各 params yaml）

## 五、常用诊断命令

```bash
# 检查 /tracked_ship 消息格式
ros2 topic echo /tracked_ship --once

# 检查 costmap 中 TS LETHAL 标记
ros2 topic echo /local_costmap/costmap --once

# 查询主威胁
ros2 service call /get_primary_threat nav2_colregs_msgs/srv/GetPrimaryThreat

# TF 检查
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map ts_virtual_base_link
```

## 六、目录结构（新增包）

```text
nav2_colregs_msgs/
nav2_colregs_vector_object_server/
nav2_colregs_costmap_layers/
  include/ src/ plugins.xml
nav2_colregs_ts_manager/
  include/ src/
nav2_colregs_local_path_bt_nodes/
  include/ src/
nav2_colregs_local_path_behavior/
  action/ include/ src/ behavior_plugin.xml
nav2_colregs_los_controller/
  include/ src/ los_controller_plugin.xml
nav2_colregs_alos_controller/
  include/ src/ alos_controller_plugin.xml
nav2_colregs_bringup/
  launch/ params/ behavior_trees/ maps/ worlds/ models/ rviz/ scripts/
```

## 七、Bug 修复记录

- **Keepout filter 动态跟随与拖影**：见 `doc/keepout_filter_bugfix.md`
- **VoxelLayer 远场残留**：`raytrace_max_range` 必须匹配 `obstacle_max_range`
- **StaticLayer 覆写动态层**：local costmap 必须移除 `static_layer`
