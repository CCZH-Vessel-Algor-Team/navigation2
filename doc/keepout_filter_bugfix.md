# Keepout Filter Bug 修复记录

## 现象

TS 移动时，global costmap 上的动态 keepout 图形：

1. **不跟随** TS 位置更新（仅首次设定 pose estimate 时出现，或 TS 进入 local costmap 时出现）。
2. 移动中产生**"拖影"**——TS 离开后旧位置 keepout 残留，未被清除。

## 根因

Upstream `keepout_filter.cpp` 中两处设计缺陷：

### A. 更新窗口定位错误（`updateBounds()`）

`maskCallback` 收到新 mask 后，`updateBounds` 用 `mapToWorld(0, 0)` 和 `mapToWorld(width, height)` 计算 mask 覆盖的世界范围，而非用 mask 自身的 `origin`。

当 mask origin 不在 global costmap 原点时（如动态 TS keepout），`mapToWorld(0, 0)` 返回的是 global costmap 左下角世界坐标，与实际 mask 区域完全错位，导致 global costmap 认为"更新窗口内没有变化"，不触发重绘。

### B. 旧区域未纳入重置窗口（`updateBounds()` + `maskCallback()`）

costmap 更新机制是两步：

1. 主 costmap 重置更新窗口内所有 cell 为 FREE_SPACE。
2. 各 layer 的 `process()` 向窗口内写入新障碍值。

`process()` 只写不擦。如果两次 `updateBounds` 之间 `maskCallback` 被多次触发（TS 高速移动、mask 发布频率高），中间途径的位置未被纳入任何一次更新窗口，旧 keepout 值永不重置，形成拖影。

## 解决思路

### 第一轮：修复更新窗口定位

在 `maskCallback` 中保存 mask 的 world origin (`mask_origin_x_/y_`)；`updateBounds` 直接使用该 origin 计算更新范围，替代 `mapToWorld(0, 0)`。

### 第二轮：修复拖影

引入"累积并集"机制：

- 每次 `maskCallback` 收到新 mask 时，将**即将被替换的旧 mask** 世界范围用 `min`/`max` 扩张并入累计并集。
- `updateBounds` 消费时，一次性将累计并集纳入更新窗口（确保所有中间位置被 costmap 重置），然后清零 `has_cumulative_bounds_`。

方案实质：窗口大小由 costmap 更新周期自然决定，无需手动设定固定步数；永不遗漏，仅一份并集（4 个 `double`）。

## 文件变更

| 文件 | 变更说明 |
|---|---|
| `nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/keepout_filter.hpp` | 新增 `mask_origin_x_/y_`、`has_cumulative_bounds_`、`cumulative_min/max_x_/y_` 成员 |
| `nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp` | `maskCallback`：保存 mask origin + 扩展累计并集 |
| | `updateBounds`：使用 mask origin 定位窗口 + 纳入累计并集后清零 |
