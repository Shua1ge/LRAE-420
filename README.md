# LRAE-420 — Go2 + MID-360 适配版 LRAE

本仓库是 [LRAE](https://github.com/NKU-MobFly-Robotics/LRAE)（Large-Region-Aware Exploration，IEEE RA-L 2024）的修改版，用于 **Unitree Go2 + Livox MID-360** 在 Gazebo Classic 中的自主探索，重点修复了"开阔/光滑地面无法建出可通行地图、frontier 长期饥饿、执行卡死无检测"等问题。

相对上游的改动全部以**参数化**方式实现：所有新参数默认值与上游行为一致，不改变原始算法在原有场景（洞穴/崎岖地形）中的表现。

## 上游引用

Q. Bi, X. Zhang, S. Zhang, R. Wang, L. Li and J. Yuan, "LRAE: Large-Region-Aware Safe and Fast Autonomous Exploration of Ground Robots for Uneven Terrains," in *IEEE Robotics and Automation Letters*, vol. 9, no. 12, pp. 11186-11193, 2024.

## 环境

- Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11
- catkin（`catkin build`）
- 可选：NVIDIA GPU 射线（`gpu_ray` + `gazebo_ros_velodyne_gpu_laser`），用于大场景实时仿真

## 仓库结构

```
lrae/
  fitplane/        可通行地图（修改）
  gen_local_goal/  全局路径前 3 m → look-ahead 目标（未修改）
  local_planner/   局部规划/路径跟踪（未修改）
  lrae_planner/    探索决策（修改）
lrae_go2/          Go2 + MID-360 适配层（非上游，新增）
docs/lrae_changes.patch   修改记录（含 diff）
```

未包含上游的 `scout_simulator/`、`simworld/`、`image/`（本工程未使用）；`local_planner/paths/` 保留（`localPlanner` 启动时读取 `paths.ply`、`pathList.ply`、`startPaths.ply`、`correspondences.txt`）。

## 相对上游的改动

### fitplane（可通行地图）

1. **拟合阈值参数化** — `fit_min_points`（默认 10）、`fit_radius`（默认 0.25 m）。
   上游硬编码"拟合盒内 ≥10 个占用体素才做平面拟合"。光滑开阔地面每个 0.5 m 盒内只有 5–9 个地面体素，几乎全部格子保持 unknown。实测（校园广场、32 线、静止）：`surface_failed` 占窗口 **98.3%**，`free` 格 **0**。

2. **邻域地表恢复** — `neighbor_recover`（默认 false）、`neighbor_radius_cells`（默认 1）、`neighbor_min_points`（3）、`neighbor_min_eig_ratio`（0.1）、`neighbor_max_z_span`（0.15 m）。
   当精确体素列没有地表时，从邻域格的地表高度做局部平面拟合。两个安全守卫：点集共线（单条激光环）拒绝、邻域高度跨度超限（台阶/路缘）拒绝。
   注意：0.3 m 邻域几乎无效（地面回波是间距 0.8–1 m 的细环，单环共线被拒绝）——0.9 m（3 格）才跨两条环。

3. **逐帧诊断** — 发布 `/fitplane_diagnostics`（`std_msgs/Int32MultiArray`）：
   `[surface_failed, fit_insufficient, fit_success, free_cells, occupied_cells, window_cells, neighbor_recovered]`，
   并 1 Hz 打印 `[fitplane] fit_ms=...` 日志。统计窗口固定为机器人 ±20 m，保证跨帧可比。

### lrae_planner（探索决策）

1. **地图 bootstrap 修复** — `initGlobalMap()` 后立即执行一次 `mapMerge()`。
   上游的 merge 只在机器人移动 ≥0.1 m 后执行，而首次执行资格在初始化前就被消耗；Go2 起身晃动小于 0.1 m 时会永久死锁（无全局地图 → 无质心 → 不动）。

2. **视点条件参数化** — `vp_free_neighbors_min`（默认 3，等价上游"恰好 3–4 个 free 邻居"）、`vp_occ_clear_cells`（默认 2 等价上游 ±0.6 m 无墙检查，0 关闭）。
   窄隧道里墙面贴近 frontier 时，上游规则会产生 0 个视点（实测）。

3. **`go_home_` 解锁** — `getCentroid` 成功时清除锁存，避免一次失败后永久停机（上游锁存后无法恢复）。

4. **frontier 兜底 `publishFrontierFallback()`** — BFS 沿可通行格找最近"未知邻接可通行格"，在"无质心 / outwin 失败 / Replan"三个失败点发布路径；目标带滞回（`last_fb_goal_`）与可达性复查。

5. **目标距离下限 `publishExplorationPathFiltered()`** — 全局路径终点距机器人小于 `goal_min_dis`（默认 1.2 m）时拒绝发布、改用兜底目标。近目标会落进 pathFollower 的 `stopDisThre`(0.2 m) + look-ahead 死区，造成原地抖动。

6. **`init_free_radius`** — merge 初始化时在机器人周围种一块自由圆（默认 0，校园 2.1 m），覆盖 MID-360 近场盲区（−7° 下视 × 0.26 m 安装高度 ≈ 2.1 m）。

### lrae_go2（适配层，非上游）

- `scripts/mid360_to_world.py`：Gazebo 点云（`lidar_link`）→ `world` 系的 TF 变换（无过滤，fitplane 需要地面点）。
- `launch/lrae_newmine.launch`：LRAE 全栈参数（含上述全部参数，默认值保持上游行为）。
- `launch/lrae_go2_campus.launch`：NJUPT 校园世界一键启动（GPU 射线、出生点、全局图原点、启动自由圆）。
- `rviz/lrae_campus.rviz`：globalMap、全局/局部路径、点云、可通行点云、Go2 模型、TF。
- `scripts/lrae_diag_logger.py`：100 Hz 诊断 CSV（指令、实际速度、路径/目标几何、覆盖增长）。
- `scripts/stuck_detector.py`：执行卡死检测（指令持续但位移为零），日志给出 z/roll/pitch、前向障碍距离、指令/实际对比；发布 `/stuck`。
- `scripts/startup_spin.py`：可选启动自检（360° 旋转）。对 360° 雷达无建图收益，仅验证偏航控制。
- `scripts/rate_probe.py`：话题频率、消息尺寸、CPU 占用探针。

## 实验结果（可复现）

可通行地图（校园广场、静止、同一 ±20 m 窗口约 12.4k 格、32 线）：

| 配置 | surface_failed | fit_insufficient | free | occupied |
|---|---|---|---|---|
| 上游 10/0.25 | 12197 (98.3%) | 171 | **0** | 31 |
| 10/0.50 | 12335 | 95 | 51 | 65 |
| 邻域 0.3 m + 10/0.25 | 12318 | 171 | 11 | 32 |
| 邻域 0.6 m + 10/0.25 | 12097 | 169 | 119 | 32 |
| 邻域 0.9 m + 10/0.25 | 11901 | 157 | 315 | 31 |
| **邻域 0.9 m + 4/0.35** | 11933 | 45 | **451** | 110 |
| 128 线（仿真加密）+ 邻域 0.3 m | 11612 | 61 | 561 | 188 |

结论：`surface_failed` 的主体是"该格 ±0.9 m 内从未被激光扫到"（88.4%），不是拟合阈值；邻域恢复只有在半径跨过两条地面环（≥0.6 m）时才有效。

动态验证（32 线 + 邻域 0.9 m + 4/0.35，90 s）：RTF 0.93，行走 27 m，`Exploring ...` 正常，globalMap 5770 free + 5161 occupied。

## 构建与运行

```bash
catkin build fitplane gen_local_goal local_planner lrae_planner lrae_go2
```

LRAE 侧接口约定（详见 `lrae_go2/launch/`）：

- `/mid360_points`（`lidar_link`）→ `mid360_to_world.py` → `/mid360_world`（`world` 系）→ fitplane
- `/state_estimation` 重映射到 `/ground_truth/state`（仿真）或 LIO 里程计（实机）
- `/cmd_vel` 重映射到 `/planning/cmd_vel_raw`（机器人底盘/策略）
- 静态 TF：`map→world`、`base→base_link`、`base→vehicle`

## 已知限制

- **近场盲区 0–2.1 m**：由 −7° 下视 FOV 与安装高度决定，软件无法完全消除；`init_free_radius` 只解决启动阶段。
- **拟合参数与扫描模式相关**：本仓库参数在 Gazebo 固定栅格下标定；真实 MID-360 为非重复扫描，需重新标定 `fit_*` / `neighbor_*`。
- **无执行恢复**：`stuck_detector` 只检测并发布 `/stuck`，恢复动作（后退+转向、目标黑名单）尚未实现。
- **全局地图越界无保护**：`map_origin` / `map_size` 必须覆盖任务区域，否则 merge 会越界写。

## License

GPLv3，继承上游 LRAE。
