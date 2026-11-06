# LRAE-420 — Go2 + MID-360 的 LRAE 适配分支

本仓库是 [LRAE](https://github.com/NKU-MobFly-Robotics/LRAE)（Large-Region-Aware Exploration, IEEE RA-L 2024）的工程分支，用于 **Unitree Go2 + Livox MID-360** 在 Gazebo Classic 仿真和实机录制的 ROS bag 上的自主探索实验。

它不是论文复现：这里记录的是"在真实数据上哪里会坏、为什么坏、改了什么、改完还剩什么没解决"。每个改动都对应一个可观测的失败模式，并尽量给出改前/改后的测量值。

**当前状态一句话**：仿真和回放都能稳定跑完；"路径上的假障碍"和"拟合覆盖率"已有可量化的改善；但**全局地图的融合语义、unknown 在规划中的语义、2.5D 可通行面的高度语义**这三个算法层面的问题仍未解决，也是下一步的主要工作。

## 上游引用

Q. Bi, X. Zhang, S. Zhang, R. Wang, L. Li and J. Yuan, "LRAE: Large-Region-Aware Safe and Fast Autonomous Exploration of Ground Robots for Uneven Terrains," *IEEE Robotics and Automation Letters*, vol. 9, no. 12, pp. 11186–11193, 2024.

## 环境

- Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11
- `catkin build`（仓库以 catkin 工作空间的 `src/` 形式组织）
- 仿真渲染：CPU 射线可用；大场景建议 GPU 射线（`gpu_ray` + `gazebo_ros_velodyne_gpu_laser`，需要 NVIDIA 容器运行时）
- 实机回放需要：FAST-LIVO2 输出的 `/cloud_registered`、`/aft_mapped_to_init`、`/tf`、`/path`

## 快速开始

### A. Gazebo 校园仿真

```bash
roslaunch lrae_go2 lrae_go2_campus.launch gui:=true rviz:=true \
  mid360_sensor_type:=gpu_ray
```

关键默认值：出生点 `(173.8, -257.4, 5.85)`（山顶）、32 线、`fit_min_points=4`、`fit_radius=0.35`、`fit_z_half_extent=0.3`、邻域恢复开、`merge_size=20`、`min_occ_hits=2`。

### B. 实机 bag 回放（开环）

回放里机器人按 bag 里程计走，**我们的规划器不控车**；评价对象是地图和路径，不是"车走得稳不稳"。

```bash
# 1) master 必须显式 -p：roscore 忽略 ROS_MASTER_URI
roscore -p 11311 &
rosparam set use_sim_time true

# 2) 先放 bag，让 /clock 流动，再起节点（否则静态 TF 会用墙钟打戳）
rosbag play --clock -r 1.0 your.bag \
  --topics /cloud_registered /aft_mapped_to_init /tf /path &

# 3) 起 LRAE
roslaunch lrae_go2 lrae_replay.launch rviz:=true
```

`lrae_replay.launch` 内部做三件事：`mid360_to_world` 把点云从 `camera_init` 换到 `map`；静态 TF 补齐 `map→camera_init`、`aft_mapped→base_link`、`base_link→vehicle`；localPlanner/pathFollower 的 `/path` 重映射到 `/lrae_local_path`。

### C. 导出 bag 自身的建图（不含 LRAE）

用于判断"是 SLAM 的图不行，还是我们的 2.5D 图不行"：

```bash
python3 lrae_go2/scripts/bag_map_dump.py \
  --bag your.bag --out bag_map.pcd --voxel 0.15
```

它把 `/cloud_registered` 的所有帧按体素并起来写成一个 PCD，不起 roscore、不回放。

## 改进清单：每个改动面对什么问题

| # | 改进 | 针对的问题 | 实测证据 | 没解决的部分 |
|---|---|---|---|---|
| 1 | 初始化即融合 + 脚下自由区（`exploration_map_merge`） | 首帧不融合、机器人原地 6.4 m 后彻底冻结 | 修复后同一场景连续走 73 m，0 次停滞（1200 s 长测） | 只是补上死锁，不改善地图质量 |
| 2 | 拟合阈值参数化（`fit_min_points / fit_radius / fit_z_half_extent`） | 光滑开阔地面在 0.25 m 球内凑不够 10 个体素，整片 unknown | 校园广场从"零可通行格"到连续可通行带 | 阈值仍是经验值，没有按地形自适应 |
| 3 | 邻域补面（`neighbor_recover` + 共线/高差护栏） | 0.3 m 邻域与 0.1 m 体素环之间有空隙，地面出现条纹空洞 | 每帧恢复约 190–240 格 | 会跨断层外推，靠 `neighbor_max_z_span=0.15` 限幅 |
| 4 | viewpoint 规则放宽（`vp_free_neighbors_min=2`，`vp_occ_clear_cells=0`） | 窄隧道里"恰好 3–4 个自由邻居"永远不成立，frontier 候选长期为 0 | 修复前 10 min 停滞时 0 候选 | 贴墙候选变多，安全性依赖局部规划 |
| 5 | 近目标抑制（`goal_min_dis`、BFS 距离、目标迟滞） | 目标落在停车半径内，原地抖动 | 路径末端最小距离 1.2 m，抖动消失 | 只处理静止目标，不处理动态障碍 |
| 6 | 性能：栅格复用+量化、const-ref 占用图、闭合集复用、viewpoint 抽样 | 重建 14.4 ms/帧；真实数据上规划 p90 3.9 s | 重建 0.1 ms；规划 p90 42 ms；单帧 fit 8–14 ms | 尾延迟仍偶发 100 ms（`assess` 分支） |
| 7 | 卡死检测（`stuck_detector.py`） | 车停了但没人知道 | 记录 cmd/实际速度、位姿、前方障碍 | **只检测，没有恢复策略** |
| 8 | 障碍膨胀拆分（`occ_dilate` / `occ_dilate_local`） | 路径两侧成片黑格 | 路径 0.5 m 内黑格 5.49% → 0.48% | 见 4.1；全局图失去 0.3 m 安全边距 |
| 9 | 回放适配（TF、`use_sim_time`、独立端口） | 复现实验时 TF 外推 88139 s、master 冲突 | 外推错误归零，单 master | 回放仍是开环 |
| 10 | 多帧累积（`accum_frames`） | 点云稀疏导致 10 m 外几乎不建图 | free +34%，fit 数 ×2.2（见 4.2） | **不改善语义**，默认仍为 1 |
| 11 | 地面高度门（`ground_gate_height`） | 平的树冠/灌木顶通过 `flatness` 被判成可通行 | 见 4.3 | 是补丁不是模型，默认关闭 |

### 4.1 障碍膨胀为什么要拆成两个开关

`plane.cpp` 的后处理会把每个 occupied 格的 8 邻域自由格改写成 `99`，而 merge 用 `md >= 90` 判障碍，于是**每个障碍被放大 0.3 m**，路边 0.6 m 内的植被会糊到路面上。

实测（2026-11-05 的 720 s bag，前 120 s 轨迹，只切 `occ_dilate`）：

| | 路径 0.5 m 内黑格 | 路径 1.0 m 内 | 全局 occupied |
|---|---|---|---|
| 膨胀开 | 5.49% | 7.73% | 12,433 |
| 膨胀关 | **0.48%** | 1.74% | 5,491 |

但 `local_traversibility_ponit_cloud`（localPlanner 的障碍输入）是从同一张 `plane_OccMap` 生成的，直接关掉会把**局部**的 0.3 m 边距也去掉。所以拆成：

- `occ_dilate=false`：`/plane_OccMap` 和 `/globalMap` 保持几何真实；
- `occ_dilate_local=true`：近场障碍点云仍按 8 邻域外扩一圈。

验证：一张 `plane_OccMap` 里 `99` 的格子为 0、`100` 为 65；对应的 trav cloud 在 25×25 窗口内有 75 个障碍点——环存在。

### 4.2 多帧累积解决了覆盖，没解决语义

`accum_frames=3` 把最近 3 帧（已在 map 系）并起来再建体素栅格。实测（同一段 130 s）：

| 配置 | 全局 free | fit_success/帧 | 新增 free 中"抬高表面"占比 |
|---|---|---|---|
| 1 帧, 4 点 | 19,084 | 332 | — |
| 3 帧, 4 点 | 25,505 (+34%) | 745 | 18% |
| 1 帧, 3 点 | 20,229 | 355 | 22% |
| 3 帧, 3 点 | 26,369 | 743 | 19% |

结论：累积确实把密度和覆盖率拉上去了，但"抬高表面被当成可通行"的比例只从 22% 降到 18–19%。**瓶颈不是点数，而是判据里没有"离地面多高"这一项**。因此 `accum_frames` 默认保持 1，只作为可选实验项。

### 4.3 地面高度门是临时判据

`ground_gate_height` 用邻域已拟合格的中位数高度作参考，高出阈值就把该格降级为障碍，用来挡住"平树冠"。它能压掉一部分假自由，但参考高度靠邻域猜，坡面和台阶边缘会误伤。**默认 0（关闭）**，不建议作为最终方案；真正的解法是把地面建成高度场、把障碍定义为地面之上的净空（见下文"未解决"第 3 条）。

## 还没解决的问题（按对规划的影响排序）

1. **全局记忆的融合语义**
   现在 merge 用 `0.2*global + 0.8*local`（远处）和 `0.96/0.04`（1 m 内）混合可通行度分数，再用"连续 2 帧"计数写障碍。分数不是概率，混合后的值没有定义；`-1`（没观测）直接跳过，所以"没看到"被当成"没变化"。后果：判据随历史漂移、近处黑格几乎清不掉、走出视野的假障碍永久保留。

2. **规划器把 unknown 当自由**
   `path_planning.cpp:92-96` 把 `-1` 映射成 `0`（`planmap_` 是 `unsigned char`，A* 又是二值判据 `>=95` 封路），所以未知格和已知自由格在全局路径里完全等价。frontier 检测读的是原始 `-1`，不受影响；受影响的是 A* 的代价——它会径直穿过未观测区域。

3. **2.5D 可通行面没有高度语义**
   判据只有"平面倾角 < 40°"和"残差 < 0.01"。一块平的树冠、屋顶、货车顶都满足。需要把表示改成"地面高度场 + 地面之上的净空"，而不是单层平面拟合。

4. **输入点云稀疏**
   实机 bag 的 `/cloud_registered` 平均 7.3k 点/帧（MID-360 原始约 20k）。实测密度：0–10 m 12.6 pts/m²，10–20 m 2.1，20–30 m 0.2。拟合窗口和融合窗口又都是 ±20 m，所以**可靠建图半径约 10 m**，更远只剩 RViz 里的彩色点，不会变成地图。

5. **动态障碍**
   `min_occ_hits=2` 是"2 帧"（约 0.2 s）的计数，不是独立观测次数；行人/车辆仍可能留下永久障碍。

6. **碰撞模型名存实亡**
   `vehicleLength/Width` 只用于旋转检查，而 `checkRotObstacle=false`；平移避障靠候选弧与 2 cm 栅格，边距来自 `pathScale` 缩放，不是真实车体轮廓。

7. **只有卡死检测，没有恢复**
   `/stuck` 只发布状态，没有倒车/转向/目标黑名单。

8. **新默认缺少仿真长测**
   `occ_dilate=false + occ_dilate_local=true` 目前只在实机 bag 回放上验证过；校园 10 分钟长测（停滞次数、最小净距）尚未跑。

9. **回放是开环**
   bag 回放只能评价"地图和路径"，不能评价"通过性和探索效率"；后者要在仿真里跑。

## 参数速查

### fitplane（`Traversibility_mapping`）

| 参数 | 默认（仿真/回放） | 作用 |
|---|---|---|
| `fit_min_points` | 10 / 4 | 拟合所需体素数；3 点会让残差恒为 0，`flatness` 判据失效 |
| `fit_radius` | 0.25 / 0.35 | 拟合支撑半径 |
| `fit_z_half_extent` | 1.0 / 0.3 | 拟合盒纵向半高；越大越容易把树冠混进地面 |
| `fit_ransac` | false | 用主平面 + 外点剔除；实测在植被区反而更差 |
| `neighbor_recover` | true（campus/replay） | 邻域补面，带共线/高差护栏 |
| `map_cell_size` | 0.3 | 输出栅格，也是膨胀距离 |
| `grid_z_below/above` | 1.2 / 2.0 | 体素栅格纵向裁剪（另有 1 m 复用 margin） |
| `occ_dilate` | **false** | 地图侧 0.3 m 膨胀（上游行为为 true） |
| `occ_dilate_local` | **true** | 局部障碍点云的 0.3 m 膨胀 |
| `accum_frames` | 1 | 最近 N 帧并起来拟合 |
| `ground_gate_height` | 0 | 地面高度门，0=关 |

### exploration_map_merge

| 参数 | 默认 | 作用 |
|---|---|---|
| `merge_size` | 20 m | 只融合机器人 20 m 内的局部图 |
| `min_occ_hits` | 2 | 连续 2 帧 occupied 才写入 |
| `safe_obs_dis` | 1.0 m | 1 m 内用 0.96/0.04 慢融合 |
| `init_free_radius` | 2.1（campus）/ 0（replay） | 初始脚下自由区 |

### lrae_planner

| 参数 | 默认 | 作用 |
|---|---|---|
| `minrange` | 8 m | 目标过近则拒绝 |
| `vp_free_neighbors_min` | 2 | frontier 候选所需自由邻居数（且不允许有 occupied 邻居） |
| `vp_occ_clear_cells` | 0 | 额外的障碍净空环，0=关 |
| `goal_min_dis` | 0.7 m | 目标最小距离 |
| `max_assess_viewpoints` | 12 | 每次评估的 viewpoint 上限（控制尾延迟） |

## 评测口径

以下指标都在同一段 bag、同一条轨迹上算，便于横向对比：

- **路径黑格率**：globalMap 中距离记录轨迹 ≤0.5 m 的格子里，`>=90` 的比例；
- **抬高自由面比例**：新增 free 格中，"0.3 m 列内最低点高于局部地面（5×5 m 内 5% 分位）0.5 m"的比例；
- **覆盖率**：globalMap 中 free 格总数；
- **拟合质量**：`[fitplane]` 日志里的 `fit_success / free / occupied / neighbor_recovered`。

参考数据：2026-11-05 的 Go2+MID-360 720 s bag，取前 130 s 轨迹；几何基准用 `bag_map_dump.py` 导出的 0.15 m PCD。

## 仓库结构

```
lrae/
  fitplane/        2.5D 可通行图（改动最大）
  gen_local_goal/  全局路径前 3 m → look-ahead 目标
  local_planner/   局部规划/路径跟踪（CMU 版本，未改算法）
  lrae_planner/    探索决策与全局地图融合
lrae_go2/          Go2 + MID-360 适配层（launch / 回放 / 诊断脚本 / RViz）
docs/lrae_changes.patch   第一轮改动的 diff 存档（不含 replay、occ_dilate、accum_frames）
```

新增的适配与诊断脚本：

- `lrae_go2/launch/lrae_replay.launch` — 实机 bag 回放
- `lrae_go2/scripts/mid360_to_world.py` — 点云换到 map 系
- `lrae_go2/scripts/lrae_diag_logger.py` — 100 Hz 速度/位姿/覆盖率记录
- `lrae_go2/scripts/stuck_detector.py` — 卡死检测
- `lrae_go2/scripts/bag_map_dump.py` — bag 自身建图导出
- `lrae_go2/scripts/bag_viewer.py` — 纯 bag 可视化
- `lrae_go2/scripts/save_grid_once.py`、`probe_snapshot.py` — 地图快照与诊断

## 已知风险

- `occ_dilate=false` 去掉了全局图上的 0.3 m 安全边距。局部规划器仍有 `occ_dilate_local` 的一圈，但全局路径可能贴着障碍 0.3 m 规划；窄通道（0.6–0.9 m）里的行为尚未在仿真长测中验证。
- `accum_frames>1` 会让人/车等移动物体在 0.15–0.25 m 内拖影，默认保持 1。
- `ground_gate_height>0` 会误伤坡面和台阶边缘。

## License

GPL-3.0，与上游一致（见 `LICENSE`）。上游算法版权归原作者所有；本仓库的修改同样以 GPL-3.0 发布。
