# LRAE on Unitree Go2 + MID-360 (ROS 1 Noetic / Gazebo Classic)

> 包级说明。**实验状态、每个改动针对的问题、尚未解决的问题以根目录 `README.md` 为准**；
> 本文件保留接口与参数细节。

This package connects the LRAE exploration stack to the existing Go2 Noetic
simulation in `~/legged_exploration_bench/env/ros1_noetic/ws`.

The robot, MID-360 model and RL policy stay in `go2_description`; LRAE only
replaces the exploration/local-planning/velocity-following layer.  The Go2
policy boundary is `/planning/cmd_vel_raw`.

## Files

```
lrae_go2/
  launch/lrae_newmine.launch       # LRAE nodes, assumes robot side is running
  launch/lrae_go2_newmine.launch   # robot side + LRAE in one launch
  scripts/mid360_to_world.py       # /mid360_points -> /mid360_world (TF only)
```

## Build

```bash
cd ~/legged_exploration_bench/env/ros1_noetic/ws
docker run --rm --network host -v "$PWD:/ws" -w /ws gbplanner_ros1_go2:latest \
  bash -lc 'source /opt/ros/noetic/setup.bash; catkin build fitplane gen_local_goal local_planner lrae_planner lrae_go2'
```

## Run

One-shot (NewMine):

```bash
roslaunch lrae_go2 lrae_go2_newmine.launch gui:=false start_delay:=12
```

Robot side separately:

```bash
roslaunch go2_description go2_stand_walk.launch \
  world:=/ws/worlds/newmine/NewMine_classic.sdf \
  use_mid360:=true use_d435:=false use_filter:=false gui:=false
# wait 10-15 s for the policy stance
roslaunch lrae_go2 lrae_newmine.launch start_delay:=15
```

## Campus

`lrae_go2_campus.launch` uses the CMU campus world plus a physical ground plane:

```bash
roslaunch lrae_go2 lrae_go2_campus.launch gui:=true rviz:=true \
  mid360_sensor_type:=gpu_ray
```

Current campus spawn is the hill summit `(173.8, -257.4, 5.85)` (ground z 5.30);
override with `x:=/y:=/z:=`.  The world is a photogrammetry mesh, so the launch
adds a ground plane for the physics body while keeping the mesh for LiDAR and
rendering.  With GPU rays the run is roughly real time; CPU rays are ~0.3x.

## Interface

| LRAE | Connected to |
|---|---|
| `pathFollower` `/cmd_vel` | `/planning/cmd_vel_raw` (Go2 policy) |
| `pathFollower` `/state_estimation` | `/ground_truth/state` |
| `localPlanner` `/state_estimation` | `/ground_truth/state` |
| `fitplane` `PointCloud_Map_topic` | `/mid360_world` |
| `lrae_planner` TF | `map -> base_link` via `map->world`, `world->base`, `base->base_link` |

`/mid360_points` is in `lidar_link`; `mid360_to_world.py` transforms it using
TF and publishes `/mid360_world`.  It does **not** filter: `fitplane` needs the
ground points.  `mid360_filter.py` is for GBPlanner occupancy and must not be
inserted here.

## Important parameters

- `minrange` (default 3.0): centroid rejection radius.  In NewMine's ~8 m wide
  tunnels, 8.0 allowed only a thin 8-10 m annulus and the planner reported
  "almost no explorable unknown regions".  3.0 keeps the local corridor
  frontiers.  For open scenes raise it to 8-20.
- `map_width`/`map_height` (1000) and `map_origin_x/y` (-20): global grid at
  0.3 m/cell = 300 m, origin -20 m.
- `maxSpeed`/`autonomySpeed` (0.5), `maxAccel` (0.5), `maxYawRate` (45 deg/s):
  matched to the Go2 policy's measured response.
- `start_delay` (15 s; 12 s in the one-shot launch): keeps LRAE from commanding
  during the policy stand-up.
- `gen_local_goal`'s `/cmd_vel` is remapped to `/cmd_vel_disabled`; its built-in
  0.5 m/s "stuck" open-loop nudge must not reach the policy.

## First-run evidence (2026-11-03)

- Build: all 8 packages succeeded.
- Flat `empty.world`: 0.5 m/s command, 52-pose path, robot at (9.35, 8.75) after
  34 s.  Sensor/TF/planner/policy loop works.
- NewMine, `minrange=8`: robot moved to (7.84, 1.19) then `/exporation_path`
  stopped; repeated "almost no explorable unknown regions".
- NewMine, `minrange=3`: path 37 poses at 37 s, robot at (3.63, -1.18); at 70 s
  robot at (8.70, 1.12), path 21 poses, look-ahead goal (11.05, 3.55).  It keeps
  exploring, with transient "current window has no centroid" messages.
- Base-only policy test: yaw-only 0.5 rad/s turned ~158 deg in 3 s; forward+yaw
  0.3/0.3 moved ~1.5 m in 3 s.  The policy is not limited to forward-only
  motion; the first smoke run's stall at a 0.2 m look-ahead goal was a local
  path/endpoint or geometry effect, not a pure yaw limitation.

## Known issues / next checks

1. NewMine progress is slow (~0.15-0.2 m/s).  Check whether this is policy
   caution on the cave ground, localPlanner speed limiting, or command mixing.
2. `plane_OccMap` ego cell can be `-1` (unknown) and the local map is only
   ~37 x 8 m in the corridor.  If stalling recurs, inspect
   `local_traversibility_ponit_cloud` for walls and consider increasing
   `vehicleWidth`/`adjacentRange`.
3. `/exporation_path` pose orientations look like non-normalized quaternions
   (e.g. w=-1.57).  `pathFollower`/`gen_local_goal` use positions, so this is
   currently harmless; do not rely on path orientation.
4. If the robot stalls again with yaw-only commands, add a small command shaper
   that injects a minimum forward speed while turning, gated by MID-360 forward
   clearance.  Do not add it blindly: a pure turn-in-place may be intentional
   in tight space.
5. The next real-robot step is the same LRAE launch with
   `/LIO/odom_vehicle` + `/LIO/clouds_lidar` instead of ground truth and
   `/mid360_points`; FAST-LIO is not part of this simulation.
