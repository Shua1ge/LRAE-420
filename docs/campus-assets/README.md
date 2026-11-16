# GO2 仿真策略与 Campus 场景

这份快照对应 ROS Noetic、Gazebo Classic 11 的 GO2 + MID360 仿真。策略权重位于 `go2_description/models/go2_rl_lab_policy.pt`，ROS 节点位于 `go2_description/scripts/go2_policy_node.py`。本次 Campus 测试的控制配置见 `lrae_go2/launch/go2_stand_walk_mount_fixed.launch`。策略以 50 Hz 推理，关节 PD 以 500 Hz 更新，kp 25、kd 0.5、action_scale 0.25。命令缩放为 `[0.5, 1.0, 1.0]`。机器人模型、控制器加载器、关节参数和 MID360 安装参考随代码提供。

场景入口为 `lrae_go2/worlds/njupt_go2.world`，物理步长 0.002 s、更新率 500 Hz。它通过 `model://njupt_xianlin_optimized` 加载校园。完整模型含 1,240 个 OBJ、237 个 STL、218 张 PNG、材质及模型描述，共 1,699 个文件。模型归档保留 visual、collision、materials/scripts 和 materials/textures 的原始相对路径。 [下载 Campus 模型](https://github.com/Shua1ge/LRAE-420/releases/download/go2-campus-assets-20261116/njupt_xianlin_optimized.tar.zst)

## 解包和校验

从本分支或对应发布标签的仓库根目录执行。需要 curl、zstd 以及 GNU tar。

```bash
repo_dir="$PWD"
asset_dir="${XDG_DATA_HOME:-$HOME/.local/share}/lrae-campus"
mkdir -p "$asset_dir/models"
curl -fL --retry 3 -o "$asset_dir/njupt_xianlin_optimized.tar.zst" \
  https://github.com/Shua1ge/LRAE-420/releases/download/go2-campus-assets-20261116/njupt_xianlin_optimized.tar.zst
(cd "$asset_dir" && sha256sum --check "$repo_dir/docs/campus-assets/archive.sha256")
tar --zstd -xf "$asset_dir/njupt_xianlin_optimized.tar.zst" -C "$asset_dir/models"
(cd "$asset_dir/models" && sha256sum --quiet --check "$repo_dir/docs/campus-assets/model-files.sha256")
export GAZEBO_MODEL_PATH="$asset_dir/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

## 加载世界、机器人和底层策略

在含本仓库 catkin 包并已 source 的 ROS1 工作区中执行。沿用现有仿真环境中的 `torch`、`xacro`、Gazebo ROS Control、effort controllers、`smb_description`、`lidar_description` 和 MID360 对应的 GPU 射线插件。在容器中运行时，世界文件和模型目录都要从容器内可见。

```bash
roslaunch lrae_go2 go2_stand_walk_mount_fixed.launch \
  world:="$(rospack find lrae_go2)/worlds/njupt_go2.world" \
  x:=173.8 y:=-257.4 z:=5.85 gui:=true \
  use_mid360:=true use_d435:=false lidar_mount_reference:=official_imu \
  mid360_sensor_type:=gpu_ray mid360_plugin:=gazebo_ros_velodyne_gpu_laser \
  mid360_h_samples:=625 mid360_v_samples:=32
```

此入口加载仿真和 12 关节策略，速度输入为 `/planning/cmd_vel_raw`。要使用仓库已有的探索入口，可给 `lrae_go2_campus.launch` 传入相同的 `world:=...`。两套入口的传感器安装配置分别以各自的 launch 为准。策略文件 SHA-256 为 `bde5210a8114aa08bb0c611e49def1482faa1b2441673e5681374c30b935e32e`。
