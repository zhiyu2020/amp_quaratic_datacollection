# legged_mpc_amp

基于 OCS2 NMPC + WBC 的四足机器人运动控制和 AMP 运动数据生成项目。当前已在 `ros1_noetic_amp` 容器内完成 Go2 的全量编译、Gazebo 初始化、MPC 控制器启动、stance/trot 低速前进、CSV 录制以及 IsaacLab `.npz` 转换验证。

## 运行逻辑

```text
Gazebo 传感器/关节状态
        ↓
状态估计 → OCS2 NMPC（状态、接触时序、足端轨迹）
        ↓
WBC/QP（关节位置、速度、力矩）
        ↓
gazebo_ros_control → Go2
        ↓
AMP logger（CSV）→ IsaacLab 转换器（NPZ/NPY）
```

启动时 Gazebo 保持暂停，Go2 通过 `spawn_model -J` 以站立关节角生成。控制脚本确认两个 controller 已加载后才解除暂停并严格启动，避免机器人在 MPC 接管前自由落体。

## 已验证环境

- 容器：`ros1_noetic_amp`
- Ubuntu 20.04 / ROS Noetic / Gazebo 11
- Pinocchio 3.3.0、HPP-FCL 2.4.5
- 机器人：Unitree Go2
- 宿主机目录：`/home/mscape/ros2/amp/legged_mpc_amp-master`
- 容器目录：`/root/amp/legged_mpc_amp-master`

## 编译

进入容器：

```bash
docker exec -it ros1_noetic_amp bash
cd /root/amp/legged_mpc_amp-master
```

首次使用安装系统依赖并编译：

```bash
apt update
apt install -y python3-catkin-tools python3-rosdep libeigen3-dev \
  libboost-all-dev liburdfdom-dev \
  ros-noetic-pinocchio ros-noetic-hpp-fcl

source /opt/ros/noetic/setup.bash
bash setup.sh
```

以后每个新终端只需：

```bash
cd /root/amp/legged_mpc_amp-master
source env.sh go2
```

## 手动运行 Go2 MPC

终端 1，启动 Gazebo（默认以暂停状态生成站立 Go2）：

```bash
cd /root/amp/legged_mpc_amp-master
source env.sh go2
roslaunch legged_robot_description empty_world.launch gui:=true
```

终端 2，加载控制器和键盘控制：

```bash
cd /root/amp/legged_mpc_amp-master
source env.sh go2
roslaunch legged_controllers keyboard_control.launch \
  enable_amp_logging:=true \
  amp_log_dir:=$(pwd)/amp_data/manual_go2
```

按键顺序建议：

1. 按 `i`：等待 controller 加载，解除暂停，启动 MPC 并保持 stance。
2. 看到 `Initial policy has been received.` 后按 `1`：切换 trot。
3. 等待约 2 秒，再按 `w`：每次增加 `0.1 m/s`，先从 `0.2 m/s` 验证。
4. 按空格清零速度；按 `0` 回到 stance。
5. 按 `l` 开始/停止 AMP 录制。

其他按键：`s` 后退，`q/e` 左右平移，`a/d` 左右转向，`k` 停止控制器，`?` 显示帮助。

## 自动运行与采集

## 导出兼容 amp_go2 的 61 维 AMP JSON

`record_flat_gaits_amp.py` 从当前机器人 `gait.info` 读取步态定义，并使用
`/ground_truth/state`、`/joint_states` 和 TF 录制与
`amp_go2/datasets/record_legged_control_amp.py` 相同的 61 维帧格式。先按“手动运行
Go2 MPC”的前两步启动 Gazebo 和控制器（按 `i` 启动 controller），再在第三个终端执行：

```bash
cd /home/mscape/ros2/amp/legged_mpc_amp-master
source env.sh go2

# 前进 1.0 m/s，5 秒
rosrun legged_controllers record_flat_gaits_amp.py \
  --out datasets/go2_motion/go2_forward.txt --gait trot --rate 50 --duration 5 \
  --cmd-vel 1.0 0.0 0.0 --zero-root-xy

# 原地左转 0.6 rad/s，5 秒
rosrun legged_controllers record_flat_gaits_amp.py \
  --out datasets/go2_motion/go2_turn_left.txt --gait trot --rate 50 --duration 5 \
  --cmd-vel 0.0 0.0 0.6 --zero-root-xy
```

列出或依次录制 `gait.info` 中的全部 Go2 步态：

```bash
rosrun legged_controllers record_flat_gaits_amp.py --list-gaits
rosrun legged_controllers record_flat_gaits_amp.py \
  --gait all --out-dir datasets/go2_flat_gaits --rate 50 --duration 5 \
  --cmd-vel 0.3 0.0 0.0 --zero-root-xy
```

全步态模式为每个步态分别写出 `go2_<gait>.txt`，会在各步态间停零并等待默认 3 秒。
跳跃类步态（例如 `lindyhop`、`skipping`、`pawup`）请先使用低速或零速度小段验证，必要时
通过 `--gait-warmup` 延长稳定时间。

下面的脚本会自动启动无界面 Gazebo、等待 controller 真正加载、运行 stance/trot 速度序列并写入 CSV：

```bash
cd /root/amp/legged_mpc_amp-master
source env.sh go2
GAZEBO_GUI=false bash scripts/collect_trot_amp_data.sh
```

需要查看 GUI：

```bash
GAZEBO_GUI=true GAZEBO_HEADLESS=false bash scripts/collect_trot_amp_data.sh
```

默认输出目录为 `amp_data/auto_trot_YYYYmmdd_HHMMSS/`，其中：

- `*.csv`：50 Hz 的根状态、关节状态和足端接触数据；
- `collection_manifest.json`：每个 sequence 的 gait、速度和持续时间。

可用环境变量覆盖机器人、目录和文件名前缀：

```bash
ROBOT_TYPE=go2 \
AMP_LOG_DIR=$(pwd)/amp_data/my_go2_run \
AMP_LOG_PREFIX=go2_trot \
bash scripts/collect_trot_amp_data.sh
```

## 转换为 IsaacLab AMP 数据

```bash
python3 scripts/convert_amp_data_isaaclab.py \
  --input_dir amp_data/my_go2_run \
  --output_dir amp_dataset_isaaclab/my_go2_run \
  --save_individual_npz
```

主要输出：

```text
amp_dataset_isaaclab/my_go2_run/isaaclab_motions.npz
amp_dataset_isaaclab/my_go2_run/isaaclab_motion_metadata.json
amp_dataset_isaaclab/my_go2_run/sequences/*.npy
```

转换器默认把 OCS2 的腿顺序 `[LF, LH, RF, RH]` 重排为 IsaacLab 常见的 `[FL, FR, RL, RR]`。`motions` 默认每帧 40 维，包含 root position/RPY、机体坐标系线速度和角速度、12 维关节位置、12 维关节速度及 4 维接触状态。训练采样不要跨越 `sequence_lengths` 边界。

## 本次端到端验证结果

隔离端口、无界面测试使用 `stance 2 s + trot 0.2 m/s 5 s`：

- controller：`joint_state_controller=running`、`legged_controller=running`；
- MPC 初始策略正常收到，无 Safety、NaN 或求解失败；
- 运动结束基座高度 `0.335 m`，姿态接近水平，沿 x 前进约 `1.44 m`；
- AMP CSV：610 帧、42 列、数值列无 NaN/Inf；
- IsaacLab `motions`：`(610, 40)`、`float32`、50 Hz、无 NaN/Inf。

验证产物位于：

```text
amp_data/validation_go2_handshake/
amp_dataset_isaaclab/validation_go2_handshake/
```

## 常见问题

- `libGL ... nouveau`：容器没有正确获得 NVIDIA/OpenGL 时会出现。无界面运行使用 `gui:=false headless:=true`；GUI 运行需要把显示和 GPU 设备正确传入容器。
- 模型生成后趴下：不要手动提前解除暂停。等待控制器终端启动后按 `i`，脚本会先确认 controller 已加载。
- 没有 `/legged_robot_mpc_observation`：执行 `rosservice call /controller_manager/list_controllers`，确认两个 controller 都是 `running`。
- 自动采集时不要同时运行另一个 `/cmd_vel` 发布器，否则多个命令源会互相覆盖。

更多说明见 [AMP 数据采集指南](docs/amp_data_guide.md)、[编译指南](docs/build_guide.md) 和 [开发验证日志](docs/DEVELOPMENT_LOG.md)。

## License

项目基于 `QiayuanLiao/legged_control` 扩展，采用 BSD 3-Clause 协议；第三方组件信息见 `THIRD_PARTY_NOTICES.md`。
