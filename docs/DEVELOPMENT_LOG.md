# 开发与验证日志

本文档记录 `legged_mpc_amp-master` 在 `ros1_noetic_amp` Docker 容器内的打通过程。所有测试均以 Go2 和项目自带的 OCS2、qpOASES、Gazebo 硬件插件为基准。

## 2026-09-22

### 环境检查

- 宿主机目录：`/home/mscape/ros2/amp/legged_mpc_amp-master`
- 容器目录：`/root/amp/legged_mpc_amp-master`
- 容器：`ros1_noetic_amp`
- ROS：Noetic
- 构建工具：`catkin_tools`
- 测试开始前未发现遗留的 ROS、Gazebo 或 controller spawner 进程。

### 构建问题 1：catkin 误用父目录工作空间

执行 `bash setup.sh` 时，catkin 将 `/root/amp` 识别为工作空间，并报错：

```text
[build] Error: Unable to find source space `/root/amp/src`
```

原因是父目录存在 `/root/amp/.catkin_tools`，而原脚本调用 `catkin config` 和 `catkin build` 时没有固定当前工作空间。仅增加 `--workspace` 仍会被 catkin 解释为“父工作空间内的子目录”；因此脚本会先建立当前项目的 `.catkin_tools` 标记，再显式传入 `--workspace "${PROJECT_DIR}"`，确保构建产物固定生成在当前项目的 `build/`、`devel/` 和 `logs/` 下。

### 后续验证

- [x] 完整编译
- [x] Gazebo 无界面启动与模型生成
- [x] 控制器加载及 MPC observation
- [x] stance 初始化稳定性
- [x] trot 低速前进运动
- [x] AMP CSV 数据写入

### 构建问题 2：缺少 Pinocchio / HPP-FCL

首次进入实际编译后，20/34 个包通过，`ocs2_pinocchio_interface` 在 CMake 阶段因找不到 `pinocchio.pc` 失败，其余 13 个下游包被跳过。检查确认容器中没有系统安装，只有旧 `/root/legged_ws` 私有构建产物。为避免跨工作空间链接，未复用旧产物，改为安装同一 ROS Noetic 软件源中的：

- `ros-noetic-pinocchio` 3.3.0
- `ros-noetic-hpp-fcl` 2.4.5
- 自动依赖 `ros-noetic-eigenpy` 3.8.2

安装成功，等待增量编译验证 API 兼容性。

### 完整编译结果

- 33/33 个包构建成功，无失败或跳过包。
- 总耗时约 3 分 55 秒（首次全量构建）。
- Pinocchio 3.3 与项目内 OCS2 API 编译兼容；仅有弃用提示，不影响产物生成。

### 运行问题 1：控制器启动前机器人已经倒地

按原默认参数启动无界面 Gazebo，等待自动脚本默认的 8 秒后读取 `/gazebo/get_model_state`：Go2 基座高度已从生成高度 `0.5 m` 降到约 `0.057 m`，确认机器人在控制器初始化前自由落体并倒伏。

修复启动握手：

1. Gazebo 默认暂停物理；
2. `spawn_model` 保留并显式使用 12 个 `-J` 参数，以站立关节角生成，基座高度改为 `0.32 m`；
3. 键盘和自动初始化程序仅在 controller manager 服务已就绪后解除暂停，并立即请求启动 MPC 控制器；
4. 自动采集脚本默认使用 `GAZEBO_PAUSED=true`。

等待重新编译与运行验证。

### 运行问题 2：暂停仿真时自动初始化死锁

第一轮端到端测试中，模型正确保持在 `z=0.32 m`，但自动采集器超时且没有 MPC observation。原因是其在控制器切换前调用 `set_amp_recording(False)`，内部使用 `rospy.Rate` 和 `rospy.sleep`；当 `/use_sim_time=true` 且 Gazebo 暂停时，`/clock` 固定在 0，等待不会结束，解除暂停逻辑无法执行。

已调整初始化顺序：先调用 `switch_on_controller()`（内部解除 Gazebo 暂停并立即切换控制器），再执行 AMP topic 发布和依赖仿真时间的等待。

### 运行问题 3：切换请求早于控制器加载完成

第二轮隔离测试中，自动脚本在仿真时间 `0.005 s` 调用 `switch_controller` 并收到成功响应，但完整 Gazebo 日志显示 `legged_controller` 直到约 `0.504 s` 才完成初始化。原请求使用 `BEST_EFFORT`，因此目标控制器尚不存在时请求仍可返回成功，后加载的控制器一直停在 `initialized`，机器人随即倒地，也不会发布 MPC observation。

已在自动采集器和键盘控制器中加入 `/controller_manager/list_controllers` 握手：必须确认两个控制器均加载后才解除物理暂停，并将启动请求改为 `STRICT`，让控制器缺失或启动失败显式报错。

### 端到端验证通过

在独立的 ROS master `11331` 和 Gazebo master `11365` 上执行无界面验证，不影响已有 GUI 会话。测试序列为 stance `2 s`，然后以 trot、`vx=0.2 m/s` 前进 `5 s`。

- 自动采集器退出码：0；
- `joint_state_controller`、`legged_controller` 最终状态均为 `running`；
- 初始策略在仿真时间 `0.007 s` 收到；
- 结束时 Go2 基座位置约 `(1.444, -0.066, 0.335) m`，四元数约 `(-0.0012, 0.0051, 0.0129, 0.9999)`，未倒伏；
- 日志未出现 Safety、NaN、异常或 MPC/QP 求解失败；
- CSV 共 610 帧、42 列、数值列无 NaN/Inf，平均记录频率约 `48.46 Hz`；
- IsaacLab 转换结果 `motions.shape=(610, 40)`、`float32`、标称 `50 Hz`，所有数值有限。

验证产物：

```text
amp_data/validation_go2_handshake/validation_0000.csv
amp_data/validation_go2_handshake/manifest.json
amp_dataset_isaaclab/validation_go2_handshake/isaaclab_motions.npz
amp_dataset_isaaclab/validation_go2_handshake/isaaclab_motion_metadata.json
```
