# 本机开发环境

所有命令从 `intramural/` 执行。开发使用独立 Docker 镜像，基础环境是 Ubuntu 22.04、ROS 2 Humble、Ignition Fortress（Gazebo 6）；不在 WSL Ubuntu 26.04 中安装 Humble。

## 构建与检查

```bash
python3 scripts/prepare-sources.py
scripts/build-image.sh
scripts/build-workspace.sh
scripts/check-environment.sh
```

`prepare-sources.py` 根据 `docker/sources.lock.json` 恢复固定的公开源码提交；已有目录版本不符或存在预期补丁之外的改动时停止，不覆盖。受校验的公开修复见 `patches/README.md`。导航主仓库的子模块版本由它的 Git 提交固定。

镜像固定基础镜像 digest，`small_gicp` 固定源码提交，`xmacro` 固定 Python 包版本。APT 与 rosdep 使用在线仓库；实际安装版本保存在阶段验证记录中，未来在线仓库更新后依赖可能变化，当前方案不保证逐字节相同的镜像重建。

当前默认镜像为 `intramural-humble:stage2`，显式更新 rclcpp 及 action / components / lifecycle 配套库，主版本要求 16.0.21。历史阶段 1 镜像保留；可用 `INTRAMURAL_IMAGE` 指定镜像进行诊断，不将旧镜像结果作为当前导航验收。

工作空间编译使用 Release、每包最多 2 个编译任务、同时构建 2 个包，默认关闭上游测试构建。此设置是环境编译验证，不代表上游所有单元测试已通过。

## 使用

进入已加载 ROS 和工作空间的 Bash：

```bash
scripts/run.sh
```

执行单条命令，无须先进入容器：

```bash
scripts/run.sh ros2 pkg list
scripts/run.sh ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py --show-args
```

打开 RViz 或上游仿真器：

```bash
scripts/rviz.sh
scripts/simulator.sh
```

在启动终端按 `Ctrl-C` 退出。脚本使用临时容器、初始化进程和当前用户 UID/GID；退出后移除该容器，源码和构建结果仍保留。

`simulator.sh` 使用本地 `launch/simulator.launch.py`：分别由本地进程管理器启动 Gazebo 服务端 / GUI，以及复用上游的机器人生成 / 裁判入口。退出按服务端、GUI、ROS 节点的顺序执行，避免传感器和时钟回调与 ROS 发布者销毁竞争；子进程异常明确记录。公开上游的必要修复以独立补丁记录。

完整导航使用 `scripts/navigation.sh`，自动获取并校验公开先验点云，启动传感器定位和 Nav2；验收入口为 `scripts/verify-stage2.sh`。详见 [第二阶段验收](../debug-log.md)。完整导航退出时先退出 RViz，再在时钟仍运行时关闭 Nav2 生命周期节点，最后关闭仿真发布源和剩余导航节点；单独打开仿真器不能代替导航验收。

## 图形与运行隔离

- `run.sh` 自动挂载当前 WSLg 的 X11、Wayland 和音频接口；存在 `/dev/dxg` 时提供 WSL GPU 接口与相关库。
- 原生 Linux 桌面挂载当前 X11/Xwayland socket 和只读 Xauthority；Intel GPU 仅映射现有 `/dev/dri/renderD*` 字符设备及其实际组。无桌面环境不在本轮部署范围内，实际 NUC 渲染器仍须现场检查。
- 不使用全局 `xhost +`、特权容器或整目录 `/dev` 挂载。
- 默认使用 ROS domain 71 和 `ROS_LOCALHOST_ONLY=1`，便于多个终端之间通信，并与默认 ROS domain 隔离。可用 `INTRAMURAL_ROS_DOMAIN_ID`（0..232）及 `INTRAMURAL_IGN_PARTITION` 设置现场分配值；所有本任务终端统一配置并用脚本进入。
- 同时使用主机网络与主机 IPC，让 Fast DDS 在不同容器间共享实际通信内存；仅共享网络会造成 topic 可发现但消息无法到达。ROS domain 与 Ignition partition 用于通信隔离，主机 IPC 本身不构成安全隔离。
- `config/fastdds-services.xml` 同时固定服务回包等待和点云传输缓冲：32 MiB SHM 段、2 MiB 最大消息，自定义 UDP 只使用回环地址；普通话题 QoS 不变。约 1 MB 点云在 GUI 场景实收 20 Hz，证据见[模型接入记录](../debug-log.md)。
- Ignition 使用 `IGN_IP=127.0.0.1`、`IGN_PARTITION=srm_intramural`，让本机服务端、GUI 和命令行工具使用同一通信配置，避免默认接口选择导致 GUI 找不到服务端。
- 当前用户拥有容器产生的文件。HOME 指向容器内 `/work/cache/home`，不会写入宿主用户的应用配置。
- 实车串口只在显式设置 `INTRAMURAL_SERIAL_DEVICE` 后映射，宿主机路径须解析为 `/dev/` 下字符设备，容器内统一为 `/dev/intramural-base`。设备映射不替代协议、断链停车和物理标定三项验证。
- 当前默认使用 NVIDIA RTX 5070 Ti 的 D3D12 GPU 加速，Gazebo 使用 `ogre`。`/usr/lib/wsl` 完整只读挂载提供接口库和显卡用户态驱动，`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 选择独显。详见 [GPU 配置说明](gpu.md)。
- 需要软件模式时设置 `INTRAMURAL_SOFTWARE_RENDERING=1`，此时使用 llvmpipe / OGRE2。GUI 启动入口在 GPU 模式下检查硬件加速，不会默默接受软件回退。
- 如图形异常，先执行 `scripts/check-environment.sh`，检查 DISPLAY、WSLg 和 `glxinfo -B` 的实际输出；不要根据设备接口存在就认定 GPU 加速已生效。

## 记录与缓存

- `nav_ws/build/`、`install/`、`log/`：构建及 ROS 运行输出。
- `cache/`：固定 small_gicp 子模块、容器 HOME 和下载。
- `artifacts/stage1/`：镜像构建、工作空间构建、环境和图形验证记录。
- `cache/maps/`：按 `config/maps.lock.json` 校验的公开点云图，不提交大型地图。
- `artifacts/stage2/`：传感器导航到点、运动中取消、真实停车、TF/时钟、轨迹与完整退出的证据。
- `scripts/verify-gui.sh`：启动 RViz 和 RM 场景 Gazebo，保存各自窗口截图，采样时钟 / 点云 / IMU，并验证 SIGINT 退出。重复验证可用 `scripts/verify-gui.sh --output /work/artifacts/stage1/gui-repeat` 指定证据目录。

出现原生崩溃时，先保留日志和调试器证据；默认容器禁用 core dump，防止大文件进入工作空间 / 构建上下文。此次排障留下的 ELF core 已归档到 `artifacts/stage1/debug-cores/`，不参与构建。

上述生成物已列入 `intramural/.gitignore`。上游各仓库独立保留许可证和 Git 历史；本目录的忽略规则不能代替各个嵌套仓库的提交前检查。
