# WSL 容器 GPU 渲染

当前显卡：NVIDIA GeForce RTX 5070 Ti Laptop GPU。Humble 容器使用 Mesa 23.2.1 的 D3D12 后端，通过 WSLg 显示窗口。此处验收的是 OpenGL 图形 / 仿真传感器加速，未验证 CUDA 算法。

## 原因与修复

原启动脚本挂载了 `/dev/dxg` 与 `/usr/lib/wsl/lib`，遗漏 `/usr/lib/wsl/drivers`。D3D12 可找到接口库，却无法加载 Windows 显卡用户态驱动，随后回退 llvmpipe。补齐 `/usr/lib/wsl` 的只读挂载后，NVIDIA 与 Intel 的 `glxinfo -B` 都报告 `Accelerated: yes`。挂载要求与 [微软 WSLg 容器说明](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md) 一致。

使用 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 明确选择独显，避免多显卡机器选择其他设备；选择方式见 [WSLg GPU selection](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg)。不安装 Linux NVIDIA 内核驱动，不修改 Windows 驱动或全局 WSL 配置。

该容器的 D3D12 后端实际提供 OpenGL 3.3。Fortress 的 OGRE2 启动后在 `GL3PlusTextureGpu::copyTo` 抛出 `UnimplementedException`；原始记录为 `artifacts/stage1/gazebo-gpu-ogre2-failure.log`，上游有 [同类问题 #662](https://github.com/gazebosim/gz-rendering/issues/662)。GPU 模式明确使用内置 `ogre` 渲染器；GUI 与传感器服务端均使用它，没有关闭激光雷达 / 相机，也没有伪造 OpenGL 版本。

退出时采用自有 `gazebo-process.py` 先停止服务端、再停止 GUI、最后停止 ROS 节点，保留每个子进程的返回码。此前同时结束出现过服务端超时和时钟桥接节点退出崩溃；先让传感器 / 时钟发布源完全退出，再结束 ROS 节点，最终配置通过重复运行验证。GDB 诊断只用于定位，不替代最终未使用调试器的验收。没有把调试器退出码当成 Gazebo 验收结果。

另修复公开底盘节点的传感器回调生命周期：销毁控制器前关闭共享回调门闩，等待正在运行的回调，排队回调只能读取存活的门闩，不能访问已释放对象。修复保留在固定公开补丁中。

## 使用

```bash
scripts/check-environment.sh
scripts/rviz.sh
scripts/simulator.sh
scripts/verify-gui.sh --output /work/artifacts/stage1/gui-gpu-repeat
```

GPU 模式默认选择 NVIDIA；GUI 启动入口先检查硬件加速，检查失败时明确报错。

WSLg 正在运行、但 SSH 或 agent 会话没有继承 `DISPLAY` 时，`run.sh` 根据实际存在的 WSLg X0 socket 补为 `:0`；已显式设置的显示地址保持原值。图形入口先打印正在检查的显示地址，`glxinfo` 在 20 秒超时后中止检查，失败时停止启动并明确报错。

WSL 硬件模式的 Gazebo GUI 单独设置 `QT_OPENGL_NO_SANITY_CHECK=1` 和 `QSG_RENDER_LOOP=threaded`，规避已记录的 Qt / GLX / Mesa 上下文启动崩溃；早期使用过的 `basic` 配置不再采用。该设置不传给 Gazebo 服务端或 ROS 节点，也不代表系统驱动已修复。

需要软件模式时单次覆盖：

```bash
INTRAMURAL_SOFTWARE_RENDERING=1 scripts/simulator.sh
```

软件模式使用 llvmpipe / OGRE2。不要在运行中的同一套仿真内混用不同模式。Intel 目前只通过 OpenGL 探测，尚未完成完整场景验收；`INTRAMURAL_GPU_ADAPTER=Intel` 仅用于独立诊断。

## 证据与边界

- `gpu-nvidia-full-mount.txt` / `gpu-intel-full-mount.txt`：补齐挂载后的 OpenGL 探测。
- `gui-gpu/graphics.txt`：NVIDIA D3D12 renderer 和 `Accelerated: yes`。
- `gui-gpu/*-driver-maps.json`：RViz、Gazebo 服务端与 GUI 的实际进程已加载 NVIDIA 用户态驱动；验证不限于单独运行 glxinfo。
- `gui-gpu/{clock,sensors,results}.json` 及截图：实际场景、时钟、点云、IMU和正常退出；`gui-gpu-repeat/` 保留重复验证。
- `base-lifecycle/results.json`：在仿真持续发布传感器数据时，8 次底盘节点启动、状态采样和 SIGINT 退出。
- `interactive-gpu-shutdown.log`：真实终端 `Ctrl-C` 验证。

已验证的渲染配置是 NVIDIA / D3D12 / OGRE。OGRE2 的 D3D12 兼容性问题仍存在，不能把更换渲染器写成 OGRE2 已修复。短时场景验收不代表长期稳定性、定位精度、导航算法效果或实车性能。
