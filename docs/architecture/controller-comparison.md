# PB、前馈跟踪与 MPPI 的离线对照

`scripts/compare-controllers.sh` 在独立进程内调用锁定的 PB 插件和系统 Nav2 Humble MPPI 1.1.20，与当前 `OmniTracker` 闭环对照。代码位于 `scripts/internal/controller-benchmark/`，JSON 行协议只传模拟状态和速度结果，不启动串口、雷达或实车导航，不改变生产启动入口。

| 后端 | 实现 |
|---|---|
| `tracker` | 当前整圈切向前馈与位置/速度反馈 |
| `pb` | 实际 PB Omni PID 插件；实车基线参数 Kp=4、Ki=0、Kd=0.3、固定前瞻 0.18 m |
| `mppi` | 实际 Nav2 MPPI；全向、固定 wz=0，保留原平移预测 |
| `mppi-response` | 同一 MPPI 优化器，增加延迟队列、一阶响应、合速度与矢量加速度预测 |

这是控制后端对照，不是旧版分段任务链或 18.554 秒现场成绩的重放。各后端共享地图约束的 C2 路线、10 个 gate、有序进度、速度曲线、停车判据及生产 `CommandPolicy` 限幅。进入后续交叉点的跳段不允许。停止发送后仍让延迟队列与一阶响应继续演化，再核对停车，避免把“发出零速”当成物理瞬停。

Nav2 两后端使用相同的 0.29 m 圆形车体膨胀代价图；独立判分还检查原始地图净空和实体桩半径。未知区保持未知。默认姿态固定，wz 预测和输出均为零；PB 不请求车头对齐。比较使用 40 Hz，MPPI 每次 512 个样本、80 步、一次迭代，没有做大范围调参。

Humble 的原模型没有本车响应参数，不能直接沿用最新 Nav2 文档的加速度配置。源码见 [Humble motion_models](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp) 和 [optimizer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp)。该优化器按角速度采样方差做除法，所以这里保持非零采样方差，并在模型预测前把 wz 清零，避免把 `wz_std=0` 配成 NaN。PB 与 Nav2 都有 `liblayers.so`，仅在 MPPI 子进程中优先解析 Nav2 库，避免依赖同名冲突。

## 运行

先完成项目常规构建，确保 PB 插件可加载。独立工具需要已安装的 `nav2_mppi_controller`、CMake 和 `nlohmann_json` 头文件；当前项目开发镜像已具备，不升级系统 Nav2。

在项目根目录提供本次地图导出的 `arena.json`、`map.yaml` 与图像：

```bash
scripts/compare-controllers.sh \
  --map-dir artifacts/your-mapping/navigation \
  --output artifacts/controller-comparison/run-01 \
  --repeats 2
```

默认使用独立 ROS domain 96；只创建离线代价地图及调试话题，无 `cmd_vel` 发布。默认比较 safe/candidate 和正常/大延迟两个假设场景。可用 `--profiles safe`、`--scenarios nominal`、`--backends tracker pb mppi mppi-response` 缩小范围，输出目录必须是新目录。

没有现场地图时，可以完全从仓库公开资源复现，不启动 Gazebo：

```bash
scripts/run.sh python3 /work/scripts/internal/generate-arena.py
scripts/compare-controllers.sh \
  --map-dir simulation/generated --arena-config config/arena.json \
  --profiles safe --scenarios nominal --backends tracker mppi \
  --output artifacts/controller-comparison/public-arena
```

默认正常模型为延迟 0.2 s、τ=0.15 s，压力模型为 0.4 / 0.3 s。它们都是人工假设。使用已经辨识的旧实车记录或后续采样覆盖正常模型：

```bash
scripts/compare-controllers.sh \
  --map-dir artifacts/your-mapping/navigation \
  --response-model artifacts/response/history-source/model-2.json \
  --scenarios nominal \
  --output artifacts/controller-comparison/identified
```

仅接受 `eligible_for_model=true` 的源时间模型，防止把含定位等待的接收延迟再次叠加。它同时更新正常模拟响应和响应版 MPPI；当前 `tracker` 仍使用其原有线性外推方法。模拟底盘增益仍为单位阵，不会把含地图朝向的 `gain_world_from_input` 误作车体标定。数据来源和其他未拟合项见[响应测量](../vehicle/response-measurement.md)。

## 输出与解释

`results.json` 保存每次成功或失败、用时、最大跟踪误差、最小桩/地图净空、绕数、回位误差、求解 p50/p95、单步总开销和超周期计数；失败包含接触、地图净空、跟踪超限、控制器错误与超时，不丢弃失败样本。`*.jsonl` 保存逐步实际模拟轨迹，`*.log` 保存插件日志，`route-*.json` 固定参考路线，`trajectories.png` 显示实际轨迹和失败终点。`manifest.json` 保存版本、源码/地图哈希及模型参数。

低阶模型、固定车头和无噪声输入用于隔离控制器差异；该工具不运行 `motion_guard`、原始点云、人挡停或实车。碰撞意味着模拟轨迹与几何接触，不代表车辆实际发生碰撞；清空场景成功也不能替代实车挡停恢复。CPU 时间来自当前 WSL 主机，不是 NUC 性能。名为 safe 的档位同样可能在压力模型下失败，不能根据档名宣称安全。

比较结果支持选择下次实车候选，不自动把 MPPI 接入运行链。现有实现仍保持单一最终速度发布者。

## 2026-10-19 对照结果

在同一现场地图与生成路线下，使用旧日志切入段辨识的延迟约 0.092 s、τ 约 0.089 s，四种后端两档均完成。以下用时属于这个离线模型，不是新的实车成绩：

| 后端 | safe | candidate |
|---|---:|---:|
| 当前前馈跟踪器 | 17.40 s | 13.63 s |
| PB | 18.13 s | 16.85 s |
| 原平移模型 MPPI | 19.75 s | 18.50 s |
| 响应模型 MPPI | 23.80 s | 19.65 s |

另用人工的正常 0.2 / 0.15 s、大延迟 0.4 / 0.3 s 模型完成 32 次对照，每个组合两次。正常场景全部完成；大延迟下，响应版 MPPI 的 safe 两次完成，其他后端的 safe 触发地图净空或模拟桩接触。candidate 在该压力场景全部失败：响应版 MPPI 跟踪误差较小，但仍违反地图净空；不能只凭跟踪误差小就判定可用。

当前前馈方案在已辨识模型下效率领先；MPPI 的价值目前体现在响应模型失配时的对照与鲁棒性研究，没有证据支持直接替换主后端。未做 MPPI 大规模调参。精选数据见[机器可读结果](../results/2026-10-19/controller-study.json)，[实际模拟轨迹](../results/2026-10-19/controller-trajectories.png)中叉号为失败终点。
