# 脚本入口

本目录第一层保留用户入口，内部 Python 实现放在 `internal/`，验收、审计、画图及 C++ 观测工具放在 `validation/`。不要直接把 `internal/` 的脚本当作独立部署入口；启动、环境变量和退出顺序由外层命令统一管理。

| 用途 | 入口 |
|---|---|
| 统一仿真、导航、串口模拟、探索建图和诊断 | `candidate.sh <模式>` |
| 镜像、工作区构建和环境检查 | `build-image.sh`、`build-workspace.sh`、`check-environment.sh` |
| 进入隔离运行环境 | `run.sh [命令]` |
| 当前校内赛任务及障碍调试 | `arena.sh`、`arena-obstacle.sh` |
| 上游对照仿真、导航、RViz | `simulator.sh`、`navigation.sh`、`rviz.sh` |
| 阶段验收 | `verify-gui.sh`、`verify-stage2.sh`、`verify-stage34.sh`、`verify-stage6.sh`、`verify-stage6-fault.sh` |
| 独立目录构建、测试及完整串口模拟路线 | `reproduce-candidate.sh --output /work/artifacts/<新目录>` |
| 固定公开源码恢复、检查与补丁导出 | `python3 prepare-sources.py`、`python3 sources.py status`、`python3 sources.py export <仓库>` |
| 按参考接口生成独立实车配置，不连接设备 | `run.sh python3 /work/scripts/prepare-vehicle.py --output /work/cache/<新目录>` |
| 实车雷达与 Point-LIO 调试，不启动底盘或任务 | `vehicle-sensors.sh`；配置目录由 `INTRAMURAL_VEHICLE_CONFIG` 指定 |
| 冻结文件与证据哈希 | `run.sh python3 /work/scripts/freeze-candidate.py --output <新目录> --evidence <文件>` |

日常使用以 [README](../README.md) 与 [候选说明](../docs/prevehicle-candidate.md) 为准，公开源码修复流程见 [补丁说明](../patches/README.md)。阶段 1—7 的验证工具是回归资源，已完成阶段不构成删除理由。
