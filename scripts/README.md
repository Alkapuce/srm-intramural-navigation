# 脚本入口

本目录第一层保留用户入口，内部 Python 实现放在 `internal/`，验收、审计、画图及 C++ 观测工具放在 `validation/`。不要直接把 `internal/` 的脚本当作独立部署入口；启动、环境变量和退出顺序由外层命令统一管理。

| 用途 | 入口 |
|---|---|
| 个人用户全局命令安装与日常实车调试 | `install-cli.sh`；`srm-nav`、`srm-map`、`srm-pulse`，见[CLI 手册](../docs/field-cli.md) |
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
| 单命令现场采集、校验、导图与重启定位验证 | `srm-map`；NUC 可安装到个人 `~/.local/bin` |
| 手动分步采集、真实 bag 局部地图和导航待机，任务及运动固定锁定 | `site-mapping.sh`、`prepare-vehicle-navigation.py`、`vehicle-navigation.sh`；流程见[实车待机](../docs/vehicle/navigation-standby.md) |
| 冻结文件与证据哈希 | `run.sh python3 /work/scripts/freeze-candidate.py --output <新目录> --evidence <文件>` |

日常使用以 [README](../README.md) 与 [候选说明](../docs/vehicle/commissioning.md) 为准，公开源码修复流程见 [补丁说明](../patches/README.md)。阶段 1—7 的验证工具是回归资源，已完成阶段不构成删除理由。
