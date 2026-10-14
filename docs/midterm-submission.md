# 算法导航赛道中期提交

提交仓库链接即可：[srm-intramural-navigation](https://github.com/Alkapuce/srm-intramural-navigation)。按赛务中期通知，须在 2026-10-17 之前发送给“算法-郭海平-老登”或“电控/算法-李识予-老登/小登”。该页面不代表已经发送链接。

仓库保持 private，提交前需要让接收方的 GitHub 账号具有读取权限。所有者负责提供访问权限；仅发链接不会自动授权。

| 评审内容 | 入口 |
|---|---|
| 系统结构、完整部署与运行 | [README](../README.md) |
| 自有有序任务、绕行及速度保护 | [intramural_mission](../nav_ws/src/intramural_mission/intramural_mission) |
| 比赛串口、模拟下位机及异常清理 | [intramural_base](../nav_ws/src/intramural_base/intramural_base) |
| 公开导航、定位、控制与仿真依赖 | [.gitmodules](../.gitmodules)、[固定版本及补丁](../docker/sources.lock.json)、[来源与许可证](../THIRD_PARTY.md) |
| 技术接口与仿真假设 | [任务说明](arena-mission.md)、[导航数据流](navigation-dataflow.md)、[实车接口](vehicle-reference.md) |
| 精选调试结果与未完成边界 | [仿真验收](validation.md)、[结果 JSON](results/2026-10-13/validation-summary.json) |
| 可重复验证 | [测试](../tests)、[验收入口](../scripts/README.md) |

2026-10-14 已从干净交付克隆完成 31 包独立构建、60 项测试与传感器/模拟串口完整路线，取消、复位及退出通过，回位误差 0.0351 m；见[克隆复验结果](results/2026-10-14/reproduction.json)。

9 个顶层公开依赖以固定 Git 子模块纳入，pb2025 的 7 个子模块递归获取，包含 Point-LIO、small_gicp 重定位及 Nav2 插件。仓库保存 3 个完整补丁；不依赖开发者未发布的本地提交，也不依赖相邻 SRM 队内源码。

```bash
git clone --recurse-submodules git@github.com:Alkapuce/srm-intramural-navigation.git
cd srm-intramural-navigation
python3 scripts/prepare-sources.py
python3 scripts/sources.py status
scripts/build-image.sh
scripts/build-workspace.sh
scripts/run.sh python3 -m unittest discover -s /work/tests -v
scripts/reproduce-candidate.sh --output /work/artifacts/midterm-reproduction
```

前述命令适用于具备 Docker、桌面图形接口及所需 GPU 驱动的开发机。NUC 的原生桌面构建、仿真和传感器调试流程见 README，不要求 Docker 或无桌面服务。

当前本机范围已完成，实车尚未交付验收。真实固件兼容、断链停车、坐标与速度标定、雷达外参、现场地图和实车闭环仍待确认。调试记录与本机仿真可以用于中期展示，不能作为实车通过的证明。
