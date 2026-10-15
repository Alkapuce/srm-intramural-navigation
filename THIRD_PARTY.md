# 来源与许可证

自有 `intramural_base`、`intramural_mission` 已声明 Apache-2.0，根 `LICENSE` 沿用其许可证文本。该声明面向本项目自有实现，不重授赛务规则、第三方源码或资源的许可。

公开基线为 [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav)，固定于 `9c124ebb0c0b6b64714390ea3c92f27490f63f9d`。全部下载来源、提交、递归子模块及补丁 SHA-256 见 [sources.lock.json](docker/sources.lock.json)。

| 项目 | 本项目用途 | 许可证定位 |
|---|---|---|
| pb2025_sentry_nav 及其子模块 | Point-LIO、定位、Nav2 控制与地形处理 | 各仓库 LICENSE / package.xml；主仓库 Apache-2.0，Point-LIO 包声明 BSD，Livox 驱动包声明 MIT |
| pb2025_robot_description | 公开对照机器人与传感器资源 | 仓库许可证，包声明 MIT |
| rmoss_core、rmoss_interfaces | 公共 ROS 工具和接口 | 各自仓库许可证 |
| rmoss_gazebo、rmoss_gz_resources、rmu_gazebo_simulator | Gazebo 插件与公开场景 | 各自许可证；Gazebo/模拟器修复目标仓库为 Apache-2.0 |
| sdformat_tools、xmacro | 公开模型生成工具 | 各自许可证，sdformat_tools 包声明 Apache License 2.0；xmacro 版本在 Dockerfile 固定 |
| small_gicp | 公共点云配准库 | 锁定源码仓库中的许可证 |

上述项目以 Git 子模块登记在本仓库 [.gitmodules](.gitmodules)，根 gitlink 指向公开基线；递归克隆包含完整上游源码、许可证及 pb2025 的 7 个子模块。自有对公开项目的修改在 `patches/`，恢复时应用并核对；其内容与适用的原项目许可证一起核查，不把所有依赖统一标成 Apache-2.0。地图来源与哈希由 `config/maps.lock.json` 标注，原始地图不上传。

相邻 SRM 上下位机仓库仅用于经用户允许的只读接口核对，具体事实来源记录在 `config/vehicle-srm-reference.json`；它们不属于开源依赖，也不参与本项目构建或运行。

`算法组校内赛规则/sentry_robot_cylinder.xacro` 是赛方直接提供的比赛模型，按原始字节保留并单独记录来源，根 Apache-2.0 不重授其许可。模型的碰撞及惯性由自有转换工具接入，缺失的私有网格以锁定公开 MID-360 资源替代；来源、变换及仿真假设见[模型接入记录](docs/debug-log.md)。
