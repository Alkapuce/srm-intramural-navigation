# 接车前候选版本操作与部署边界

本目录中的候选版本用于 WSL 本机和隔离容器的重复仿真。现已接入赛方圆柱哨兵模型；场地尺寸、平面驱动、解析先验、雷达扫描及光学自遮挡仍是仿真假设。真实 NUC、雷达、串口及当前 SRM 固件没有参加本轮试验；[模型接入记录](validation.md)与现场验收分别记录。

## 统一入口

在 `intramural/` 先运行 `scripts/check-environment.sh`、`scripts/build-workspace.sh`。下列命令互斥运行，同一时刻只启动一套 ROS domain 71 / Ignition partition `srm_intramural`：

```bash
scripts/candidate.sh simulation                 # 理想定位的场景与任务
scripts/candidate.sh navigation                 # 仿真雷达/IMU，Point-LIO + small_gicp
scripts/candidate.sh serial-sim                 # 同上，速度经伪终端和模拟下位机
scripts/candidate.sh mapping --output /work/artifacts/stage57/my-map
scripts/candidate.sh diagnose --output /work/artifacts/stage57/my-diagnostics
```

需要从独立构建目录验证当前源码、测试和全圈串口模拟时，运行 `scripts/reproduce-candidate.sh --output /work/artifacts/stage57/<新目录>`；每次使用未存在的目录名。当前结果和操作边界见 [阶段 7 验收](validation.md)。

前三个入口启动后待机；在另一终端调用 `scripts/run.sh ros2 service call /intramural_robot/mission/start std_srvs/srv/Trigger '{}'`，状态可用 `scripts/run.sh ros2 topic echo /intramural_robot/mission/status` 查看。取消、复位和退出顺序见 [任务说明](arena-mission.md)。映射模式自动完成一次自主 8 字并将传感器注册点云抽样、变换、10 cm 体素化写为 `sensor-survey.pcd`，附带来源/配置快照。该探索性地图的导航和 map→odom 对齐仍借助场景解析先验；它不能证明无先验自主建图，更不能当成实车现场地图。诊断模式只读核查公开源码版本、包和当时的 ROS 节点/话题，输出 `diagnostics.json`。

## 在共用 NUC 上的准备顺序

下列 NUC 权限及网络值来自 2026-09-30 现场记录，当前连接前按 [README](../README.md) 重新核查；未在本轮远程连接或部署。

1. 使用个人用户 `qpj` 与独立工作空间目录。NUC 现场无外网、无 Docker 且选手无 sudo 权限；代码通过笔记本 WSL2 局域网执行 `rsync` 单向推送到 `~/workspaces/srm/intramural/`，无需在 NUC 上配置 GitHub 凭据。
2. 宿主机已由赛方预装原生 ROS 2 Humble（`/opt/ros/humble`）与 `colcon`，使用原生编译命令 `source /opt/ros/humble/setup.bash && colcon build --symlink-install` 编译核心包。
3. 赛方模型已取得并接入仿真，接车后核对实际安装、雷达驱动、现场地图和串口设备。历史记录中 NUC 用户属于 `dialout`，串口为 `/dev/ttyACM0`，有线网口 `enp86s0` 使用 `192.168.1.50/24`；这些值须用当前设备、权限和真实点云调用复核。
4. **架空安全门禁实测**：在断开动力或安全架空（麦轮悬空离地 >3cm）条件下，验证遥控器急停接管有效性（上位机发微慢速时，安全员将右拨杆拨回 MID，轮子必须瞬间切断动力停转）。逐轴测量正方向与速度比例（$2\times2$ 映射矩阵），更新 `base-vehicle.json` 的三项验证字段。门禁未完成前，串口节点拒绝打开设备。
5. 逐项注入控制停更、定位停更、任务取消和物理断线。参考源码及模拟下位机没有提供断链超时停车，当前实车固件尚未实测；须验证真实断链停车或经赛务认可的等效硬件停机机制。遥控急停先在调试中测试，正式运行遵循禁止人工接管的规则，不能把人工操作当作已通过的自主停车能力。
6. 使用实测安装和现场地图，核对 TF 唯一来源、雷达外参、地图初始化、制动表现、绕桩净空和人为干扰下的实际运行；重新执行完整验收。不得把本机仿真的回位误差移作实车结果。

通信层直接运行真实设备的入口是 `intramural_base` 的 `serial_bridge` 节点，需显式传入 `/dev/serial/by-id/...` 设备和已完成的车辆标定副本。当前统一候选脚本故意只包含仿真、串口模拟与诊断模式，待设备和标定证据齐备后再增加实车启动文件，避免误用仿真真值或单位映射。

可复核的版本、场景、日志和未验证项分别保存在 `docker/sources.lock.json`、`config/arena.json`、`artifacts/stage57/` 和阶段 5—7 验收文档。输出目录使用新的名称保留历史失败与成功记录，`artifacts/`、`simulation/generated/`、构建缓存和临时伪终端路径均不进入源码管理。

相邻 SRM 仓库的已知接口值及当前固件冲突见[实车参考参数](vehicle-reference.md)。可用 `scripts/prepare-vehicle.py` 生成独立交接配置，不直接复制队内实现；地址、外参和圆形半径仍需按实际车辆确认。
