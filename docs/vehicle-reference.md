# SRM 实车参考参数与接车边界

已只读核对相邻 `control-2026`（`3b889edcbf4676f236e6e2cd3030fde9853ed9be`）和 `srm_auto_sentry`（`fc5362f1ba476bdd464b3362f389127f34923f19`）。[参考参数](../config/vehicle-srm-reference.json)是独立编写的接口事实记录，保留具体来源文件；未复制队内实现、整份配置、地图或车体模型。仓库内容与当前车辆烧录固件可能不同，以下参数不构成实测标定。

2026-09-30 用户在实验室确认：拟使用的导航车是 SRM 上赛季哨兵，赛方还未配置完成并交付调试。因此以上两份参考提交的传感器、网络、安装参数继续作为接车初值；同一车辆的来源已由用户说明，配置完全一致仍是待接车核对的判断。比赛规则及导航协议优先，既有 20 字节发送帧或解析冲突不沿用。

已查看本地照片 `docs/09_30real_debug/IMG_20260930_170648.jpg`：可见麦克纳姆轮底盘、上方云台和底盘边侧的圆顶雷达，雷达看起来安装在底盘而非上方云台。单张透视照片不能测出精确平移、旋转、整车 footprint 或固件行为；不能据此确认 `[0.15, -0.15, 0.22]` m、驱动 yaw −90° 已实测。车体轮廓还应包含雷达支架、防撞结构及实际突出的部件。现场照片与原始草稿保留本机，不随 GitHub clone 获取。

| 参数 | 本项目采用或记录的值 | 核对位置与限制 |
|---|---|---|
| 下位机串口 | 115200、8N1、无流控 | `control-2026/Hardware/stm32-f4/Src/usart.c:49`；哨兵 `robot.c:323` 使用 USART1 |
| 串口设备 | `/dev/ttyACM0` 仅作历史提示 | 上位机 `standard_robot_pp_ros2.yaml:3`；启动仍要求显式设备，优先 `/dev/serial/by-id/` |
| MID-360 地址 | 主机 `192.168.1.50`，雷达 `192.168.1.184` | 上位机 `rm_nav_bringup/config/reality/MID360_config.json`；接车核对网卡和实际设备后可覆盖 |
| 雷达/主机 UDP 端口 | 56100—56500 / 56101—56501，步长 100 | 同一 MID-360 文件；生成工具不修改网卡、路由或雷达 |
| 发布频率、坐标系 | 10 Hz、`livox_frame` | `rm_nav_bringup/launch/bringup_real.launch.py:68` |
| 点云消息格式 | 本项目选择 `xfer_format=1`、LivoxCustomMsg | 锁定公开驱动 `livox_ros_driver2/src/lddc.h:45` 也定义格式 4（AllMsg），`lddc.cpp:132` 实现双格式发布；选择 1 用于固定本入口的输入类型 |
| Point-LIO 输入 | Livox 类型 1、4 线、时间单位 3、盲区 0.5 m、IMU 200 Hz、加速度单位 g | 队内 `pointlio_mid360_real.yaml`，同类值也存在于公开 `point_lio/config/mid360.yaml`；真实点内时间由驱动提供，禁用仿真 `lidar_adapter` |
| 安装位置参考 | base→lidar `[0.15, -0.15, 0.22]` m、RPY 0 | 实际 real launch 读取的 `measurement_params_real.yaml`；另一个未选用的 URDF 默认值不同，因此不采用它 |
| 驱动旋转参考 | roll 0°、pitch 2°、yaw −90° | MID-360 配置；与 TF、IMU 旋转及 Point-LIO 外参一起现场核对，避免重复旋转 |
| 雷达→IMU 参考 | T `[-0.011, -0.02329, 0.04412]` m，R 单位阵 | 真实 Point-LIO 配置与公开默认；保留为参考，未确认适配当前安装 |
| 车体半径参考 | 0.27 m | 队内 `nav2_params_real.yaml` 的圆形近似；不代表已测 footprint，也不改写已验收仿真尺寸 |

## 赛方提供的模型（2026-10-12）

用户确认 [sentry_robot_cylinder.xacro](../算法组校内赛规则/sentry_robot_cylinder.xacro) 由赛方提供。这份文件单独记为赛方模型资料，可作为本赛建模依据；同名文件存在于队内参考仓库，不改变本次提供来源，也不意味着允许复制队内其他模型资源或实现。

只读比较参考提交 `fc5362f1ba476bdd464b3362f389127f34923f19` 的 `src/pb_rmsimulation/src/rm_nav_bringup/urdf/sentry_robot_cylinder.xacro`：赛方文件使用 CRLF，参考文件使用 LF，统一换行后内容完全一致。赛方原文件 SHA-256 为 `ba3fe8889a4108ee6e63f26a0301f9ad1d75d90f94bae74762eb712354de5dce`，原文件未改写。

| 模型项 | 文件中的定义 |
|---|---|
| 车体 | 圆柱半径 0.27 m、高 0.20 m，中心位于 base_link 上方 0.15 m；车体质量 8.2 kg，不是整车实测质量 |
| 四轮 | 半径 0.075 m、宽 0.04 m；中心 x/y 为 ±0.18 m、z 为 0.075 m，模型轴距/轮距均为 0.36 m |
| TF 结构 | base_link、wheel_1～wheel_4、livox_frame 共 6 个 link、5 个 fixed joint；没有 base_footprint |
| 雷达安装参数 | livox_joint 使用外部 xyz/rpy 参数，默认均为零；模型本身没有写入已记录的实车外参 |
| 雷达外观依赖 | `package://pb_rm_simulation/meshes/mid360.stl`，当前公开工作空间无法解析该资源 |

已用当前 Humble 环境实际展开默认参数和 `xyz:="0.15 -0.15 0.22" rpy:="0 0 0"` 两种 URDF，参数断言和 `check_urdf` 树解析通过。本地证据在 `artifacts/maintenance/2026-10-12/organizer-model/`。旧实车 launch 正是从 `measurement_params_real.yaml` 注入这组平移及零 RPY；MID-360 驱动内的 pitch 2°、yaw −90° 属于另一层坐标处理，不能再无依据叠加进该 joint。雷达 visual 的局部 origin 只对齐网格外观，也不是额外安装 TF。

这说明赛方模型与上赛季参考模型一致，不能据此确认固件、网络、驱动配置或实车安装完全相同。文件没有 Gazebo 雷达传感器、驱动插件或麦轮动力学，四轮为固定关节。本轮已通过 `scripts/internal/organizer_model.py` 接入仿真：原生转换保留碰撞和惯性，补充公开 MID-360 传感器/外观、平面速度驱动及 `base_footprint` / `front_mid360` 别名。车轮与圆柱的联合包络半径 0.28462 m，导航保守圆 0.29 m；安装初值 XYZ=(0.15,−0.15,0.22) m、RPY=0。原始 Xacro 字节保持不变，缺失的队内 STL 没有复制，派生结果及来源写入 `simulation/generated/organizer-model.json`。模型接入不改变任何实车标定或停车验收开关。参考安装点位于粗略圆柱内，派生 SDF 的 GPU 雷达用可见性掩码排除本车粗略外观自遮挡，GUI 外观和全部碰撞保留；该光学假设及扫描对照见 [仿真说明](arena-mission.md)。

## 生成接车配置

在 `intramural/` 运行，输出目录必须尚未存在：

```bash
scripts/run.sh python3 /work/scripts/prepare-vehicle.py \
  --output /work/cache/vehicle-srm-reference
# 当前机器/雷达地址不同，可额外指定 --host-ip <IPv4> --lidar-ip <IPv4>
```

工具生成 MID-360 网络 JSON、公开驱动参数、Point-LIO 参数、串口节点参数、车辆标定模板及 `handoff.json`。Point-LIO 完整配置以固定公开项目的默认文件为基础，只覆盖已核对的接口值；交接清单记录参考提交、配置哈希、选用 IP 和静态 TF 参数。工具不启动传感器、打开串口或改变网络。生成的绝对路径按本项目 `/work` 容器挂载使用；这些文件供接车核对，不自动接入仿真或构成完整实车 launch。

实车串口节点在打开设备前分别要求 `firmware_protocol_verified`、`link_loss_stop_verified`、`calibrated_for_vehicle` 为 true，并检查有效的速度映射与限幅。模板全为 false，映射为空。只有取得对应的固件协议测试、断链停机测试和逐轴物理标定记录后才能填写；不能为绕过启动拒绝而只修改开关。仿真模式仍保留独立、无超时停车的模拟下位机。

## 当前参考源码的协议冲突

比赛规范仍是 **19 字节指令、12 字节数据、递增序号、无 cmd_id**，本项目发送器与测试保持这一约定。参考仓库不能直接视为兼容固件：

- 队内上位机 `packet_typedef.hpp:186` 的发送结构额外含 `is_recovering`，总长 20 字节、数据长 13；与比赛指令不一致。
- 下位机 `navigator.c:351` 将帧长度算为 `5 + 4 + len + 2`，len=12 时需 23 字节缓存，但 CRC16 又按 19 字节位置读取。流式连续输入也会受其按 23 字节推进索引的影响。
- `navigator.c:380` 将帧头第 4 字节作为命令 ID 判断，仅接受 `0x01`；比赛定义该字节为序号，正常递增不能保证每帧被接收。
- `navigator.c:383` 从数据起点复制整个 18 字节 `navigator_recv_t`，超过 12 字节有效数据。以上问题未在参赛项目中通过增大发送帧、固定序号或复制队内解析器绕过。
- `control-2026/CMakeLists.txt:23` 的 MCU 选择是 F4，列出的 `ROBOT_TYPE` 均注释；无法由该默认文件确认实车哨兵构建参数或烧录版本。

这些是当前提交的静态源码事实，尚未运行真实 MCU 解析或确认赛务固件。接车时应取得实际固件来源/构建参数，以协议样例、0—255 序号及碎片收帧测试确认兼容性。

## 速度与姿态仍需实测

队内上位机 `standard_robot_pp_ros2.cpp:626` 直接把 ROS linear.x/y 写入协议 vx/vy。下位机哨兵 AUTO 路径 `robot.c:135` 再执行 `vx_initial=-vy*5000`、`vy_initial=vx*5000`、`wz=0`；随后麦轮底盘 `chassis.c:212` 依据云台偏角继续变换。因此不能在上位机再盲目交换轴，也不能把 5000 直接解释为实测 m/s 倍率。当前 `body_to_protocol` 继续留空。

`chassis.c:199` 的 CHASSIS_FOLLOW 会增加云台跟随 PID 角速度，所以协议 wz=0 不能证明车体朝向固定。自主模式、云台参考系、底盘模式、物理速度增益、footprint、外参及断链停机仍按[接车顺序](prevehicle-candidate.md)验证。本机配置和仿真通过不替代这些证据。
