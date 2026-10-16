# 现场命令手册

日常只需要三个入口：`srm-map` 建图，`srm-nav` 启动与查看导航，`srm-pulse` 单独诊断底盘。命令在**当前机器**执行，不自动 SSH。连接 NUC 后，在它的终端执行下面的现场命令。

第一次部署见 [NUC 部署](deployment/nuc.md)。已经部署好的车辆，可直接从“现场建图”开始。

最常用的流程无需填写路径：先 `srm-map`；完成后在两个终端分别运行 `srm-nav sensors`、`srm-nav drive`，第三终端查看状态，再 `srm-nav start`。停在其他已定位位置时改用 `srm-nav start-here`。

## 运行环境

当前 NUC 已注册 `srm-nav`、`srm-map`、`srm-pulse`，PATH 和默认车辆配置 `cache/vehicle-current` 均已配置，自动使用原生 Humble，默认 ROS domain 为 71。新开终端即可执行，无需重复安装或设置环境变量。

新机器的安装与构建见 [NUC 部署](deployment/nuc.md)，车辆参数准备见[实车接口](vehicle/reference.md)。这些命令在当前用户的 `~/.local/bin/`，只操作当前机器；同一物理雷达和串口仍需独占使用。

## 现场建图

### 1. 上电并检查设备

```bash
srm-nav status
```

检查雷达供电、有线地址、串口和相关进程。重启后尤其留意旧哨兵程序是否重新自启；由程序所属用户或现场负责人停止整套旧控制程序，再启动本项目。`status` 只列出状态，不结束进程；普通用户也未必能看到其他用户全部串口占用。

### 2. 采集新场地

将车放在计划停车位，朝向确定后启动：

```bash
srm-map
```

先保持静止，等终端出现“录制中”，再由操作者缓慢移动整车，覆盖两桩、两侧绕行区域及出入路线。可在电机禁用后手推；若使用遥控驾驶，需由现场人员控制，本命令本身不发送底盘速度。

回到起始位置和朝向、停车，然后在建图终端按 **Enter**。程序依次收尾录包、校验数据、导出三维和二维地图、尝试提取两桩，最后重启传感器检查地图配准并释放本次进程。显示 `bag-quality`、`export-map`、`reload-check` 等阶段名时仍在处理，请等待结束。

**Ctrl-C 取消整个流程**，保留已有数据；它与 Enter 正常收尾不同。

### 3. 取用输出目录

每次输出到新的 `artifacts/mapping/<时间戳>/`：

| 产物 | 用途 |
|---|---|
| `bag/`、`bag-quality.json` | 原始采集与时间戳、消息完整性检查 |
| `map/` | `prior.pcd` 定位地图、二维栅格与待机配置 |
| `map-quality.json` | 已知空闲区和车体净空统计 |
| `navigation/field-detection.json` | 两桩候选、路线检查和失败原因 |
| `navigation/manifest.json` | 识桩及完整路线检查成功后才生成的导航配置清单 |
| `reload-check/`、`result.json` | 重载定位结果、阶段记录和进程收尾结果 |

自动识桩依据细高物体的几何形状与高度连续性；只有恰好两个无歧义候选，且完整八字位于已观测的可通行区域，才生成可供 `drive` 使用的配置。仍应核对识别结果与现场两桩、停车点一致。它没有识别任意场景中“比赛桩”语义的能力。

`status=complete` 表示建图与重载流程结束；若 `field_geometry_detected=false`，地图仍保留，但没有可直接发车的两桩配置。阅读 `navigation/field-detection.json` 与 `field-detection.log`，补扫缺失区域或排除候选歧义，不要把未知区填白。

常用变体：

```bash
# 就绪后录制 120 秒，自动收尾
srm-map --duration 120

# 复用最新已收尾的录制；仍会连接实车传感器检查重载定位
srm-map --bag

# 可选：指定已有个人配置，项目相对路径也可用
srm-map --vehicle-config cache/vehicle-01

# 对最新原图单独识桩与检查路线，不连接设备、不改原图
srm-nav field
```

省略路径时自动选最新有效产物，执行前会打印所选目录。导航按清单文件修改时间选取，仅考虑正常产物目录中的已通过场地检查配置，不扫描归档和测试树；原图与 bag 则分别按地图清单、录包元数据修改时间选取。仍需确认所选图属于当前场地；要固定旧图时显式传入目录。

录制过程中通过 ROS 话题监控，**不要查询正在写入的 SQLite bag**；`bag-info`、`bag-check` 留到录制正常结束后使用。

## 启动导航：三个终端

### 终端 1：传感器，保持运行

```bash
srm-nav sensors
```

启动 MID-360 与 Point-LIO，不打开底盘串口。需要其他车辆配置时可用 `srm-nav sensors cache/vehicle-01`；对应导航终端也须选择同一份配置。日常统一使用 `cache/vehicle-current` 即可。

### 终端 2：导航与底盘通信，保持运行

默认选择最新通过两桩和完整路线检查的导航目录：

```bash
srm-nav drive
```

`drive` 打开串口桥、激活导航节点，**不自动发车**。需要固定已有地图时传入路径；下面 `cache/my-navigation` 是示例，可替换为实际目录。路径支持当前目录相对路径、项目内相对路径和绝对路径：

```bash
srm-nav drive cache/my-navigation
```

串口不能唯一识别时，可用 `INTRAMURAL_SERIAL_DEVICE` 指定，或作为 `drive` 的第二个参数传入。个人车辆配置不使用默认目录时，在仓库根目录设置 `export INTRAMURAL_VEHICLE_CONFIG="$PWD/cache/vehicle-01"` 并在传感器与导航终端保持一致。`INTRAMURAL_NAVIGATION_CONFIG` 可固定导航配置，优先于自动选择；显式路径参数优先于环境变量。任务、控制器和个人车辆限速须一致；入口拒绝超出个人车辆限速或 0.5 软件上限的任务配置。

该终端持续输出日志是正常运行。不要在这里重复启动第二套导航，第三终端用于控制任务。

### 终端 3：查看状态，再发车

```bash
srm-nav mission-status
```

`state=IDLE`、`ready=true`、`tf_problem` 和 `reason` 为空，表示软件已就绪。还应确认地图中的位姿与实车一致、现场允许运动。

从配置停车位发车：

```bash
srm-nav start
```

已在地图中正确定位，但车停在其他位置，调试时使用：

```bash
srm-nav start-here
```

`start-here` 从当前位置规划接入路线，**重新跑完整八字，最后返回原配置停车位**。它不继承上轮进度，不替代重定位；未知区、障碍或车体净空不足时可能找不到接入路径。

路线依次为 `ENTER → LOWER → UPPER → LOWER_EXIT → RETURN → DONE`。任务内部有 10 个有序分段，`segment` 是分段编号，`progress` 是段内路径索引，不是全程百分比。绕过交叉点时也必须保留当前阶段。

## 停车、再运行与换电

| 需要做什么 | 命令 / 操作 | 行为 |
|---|---|---|
| 取消当前任务 | `srm-nav cancel` | 关闭任务运动使能、取消控制目标并输出零速；保留取消状态 |
| 清理结束或故障状态 | `srm-nav reset` | 等待取消结束后复位为 `IDLE`；不移动车辆、不修复故障 |
| 在当前位置重新跑一轮 | `srm-nav start-here` | 前提是问题已处理、定位正确且任务已复位 |
| 查看一次状态 | `srm-nav mission-status` | 显示状态、位姿、阶段、阻断次数与原因 |
| 结束导航 | `srm-nav standby-down` | 正常关闭导航生命周期；随后在终端 2 按 Ctrl-C |
| 结束传感器 | 终端 1 按 Ctrl-C | 在导航退出后停止传感器，等待正常收尾 |

`cancel` 是软件停车，不能替代现场切断动力。关机或换电前，按“取消任务 → 确认车停 → 关闭导航 → 停传感器”的顺序操作。

NUC 重启后，重新检查旧自启程序和设备占用，再启动传感器、导航。若车已离开建图起点，新 Point-LIO 的零点不再等于原地图停车位，需要重新给出地图中的初始位姿并检查配准。可在 RViz 将 **2D Pose Estimate** 的 Topic 设为 `/intramural_robot/initialpose` 后指定当前位置和朝向；或在导航启动前设置该地图 `nav.yaml` 的 `small_gicp_relocalization.ros__parameters.init_pose`，顺序为 `[x,y,z,roll,pitch,yaw]`，角度单位为弧度。

仅在确认车体位置、朝向未动时，才可沿用断电前位姿作为初值。初值只是配准起点，仍须检查点云与地图、车体实际位置一致。`ready=true` 本身不证明全局位置正确。

## 障碍停车与窄道排查

实车配置默认 `allow_bypass=false`：路线受阻进入 `BLOCKED`，先输出零速并取消当前目标；障碍清除并稳定约 0.5 秒、控制目标取消完成后，自动恢复同一阶段。正常等待**不需要重复 start 或 reset**。持续障碍绕行仅在显式启用该策略的配置中执行，仿真说明见[任务与绕行](simulation/arena-mission.md)。

查看任务和最终保护状态：

```bash
srm-nav mission-status
srm-nav ros topic echo --once --full-length /intramural_robot/motion_guard/status
```

| 看到的情况 | 优先检查 |
|---|---|
| `IDLE` | 尚未调用发车服务；`drive` 不会自动 start |
| `start_permitted=false` | 启动的是 `standby`，或配置未放行 |
| `BLOCKED`、`raw_lidar_hazard=true` | 原始点云进入车体或近期路线扫掠范围，查看实物与点云 |
| `BLOCKED`、原始雷达未报危险 | 看任务 `reason`、当前路径和代价地图，可能是地图中的障碍 |
| `FAULT` | 根据 `reason` 处理定位、跟踪、桩旁净空或控制失败，再 reset |
| `tf_problem` 非空或 guard 输入 `stale` | 查看 TF 和对应数据是否停更；任务结束后控制输入过期、输出零速是正常情况 |
| 有有效串口回包却不动 | 回包不是速度采用 ACK；核对导航档位、协议及独占，再单独诊断底盘 |

未来路线保护通过 `mission/collision_path` 获取**当前有序进度之后**的近期路线，随速度计算延迟和制动距离，沿弧线扫掠半径 0.29 m 的车体。`collision_check=ordered_path` 表示该模式生效；路线或新鲜 TF 不可用时使用速度方向保护，状态为 `velocity`。不在未来扫掠内的侧后方点不阻断，实际会经过的侧方区域仍需检查，不能忽略未来将覆盖的侧方区域。

窄道中反复停车时，先看触发点到**未来路径和完整车体包络**的距离，不能仅凭“物体在车侧面”判断误报。安全取消、停稳后，临时移开可移动杂物做对照；不要按“桩”或“人”的类别屏蔽点云。

实车生成配置的主要参数：

| 参数 | 值与含义 |
|---|---|
| 巡航指令上限 | 0.4；任务、控制器、平滑器和个人底盘配置需匹配 |
| 前瞻 / 终点减速范围 | 固定 0.18 m / 0.2 m，适应小半径绕桩 |
| 检查点 / 最终停车容差 | 0.15 m / 0.12 m；同时检查剩余有序路径长度 |
| 车体半径 / 额外静态净空 | 0.29 m / 0.02 m；车体包络不缩小 |
| 同帧触发点数 | 原始扫掠范围内至少 3 点；单个零星返回不阻断，稀疏小物体需现场复核 |
| 极近距回波过滤 | 仅过滤雷达原点 0.15 m 内的零强度点；其他返回保留 |

速度使用现场批准的 1:1 调试初值，指令上限尚不能当作独立测得的实际车速。修改配置或代码后，正常停止并重启相应导航节点才会生效。

## 单独点动底盘

点动直接写串口，不依赖地图、雷达或 Nav2。先结束正在控制同一串口的导航/旧通信程序；**四轮可靠架空，现场有人可立即断动力**，再运行：

```bash
# 默认比赛 19 字节，x 轴 0.02，持续 0.3 秒
srm-pulse x --wheels-lifted --stop-available

# 经现场允许后逐步增加；当前工具最大幅值为 0.10、最长 0.5 秒
srm-pulse x --value 0.10 --seconds 0.3 --wheels-lifted --stop-available
srm-pulse y --value 0.10 --seconds 0.3 --wheels-lifted --stop-available

# 负值测试反向
srm-pulse x --value -0.05 --seconds 0.3 --wheels-lifted --stop-available

# 仅用于排查旧固件；正式导航始终使用比赛 19 字节
srm-pulse x --legacy20 --value 0.05 --seconds 0.3 --wheels-lifted --stop-available
```

两个确认参数按字面表示架空与可断动力，不应在落地时虚填。`--value` 是协议值，不能据此直接计算距离。当前车辆已观察到 `+x` 向前、`+y` 向左；换车仍需核对。

工具先发 3 秒零速，有有效反馈后才点动，随后发 1 秒零速并清理串口；中断也尝试回零。`--legacy20` 使用旧版 20 字节布局和发送后约 5 ms 等待。结果保存到 `artifacts/field/manual-pulse-*/`；`valid_feedback_packets` 只表示有效回包数，物理运动和实际停下仍由操作者观察。此工具不主动断开链路，不是断链停车验收命令。

## 其他常用命令

| 命令 | 用途 |
|---|---|
| `srm-nav topics` | 列出本项目 domain 的话题 |
| `srm-nav hz /livox/lidar` | 点云接收频率；Ctrl-C 结束，现场通常约 10 Hz |
| `srm-nav hz /livox/imu` | IMU 接收频率；短窗口订阅计数不等于设备生成频率 |
| `srm-nav tf` | 默认查看 `map → base_footprint`；可另传父、子坐标系 |
| `srm-nav standby` | 最新原图锁定待机，不启动串口桥，任务启动被拒绝 |
| `srm-nav check-standby` | 对已启动的 standby 激活生命周期并检查零速待机 |
| `srm-nav bag-info` | 查看最新已收尾的录制 |
| `srm-nav bag-check` | 校验最新已收尾的录制并保存报告 |
| `srm-nav map-check` | 检查最新原图的栅格通行性；不能代替完整任务路线检查 |
| `srm-nav zero` | 独占唯一匹配串口，仅发送零速 8 秒并记录反馈 |
| `srm-nav ros <ROS2参数>` | 在项目环境执行任意 `ros2` 子命令，其影响取决于参数 |

独立检查报告位于 `artifacts/diagnostics/`。命令找不到时，先运行 `export PATH="$HOME/.local/bin:$PATH"` 并检查安装链接；不需要改系统 PATH 或其他用户配置。

上述地图、bag 和串口命令都保留可选路径参数。例如 `srm-nav field artifacts/mapping/某次/map`、`srm-nav bag-info artifacts/mapping/某次/bag`、`srm-map --bag artifacts/mapping/某次/bag` 可固定输入；目录不存在或没有有效候选时明确报错，不猜测使用归档副本。`srm-nav drive` 重复启动会直接提示已有实例，不再拉起第二套导航。
