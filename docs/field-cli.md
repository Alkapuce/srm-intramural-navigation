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

在场地内保持车辆静止启动，**车头朝向可以任选，不必正对两桩**：

```bash
srm-map
```

复用 9.43 秒实测版的控制参数并适配新图，使用 `srm-map --profile candidate`。此分支的 candidate 固定使用相切圆弧路线，按新桩位、间距及停车点重算半径、平滑出入路径和速度曲线；速度上限 2.0、切向加速度 3.5、横向加速度 2.5、末端减速度 1.5。固定的是参数和路线族，不复用旧图坐标，也不保证新场地达到相同用时。两侧路线均需通过新地图净空及已有响应模型检查；较窄场地可能无法生成该路线。

先保持静止，等终端出现“录制中”，再由操作者缓慢移动整车，覆盖两桩、两侧绕行区域及出入路线。可在电机禁用后手推；若使用遥控驾驶，需由现场人员控制，本命令本身不发送底盘速度。

采图过程中可以正常移动和转弯。覆盖完成后，把车停在**本次任务选定的起始位置**，保持静止，在建图终端按 **Enter**；不必精确摆回采图开始的位置和朝向。

程序依次收尾录包、校验数据、导出三维和二维地图、重启传感器并重定位。配准后读取车辆在新图中的当前位置，作为任务起点和返回点，再识别两桩，联合搜索整圈路线与速度，保存结果并释放本次进程。显示 `bag-quality`、`export-map`、`reload-check` 等阶段名时仍在处理，请等待结束。这里记录的是车辆位置，不会识别地面的停车标记；操作者须把车停在赛方规定的位置。

导航不要求车头始终朝向采图方向：定位连续运行时可以跟踪朝向变化；重启后启动 `drive` 时会自动匹配当前点云和旧图的位置、朝向，再由 `start-here` 接入路线。当前导航仅控制全向平移、角速度为零，不会主动把车头转到路线切线方向。

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
| `navigation/route-plan.json`、`route-plan.png` | 冻结的整圈路线、速度曲线及预览 |
| `navigation/route-analysis.json` | 起点来源、候选搜索与最终路线检查 |
| `reload-check/`、`result.json` | 重载定位结果、阶段记录和进程收尾结果 |

建图结束自动导出 PNG 到本机 `artifacts/.png/`，终端只打印结果和文件位置。普通地图保留障碍与未知区域；已有规划时同时保存路线、速度高清图。文件名包含来源目录，各次建图互不覆盖。

```bash
# 将最新地图/路线导出为 PNG，不启动 ROS 或设备
srm-nav preview
srm-nav preview artifacts/mapping/本次
srm-nav preview artifacts/mapping/本次/navigation --output artifacts/route.png

# 导出正常产物目录下全部已建地图，保留原始文件
srm-nav preview --all
```

WSL 可以通过现有 `ssh nuc` 自动接收 PNG。默认接收位置是 `~/workspaces/srm/intramural/artifacts/.png/`；在 WSL 安装一次用户定时器，每 5 秒同步新增或更新的图片：

```bash
mkdir -p artifacts/.png/ ~/.config/systemd/user/
cp config/systemd/srm-map-png-sync.{service,timer} ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now srm-map-png-sync.timer
```

其他项目路径可在 service 文件中修改。查看状态用 `systemctl --user status srm-map-png-sync.timer`，暂停同步用 `systemctl --user stop srm-map-png-sync.timer`。连接暂时中断时保留两端图片，恢复连接后继续同步。PNG 不参与净空计算或控制。

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

# 新图默认 safe；已有响应模型时可附加，不改原车辆参数
srm-map --profile safe --response-model artifacts/response/本次/model-0.json

# 对最新原图单独识桩与联合规划；默认停车点为建图原点，不连接设备
srm-nav field

# 已启动与指定地图匹配的定位待机：把当前车位设为新的起点/返回点
srm-nav field artifacts/mapping/本次/map --parking-here --profile safe

# 离线明确指定地图坐标；不读取实车位姿
srm-nav field artifacts/mapping/本次/map --parking -0.6 0.2 --profile candidate
```

`--parking-here` 同时核对当前发布的地图和新鲜定位，不会把另一张图的位姿直接套入。重新选起点需重新生成场地配置；`start-here` 只改变本轮接入位置，返回目标仍是配置的停车点。`safe`、`candidate` 的速度、响应假设与路线优化方法见[整圈轨迹导航](architecture/trajectory-navigation.md)。

省略路径时自动选最新有效产物，执行前会打印所选目录。导航按清单文件修改时间选取，仅考虑正常产物目录中的已通过场地检查配置，不扫描归档和测试树；原图与 bag 则分别按地图清单、录包元数据修改时间选取。仍需确认所选图属于当前场地；要固定旧图时显式传入目录。

录制过程中通过 ROS 话题监控，**不要查询正在写入的 SQLite bag**；`bag-info`、`bag-check` 留到录制正常结束后使用。

## 启动导航：三个终端

调试途中需要按实际走过的路线退回停车位时，可先退出第二终端的导航，再执行 `srm-nav return-home /绝对路径/轨迹.jsonl /绝对路径/navigation`，随后在第三终端执行 `srm-nav start`。该入口倒序使用记录位姿，指令限速 0.1，只执行回位；记录需从该地图停车位附近开始且没有明显定位跳变。回位后退出第二终端，使用原导航目录重新 `drive` 才会恢复完整八字。

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

`drive` 先静止采集约 2 秒点云，搜索旧图中的位置和朝向，匹配成功后更新定位初值，再打开串口桥、激活导航节点，**不自动发车**。采样期间保持静止；传感器须已运行。需要固定已有地图时传入路径；下面 `cache/my-navigation` 是示例，可替换为实际目录。路径支持当前目录相对路径、项目内相对路径和绝对路径：

```bash
srm-nav drive cache/my-navigation
```

串口不能唯一识别时，可用 `INTRAMURAL_SERIAL_DEVICE` 指定，或作为 `drive` 的第二个参数传入。个人车辆配置不使用默认目录时，在仓库根目录设置 `export INTRAMURAL_VEHICLE_CONFIG="$PWD/cache/vehicle-01"` 并在传感器与导航终端保持一致。`INTRAMURAL_NAVIGATION_CONFIG` 可固定导航配置，优先于自动选择；显式路径参数优先于环境变量。任务、控制器和个人车辆限速须一致；入口拒绝超出个人车辆限速或 2.0 软件上限的任务配置。

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

命令接受任务后保持在当前终端，每秒显示阶段、总计时、跟踪误差和挡停次数；阶段或故障变化立即显示。正常结束打印开始/结束时间、总用时、起步点附近停留时间、最大跟踪误差、停车位置偏差和记录目录。任务计时包含切档等待；“离开起步点 5 cm 后”的时间来自定位估计，不替代比赛正式计时。Ctrl-C 请求取消本轮任务；需要从另一终端停车时仍用 `srm-nav cancel`。

正常完成后可直接再次 `srm-nav start`，命令会先复位已完成状态再发车；运行中不会重复启动，故障或取消后仍需处理原因并 `reset`。完成摘要可随时重看：

```bash
srm-nav result
srm-nav result artifacts/navigation-runs/运行ID
```

已在地图中正确定位，但车停在其他位置，调试时使用：

```bash
srm-nav start-here
```

`start-here` 从当前位置规划接入路线，**重新跑完整八字，最后返回原配置停车位**。它不继承上轮进度，不替代重定位；未知区、障碍或车体净空不足时可能找不到接入路径。

新轨迹引擎按实测地图、局部代价地图和实时点云保护，并检查完整桩底座；A* 接入与连续曲线使用一致的净空判据。`pole_clearance_from_map=true` 在旧引擎中的行为不等同于新引擎取消底座检查。0.29 m 车体、地图未知区和循迹保护保留；解析净空仍是定位与模型估计，不能当作车体到底座的独立实测间距。

新生成配置默认使用整圈轨迹引擎：先根据本次实测地图生成切入、两瓣八字和返回轨迹，再一次规划全程速度。10 个分段只作为有序进度标签，普通交接不取消目标、不归零；切入路径与主路线一起检查净空、曲率与绕桩顺序。`start-here` 会重新规划当前已定位位置的切向接入，不能保证任意障碍布局都有可行路线。

```bash
# 建图后默认使用 safe；显式生成提速候选，原图与旧配置均保留
srm-nav field artifacts/mapping/本次/map --profile candidate --output cache/navigation-candidate
srm-nav drive cache/navigation-candidate
# 等 ready 后，在另一终端显式发车
srm-nav start-here
```

`safe` 上限 0.65，`candidate` 上限 1.2；两个档位均会按曲率、矢量加速度和最终制动距离调速。个人车辆配置中的速度、加速度上限须覆盖选定档位；超出时入口会拒绝，请核对实际底盘条件后配置。`gold-reference` 只保留旧引擎参数作对照，不能当作逐字节恢复的 18.554 秒实车版本。详细设计和本机复现见[整圈轨迹导航](architecture/trajectory-navigation.md)。

路线依次为 `ENTER → LOWER → UPPER → LOWER_EXIT → RETURN → DONE`，内部仍记录 10 个有序分段。新引擎 `segment` 是分段号，`continuous_progress` 是全程有序弧长（米），`progress` 是按采样间距换算的全程索引；`tracking` 中包含剩余长度、跟踪误差和前馈/反馈。交叉点不做全图最近点匹配。每次运行生成独立 `run_id`、`route_id`，源码哈希、配置和状态保存在 `artifacts/navigation-runs/<run_id>/`。

## 停车、再运行与换电

| 需要做什么 | 命令 / 操作 | 行为 |
|---|---|---|
| 取消当前任务 | `srm-nav cancel` | 关闭任务运动使能并输出零速；观测停稳后进入 CANCELLED，未确认则进入 FAULT |
| 正常完成后再跑 | `srm-nav start` | DONE 自动复位后开始新一轮，并在当前终端显示到结束 |
| 清理结束或故障状态 | `srm-nav reset` | 等待取消结束后复位为 `IDLE`；不移动车辆、不修复故障 |
| 在当前位置重新跑一轮 | `srm-nav start-here` | 前提是问题已处理、定位正确且任务已复位 |
| 查看一次状态 | `srm-nav mission-status` | 显示状态、位姿、阶段、阻断次数与原因 |
| 查看完整结果 | `srm-nav result [RUN_DIR]` | 显示最新一轮或指定目录的结果、时间、误差和日志位置 |
| 结束导航 | `srm-nav standby-down` | 正常关闭导航生命周期；随后在终端 2 按 Ctrl-C |
| 结束传感器 | 终端 1 按 Ctrl-C | 在导航退出后停止传感器，等待正常收尾 |

`cancel` 是软件停车，不能替代现场切断动力。关机或换电前，按“取消任务 → 确认车停 → 关闭导航 → 停传感器”的顺序操作。

NUC 重启后，重新检查旧自启程序和设备占用。启动 `srm-nav sensors`，车保持静止，再启动 `srm-nav drive`；自动定位会重新搜索旧图中的位置和朝向，不要求摆回建图起点。就绪后从当前位置发车用 `srm-nav start-here`，仍返回地图配置中的停车位，不改变两桩和地图坐标。

只检查定位而不启动导航时，运行 `srm-nav localize`，或指定导航目录。结果与原始采样保存在 `artifacts/localization/`，报告包括匹配残差、重叠率和估计车体位姿；成功时仅更新下次启动使用的 `nav.yaml` 初值并保留备份，不发车。该命令要求先退出当前导航，传感器继续运行。`standby` 同样在启动前自动定位。

自动定位适用于已建图、近水平地面且有可区分几何的场景；场景严重变化、点云不足或多个位置同样匹配时会报告失败，不覆盖旧初值。确认选用当前场地地图，并核对报告位姿与实车大致一致；`ready=true` 本身不证明位置正确。必要时可在 RViz 将 **2D Pose Estimate** 的 Topic 设为 `/intramural_robot/initialpose` 后人工指定当前位置和朝向。

## 障碍停车与窄道排查

实车配置默认 `allow_bypass=false`：路线受阻进入 `BLOCKED` 并输出零速；同一未来路径上的障碍清除并稳定约 0.5 秒后，自动恢复原弧长进度。正常等待**不需要重复 start 或 reset**。持续障碍绕行仅在显式启用该策略的配置中执行，仿真说明见[任务与绕行](simulation/arena-mission.md)。

整圈引擎在阻挡前后检查同一条近期有序路线，并保留触发停车时的检查距离；停车后不会因为指令速度变为零就缩短检查范围，提前把尚未离开的障碍当作清空。点云、代价地图和定位都必须保持新鲜。真人语义识别尚未接入，未来路径中的其他真实障碍同样会停车。

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

整圈引擎通过原子的 `mission/trajectory` 消息提供带 `run_id`、`route_id` 和时间戳的未来路线；`mission/collision_path` 保留作可视化。保护检查当前运动在响应延迟内的延伸，再接入近期有序路线，并按制动距离扫掠完整车体。侧后方不在实际扫掠中的点不阻断；轨迹过期、版本不匹配或 TF 不可用时停止。旧引擎仍保留 `ordered_path/velocity` 保护路径。

窄道中反复停车时，先看触发点到**未来路径和完整车体包络**的距离，不能仅凭“物体在车侧面”判断误报。安全取消、停稳后，临时移开可移动杂物做对照；不要按“桩”或“人”的类别屏蔽点云。

实车生成配置的主要参数：

| 参数 | 值与含义 |
|---|---|
| 巡航指令上限 | safe：0.65；candidate：1.2，均为合速度限幅 |
| 正常矢量加速度 / 横向加速度 | safe：1.2 / 0.65；candidate：2.5 / 1.4 m/s² |
| 响应延迟 / 制动预测 | 0.2 s / 0.2 m/s²，仍需实车日志校准 |
| 控制方式 | 切向速度前馈＋独立切向/法向位置纠偏＋速度反馈，不再靠追固定前瞻点产生巡航速度 |
| 最终停车 | 位置 < 0.08 m、有序余程 < 0.05 m、估计速度 < 0.06 且稳定 0.15 s |
| 车体半径 / 额外静态净空 | 0.29 m / 0.02 m，地图检查另计像素和路径采样误差 |
| 同帧触发点数 | 原始扫掠范围内至少 3 点；稀疏小物体需现场复核 |
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
