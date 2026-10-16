# 实车调试命令手册

三个个人用户全局入口：`srm-map` 建图、`srm-pulse` 点动、`srm-nav` 日常检查与待机调试。源码位于项目 `scripts/`，命令通过 `~/.local/bin` 的符号链接调用，修改源码后无需重新安装。

## 在哪里运行

- 实车设备操作在 **NUC 的 qpj 终端**运行；从笔记本先执行 `ssh nuc`。命令不会自动连接另一台机器。
- WSL 也注册了同名命令，可查看帮助及检查本地备份；ROS 环境由 `scripts/run.sh` 选择原生 Humble 或项目 Docker。容器模式下数据放在项目目录内。
- 重装或换项目位置后，在项目根执行 `scripts/install-cli.sh`。无需 sudo；已有其他同名命令会被拒绝覆盖。若当前 shell 找不到命令，重新登录或执行 `export PATH="$HOME/.local/bin:$PATH"`。
- `nuc-desktop` 是已有的 **WSL** 桌面连接命令，负责个人远程桌面，不启动雷达或底盘。

```bash
srm-nav help
srm-pulse --help
srm-map --help
```

## 常用命令

| 操作 | 命令 | 影响 |
|---|---|---|
| 主机、网卡、设备、相关进程和雷达端口 | `srm-nav status` | 只读，不打开串口 |
| ROS 话题 | `srm-nav topics` | 只读，本项目默认 domain 71 |
| 雷达频率 | `srm-nav hz /livox/lidar` | 订阅，Ctrl-C 结束 |
| IMU 频率 | `srm-nav hz /livox/imu` | 订阅，Ctrl-C 结束 |
| 里程计频率 | `srm-nav hz /intramural_robot/aft_mapped_to_init` | 订阅，Ctrl-C 结束 |
| 地图到车体 TF | `srm-nav tf` | 订阅，默认 map → base_footprint |
| 指定 TF | `srm-nav tf odom base_footprint` | 订阅，Ctrl-C 结束 |
| 原生 ROS 命令 | `srm-nav ros node list --no-daemon` | 按实际 ROS 子命令执行 |

`status` 能发现已知旧哨兵进程和端口冲突，但普通用户不能查看其他用户的全部文件描述符，不能仅凭输出为空宣称串口全机独占。NUC 重启后必须重新检查旧程序自启；ROS domain 不隔离物理雷达和串口。

## 建图、导出与重载

NUC 任意目录执行：

```bash
srm-map
```

静止初始化后采集，扫完回起点停车，按 **Enter** 正常保存。工具继续校验 bag、导出地图、重启传感器验证地图配准并收尾。**Ctrl-C 取消**，已录制的数据保留，不当作完整验收成功。

```bash
srm-map --duration 120
srm-map --bag /绝对路径/已关闭的bag
```

默认车辆配置为项目 `cache/vehicle-current`；可用 `--vehicle-config` 指定。默认输出在项目 `artifacts/mapping/时间戳/`，包含原始 bag、PCD、栅格、校验结果、重载验证与进程日志。地图坐标与场地几何说明见[实车导航待机流程](vehicle/navigation-standby.md)。

**流程完成不等于地图能导航。** 检查 `map-quality.json` 的 `navigation_grid_has_clearance`，并另外核对比赛路线。原实验室地图的三维局部配准通过，但仅按地面回波投影的二维空闲区域零碎；后续射线建图候选已出现符合 0.29 m 半径的空闲区域，仍需逐次检查完整路线及重载结果。不能靠缩小车体半径把检查变绿。比赛场地变化后重新采集。

仅对正常结束的录制执行：

```bash
srm-nav bag-info /绝对路径/bag
srm-nav bag-check /绝对路径/bag
srm-nav map-check /绝对路径/map
```

`map` 目录应含 `manifest.json`、`arena.json`、`map.yaml`、`map.pgm`、`prior.pcd`。检查报告自动写入 `artifacts/diagnostics/命令-时间戳/`。`map-check` 完成报告写入后仍需阅读判定字段；退出码 0 不表示地图可通行。不要对活动录制的 SQLite 文件执行分析或统计查询。

## 单独启动传感器和导航待机

以下用于分层排障，**不要与正在执行的 `srm-map` 重复启动**。

终端一：

```bash
srm-nav sensors
# 或指定已有个人配置：
srm-nav sensors /绝对路径/vehicle-config
```

终端二：

```bash
srm-nav standby /绝对路径/map
```

终端三：

```bash
srm-nav check-standby /绝对路径/map
```

待机入口不启动串口桥，任务启动被锁定，最终速度为零。检查命令会激活地图、控制器、速度平滑的生命周期，并验证数据、TF、拒绝任务启动及零速；它不是纯只读命令，也不构成实车运动验收。

正常退出：先执行 `srm-nav standby-down`，确认成功，再分别在导航和传感器终端按 Ctrl-C。不要用全局 `pkill` 结束其他人的任务。

## 串口零速与架空点动

仅发零速诊断仍会打开串口，须先确认没有其他控制程序：

```bash
srm-nav zero /dev/serial/by-id/实际设备名
```

约 8 秒发送零速，保存回包及统计。收到状态帧不等于速度指令已被 MCU 接收/执行。

点动要求四轮可靠架空、云台旋转范围安全，并有人可立即切断电机动力。每次只执行一条，观察结束能否停下：

```bash
srm-pulse x --wheels-lifted --stop-available
srm-pulse y --value -0.02 --seconds 0.3 --wheels-lifted --stop-available
```

默认协议 19 字节，数值 0.02、持续 0.3 秒，先发零速 3 秒，结束持续回零后关闭串口。值是协议量，不是 m/s；x/y 尚未标定为实车前后左右。幅值限制 ±0.05，时长不超过 0.5 秒。

默认只在 `/dev/serial/by-id/` 下存在唯一匹配 `usb-1a86_USB_Single_Serial_*-if00` 的设备时选择该端口；没有或有多个匹配时拒绝发送。可用 `--device` 或环境变量 `INTRAMURAL_SERIAL_DEVICE` 显式指定。输出自动保存至项目 `artifacts/field/manual-pulse-时间戳/`，可用 `--output` 指定尚不存在的目录。

历史固件对照仅显式使用：

```bash
srm-pulse x --legacy20 --wheels-lifted --stop-available
```

该选项发送 20 字节整帧、13 字节数据，额外的 `is_recovering` 固定为 0；正式发送器仍为比赛 19 字节协议。两种格式当前均没有实车点动成功证据。Ctrl-C/SIGTERM 会尝试回零，但断线、断电或强制杀进程后的物理停车仍未验证。

没有“强制导航使能”“解锁实车”或批量循环点动命令。当前右上档仍可用平移摇杆驱动底盘，与参考固件的独立导航输入分支不一致；需要确认实际固件及控制模式，不能只凭本工具成功退出判定车辆已就绪。
