# 实车传感器与底盘调试

当前实车入口用于传感器诊断和独立通信调试，尚未组成已验收的实车自主导航链。完整构建与桌面环境先按 [NUC 部署](../deployment/nuc.md)准备；硬件初值和协议冲突见[参考接口](reference.md)。

## 生成独立参数

在 NUC 项目根目录确认串口、雷达连接和实际地址：

```bash
ip -br address
ls -l /dev/serial/by-id/
id
```

按参考地址生成新目录；现场地址不同时修改参数：

```bash
python3 scripts/prepare-vehicle.py \
  --output cache/vehicle-01 \
  --host-ip 192.168.1.50 --lidar-ip 192.168.1.184
```

生成器不配置网卡、不连接设备，拒绝覆盖已有目录。外参、速度映射和固件状态仍需实测。

## MID-360 与 Point-LIO

```bash
export INTRAMURAL_NATIVE=1
export INTRAMURAL_VEHICLE_CONFIG="$PWD/cache/vehicle-01"
scripts/vehicle-sensors.sh
```

此入口只启动公开 Livox 驱动和 Point-LIO，不启动底盘、任务或速度发布者。使用 LivoxCustomMsg，真实点内时间由驱动提供，不使用仿真 `lidar_adapter`。

另一个终端设置相同运行环境，检查实际数据：

```bash
scripts/run.sh ros2 topic info /livox/lidar -v
scripts/run.sh ros2 topic hz /livox/lidar
scripts/run.sh ros2 topic hz /livox/imu
scripts/run.sh ros2 topic hz /intramural_robot/aft_mapped_to_init
scripts/run.sh ros2 topic hz /intramural_robot/cloud_registered
```

参考点云为 10 Hz、IMU 为 200 Hz，还需核对点内时间、IMU 单位、轴向、静止初始化和位姿漂移。频率正常不等于定位正确。该入口输出 Point-LIO 的 `camera_init` / `body`，尚未构成实车 `map → odom → base_footprint` 导航链。

## 底盘通信门禁

真实串口打开前，需要各项实际验证记录：

| 验证字段 | 必须确认的事实 |
|---|---|
| `firmware_protocol_verified` | 当前固件接受比赛 19 字节帧、CRC、递增序号和分片输入 |
| `link_loss_stop_verified` | 真实断链可停车，或存在经测试的等效硬件停机机制 |
| `calibrated_for_vehicle` | 逐轴方向、速度比例、坐标映射和限幅已标定 |

`base-vehicle.json` 默认三个字段均为 false，映射为空；串口节点在打开设备前拒绝未验证配置。架空和低速测试需有实际有效的停机手段，遥控器模式与拨杆映射按当前固件确认，不能用历史参考替代测试。

完成验证后，独立通信入口为：

```bash
scripts/run.sh ros2 run intramural_base serial_bridge --ros-args \
  -r __ns:=/intramural_robot \
  -p simulation:=false \
  -p device:='/dev/serial/by-id/<实际设备>' \
  -p calibration:="$PWD/cache/vehicle-01/base-vehicle.json"
```

将设备占位符替换为实际路径后执行。此节点是通信层，不是完整自主导航入口。上位机零速、取消和看门狗不能保证物理断链后的停车；模拟下位机的末帧保持现象见[调试日志](../debug-log.md)。

## 接入完整导航前

准备实车现场地图和实测安装外参，确认 TF 每个变换只有一个发布者，核对初始化、时间戳、车体轮廓、制动净空和低速运动。真实控制链仍需保持 `motion_guard → serial_bridge` 的唯一出口；不能把仿真时钟、平面驱动、理想位姿或解析先验直接用于实车验收。
