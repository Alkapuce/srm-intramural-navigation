# NUC 离线部署

流程是开发机准备完整源码，通过局域网同步到 NUC，再在 NUC 的原生 ROS 2 Humble 环境编译和桌面调试。NUC 当前没有外网，不在 NUC 上直接从 GitHub 下载。

## 开发机准备交付目录

在已经提交要部署代码的项目根目录执行；交付目录必须尚未存在：

```bash
project_root="$PWD"
delivery_root="$project_root/artifacts/deployment/nuc-source"
git clone --no-local --single-branch --branch main --recurse-submodules \
  "$project_root" "$delivery_root"
cd "$delivery_root"
python3 scripts/prepare-sources.py
python3 scripts/sources.py status
```

此目录只包含 `main` 的已提交内容，子模块取公开基线，再应用登记补丁。`.git/modules/` 是子模块版本信息的一部分，离线同步时须连同根 `.git` 保留。

将 NUC 与开发机连接到同一局域网，填写个人用户和实际地址：

```bash
nuc_target='<NUC_USER>@<NUC_IP>'
ssh "$nuc_target" 'mkdir -p ~/workspaces/srm/intramural'
rsync -avz --progress \
  --include='/cache/' \
  --include='/cache/small_gicp/***' \
  --include='/cache/livox-sdk2/***' \
  --exclude='/cache/***' \
  --exclude='artifacts' \
  --exclude='nav_ws/build' \
  --exclude='nav_ws/install' \
  --exclude='nav_ws/log' \
  --exclude='simulation/generated' \
  --exclude='/config/local/***' \
  --exclude='.env*' --exclude='__pycache__' \
  --exclude='core' --exclude='core.*' \
  "$delivery_root/" "$nuc_target:~/workspaces/srm/intramural/"
```

已有 NUC 工作目录有个人修改时先保留，首次部署可使用新目录。更新时从新的已提交版本重新准备交付目录，再同步；不要直接覆盖仍在运行或有未保存修改的工作区。

## NUC 核查环境

在 NUC 桌面终端执行：

```bash
source /opt/ros/humble/setup.bash
id
cat /etc/os-release
command -v ros2 colcon python3
ign gazebo --versions
glxinfo -B
python3 -c 'import xacro, numpy, scipy'
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix nav2_controller
```

完整构建还需要 PCL、Eigen、glog；Livox SDK2 使用交付目录中固定版本的 `cache/livox-sdk2` 源码，构建为驱动私有静态库。现场二维导图还需要 `c++` 与 `liboctomap-dev`（头文件和 OctoMap 库），依赖依据见 [Dockerfile](../../docker/Dockerfile)。宿主依赖缺项由赛务补齐或在个人前缀准备，不修改共用系统配置。NUC 当前的图形和设备需要现场核查，不能沿用开发机 GPU 验证结论。

## 全量构建

```bash
cd ~/workspaces/srm/intramural
python3 scripts/prepare-sources.py
python3 scripts/sources.py status
export INTRAMURAL_NATIVE=1
scripts/build-workspace.sh
scripts/run.sh python3 -m unittest discover -s /work/tests -v
```

统一入口指定本地 small_gicp、Livox SDK 源码并关闭桌面通知，避免离线下载和 SSH 下的通知阻塞。只编译任务包和雷达驱动不足以运行完整仿真或当前全部测试，因此这里使用完整工作空间构建。

## 桌面仿真

每个本任务终端一致设置通信域与分区；有其他选手共用网络时使用现场分配值：

```bash
cd ~/workspaces/srm/intramural
export INTRAMURAL_NATIVE=1
export INTRAMURAL_ROS_DOMAIN_ID=71
export INTRAMURAL_IGN_PARTITION=srm_intramural
scripts/candidate.sh serial-sim
```

另开桌面终端，设置相同变量后查看任务状态；`ready=true` 时调用启动服务：

```bash
scripts/run.sh ros2 topic echo /intramural_robot/mission/status
scripts/run.sh ros2 service call /intramural_robot/mission/start std_srvs/srv/Trigger '{}'
scripts/run.sh ros2 service call /intramural_robot/mission/cancel std_srvs/srv/Trigger '{}'
scripts/run.sh ros2 service call /intramural_robot/mission/reset std_srvs/srv/Trigger '{}'
```

订阅状态后按 `Ctrl-C` 结束该订阅，再执行服务命令。取消不传送机器人，重复启动要求回到停车位；在主启动终端按 `Ctrl-C` 正常结束仿真。

批量 `verify-stage*.sh` 和 `reproduce-candidate.sh` 目前使用容器路径，在开发机运行；NUC 原生仿真使用上面的桌面启动和服务操作。参数与 TF 详情见[仿真说明](../simulation/arena-mission.md)，实车传感器和串口调试见[接车调试](../vehicle/commissioning.md)。
