# 轨迹引擎动态避障

`navigation_engine=trajectory`、`allow_bypass=true` 时，新增后台绕行规划。静态障碍保持在场时可以生成替代路线，继续完成有序八字并返回；输入失效、当前停车包络被占用或没有可执行通道时仍停车。该功能处于验收候选阶段，现有配置默认关闭。

雷达原始点直接进入最终防撞保护。规划链在同一运动补偿结果上保留支持点数，压缩、过滤静态地图背景、聚类并做带衰减加速度的卡尔曼跟踪。首次观测即参与检查，后续观测估速，短遮挡保留预测占用。运动状态要求导数超过其估计误差；低可信状态仍保留微小速度的恒速外推，以及时响应起动，但不外推低可信加速度。观测包围轮廓与车辆圆包络分别计算，不把方形目标统一扩大成外接圆。

目标状态含位置、速度及短时衰减加速度。衰减常数 0.4 s 和过程噪声属于动态目标假设，不能称为实测底盘参数；其作用是覆盖静止目标突然起步，避免恒速预测在启动时持续落后。正常巡航和替代路线均检查实际响应及完整停车尾段。预测默认覆盖 2 s，并检查对应停车尾段；停稳车体保持占用直到整个预测窗口结束，防止随后横穿的障碍撞向停车位置。

自运动补偿不会把移动目标的旧测量变成当前测量；滤波器使用实际点内源时间及延迟观测矩阵。移动模型短暂保持，避免单帧估速靠近零时立即撤掉预测余量、反复加减速。几何搜索使用当前轮廓与定位误差，速度预测误差在带时间轨迹上统一计算。基础误差余量是这份误差预算的下限，预测仅补足超出部分，避免重复膨胀。

后台进程最多处理一个请求和一个最新待处理请求。候选限定在当前和相邻有序阶段，对障碍两侧及三个后方接回位置搜索；A* 考虑净空，曲线平滑后重新检查地图、原任务走廊、绕数、动态碰撞和实际控制响应。候选速度只修改冲突与接回区间，完成时间评分覆盖剩余任务，移动目标同时比较沿线减速。当前通道仍有效时，换边需达到收益门槛。

有远期冲突但尚无有效替代路线时，可继续执行检查过的 0.3 秒近期轨迹及其完整停车尾段；检查时限过期则停车，不因后台正在计算就固定停稳。80 ms 是结果接收预算，超时结果丢弃，不能把 Python 数值求解描述为硬实时抢占。

新轨迹保留非零边界状态，预测采用实际定位更新周期。执行时再次检查当前位置、最新观测及障碍预测，只接收同一运行和路线来源的有效结果。原任务阶段进度与替代轨迹弧长分别记录；原入场通道改变后，返回以实际入场路线逆序为基准。停车位和已完成阶段不迁移。

`mission/trajectory` 原子包含指令、运行/路线身份、定位源时间和近期路径。新引擎的保护层仅执行该快照中的指令；原 Twist 保留诊断。控制器为 40 Hz，最终保护检查为 100 Hz，雷达处理仍按真实观测频率运行。取消和运动禁止始终优先。

## 验证入口

所有输出使用新的项目内 `artifacts/` 子目录，原始地图和 bag 只读。

```bash
scripts/verify-avoidance.sh --suite model --output artifacts/avoidance-model
scripts/verify-avoidance.sh --suite replay --output artifacts/avoidance-replay
scripts/verify-avoidance.sh --suite gazebo --output artifacts/avoidance-gazebo
scripts/verify-avoidance.sh --suite all --output artifacts/avoidance-all
```

`model` 默认运行 300 个固定种子的几何及响应模型场景；`gazebo` 默认运行 60 次场景及进程停更/模拟断链检查。`--cases` 和 `--case-offset` 用于明确标记的开发子集。无障碍增量耗时对照使用同一力驱动、传感器定位和串口链：

```bash
scripts/verify-avoidance.sh --suite gazebo --baseline --cases 10 --output artifacts/avoidance-off
scripts/verify-avoidance.sh --suite gazebo --cases 10 --output artifacts/avoidance-on
scripts/verify-avoidance.sh --suite gazebo --cases 1 --case-offset 12 --gui --output artifacts/avoidance-gui
```

物理场景使用自有 `intramural_sim` Fortress 插件，由延迟指令、实测有效响应和驱动力上限生成力，让刚体引擎积分；保持原模型碰撞与传感器。固定轮组和简化接触仍是仿真假设。只有显式提供 `drive_acceleration_limit` 的仿真配置使用相应制动包络，实车不得用命令加速度冒充测得的制动力。验收器用真值检查实际净空、绕数和停车，不把真值交给传感器定位。

传感器桥沿用已安装 `ros_gz_bridge` 的转换，在退出前显式释放桥端点和定时器，再关闭 ROS 上下文；所有权释放另有 C++ 回归检查。

动态演员也由有限力驱动，其加速度和响应时间是场景假设。先运行独立无障碍圈建立演员绝对时序，后续演员不根据避让后的车辆位置追车；另用该固定时序与无避让参考检查是否确实形成交会冲突。提前让行后实际距离较远仍可有效，缺乏交会的场景不会计为动态通过。

重放分两层：录制位姿用于组件级时间补偿与目标跟踪；原始 Livox/IMU 在隔离图中以 1× 运行 Point-LIO、小型配准和零指令模拟串口，禁止同时注入录制 TF/里程计。历史观测不会随新路径改变，因此重放不能证明车辆沿反事实路线完成绕行。人工移动的仿真目标和几何压力场景另行标记。

每轮记录源码、配置、输入身份、命令、退出码、轨迹、净空和耗时。功能结果与计算耗时分别判定，失败记录保留。静态检查、模型、重放、物理仿真和实车验收分别解释。

## 配置与现场边界

`allow_bypass=false` 保持原挡停、清障后恢复。已有冻结地图配置不自动修改；显式启用时，使用 `srm-nav field MAP_DIR --allow-bypass --output cache/navigation-bypass` 在新目录重新生成；关闭使用 `--no-bypass`，原地图和配置保持不变。仍使用现有 `srm-nav` 连接、定位、待机、显式 drive/start、cancel/reset 流程；不要直接改带哈希的 `arena.json` 后继续使用旧缓存路线。

诊断查看 `mission/status` 中的 `behavior`、`obstacle_version`、`route_id`、`original_progress`、`bypass_count` 和 `avoidance`；后者含求解时间、速度倍率、接回位置和拒绝原因。被接受的替代路线保存为运行目录中的 `route-<id>.json`。关闭开关并重新生成配置可回到挡停行为；基线恢复以冻结版本和对应个人地图配置为准。

目前已有低速有效响应与停车日志，完整的高速、正反向及侧向无遮挡制动样本仍需现场补采。使用 `scripts/measure-response.sh plan --axis both --levels 0.5 1.0 1.5 --repeats 2` 先检查采集序列；真正执行仍需现有车辆限幅、操作者和停机条件。正式当前上限之外的速度不得靠修改工具绕过。软件验收不能代替 NUC 性能测量和现场静态、动态绕行验收。

搜索代价参考 [Nav2 Smac 2D 的净空代价设计](https://api.nav2.org/nav2-humble/html/smac__planner__2d_8cpp_source.html)，在项目已有 A* 中实现；力驱动使用 Fortress 的 [ExternalWorldWrenchCmd 世界坐标力接口](https://gazebosim.org/api/gazebo/6/gz_2sim_2components_2ExternalWorldWrenchCmd_8hh.html)。本轮保留现有控制器和最终速度所有权。
