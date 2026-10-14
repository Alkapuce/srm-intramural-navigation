# 公开上游修复

基础提交和补丁 SHA-256 见 `../docker/sources.lock.json`。`prepare-sources.py` 在干净的固定提交上应用补丁；已有 `intramural/local-fixes` 分支则核对相对于固定上游基线的完整差异。重复执行只接受与补丁完全一致的 diff，遇到其他改动或暂存内容时停止并保留现场。提交上游修复后使用 `python3 scripts/sources.py export <仓库>` 在根目录更新补丁和校验值，先核对全部依赖，再修改指定公开源码；下方给出补丁维护步骤。

- `rmoss-gazebo-simulation-fixes.patch`：灯条原来连续两个物理步删除、重建同名 visual，异步 GUI / 传感器渲染可出现重复 visual 并崩溃。改为向原实体写入 `VisualCmd` 更新颜色，保留灯条功能。同时在底盘节点销毁控制器前停止传感器回调，通过共享互斥门闩等待正在执行的回调并拒绝排队回调，避免访问已销毁对象。依据 Gazebo 6 的 [UserCommands](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/user_commands/UserCommands.cc) 与 [RenderUtil](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/rendering/RenderUtil.cc) 实现。
- `rmu-simulator-python-entrypoint.patch`：裁判脚本添加 Python shebang，修复 `Exec format error`；捕获正常退出的 `KeyboardInterrupt` / `ExternalShutdownException`，释放 ROS 节点。

- `pb2025-terrain-shutdown.patch`：两种地形分析节点复用一个 executor，并在循环入口检查 ROS context；避免中断后重新创建 executor 的 guard condition，引发退出异常。另使底盘里程计/TF 独立于点云同步发布并保留源时间戳；初始化 LOAM 安装变换，取得有效静态 TF 前不转换点云。真实 ROS 回归见 `test_sensor_scan_generation.py`、`test_loam_startup.py`，修复对照与边界见[模型验收](../docs/validation.md)。

这些是公开仿真依赖的修复，没有引入 SRM 队内代码。保留上游版权与许可证。

## 修改与导出

根仓库的 9 个 gitlink 必须对应 `docker/sources.lock.json` 的公开提交，不能指向本机未发布的修复分支。子模块设置 `ignore=dirty`，便于应用锁定补丁后保持根状态清晰；提交前仍必须用 `sources.py status` 检查真实源码，额外改动会被拒绝。

以 pb2025 为例，在已经通过 `sources.py status` 的目录中修改：

```bash
project_root="$PWD"
source_repo=nav_ws/src/pb2025_sentry_nav
python3 scripts/sources.py status

# 首次开发：从当前已应用补丁的工作树创建本地分支，先提交登记补丁。
git -C "$source_repo" switch -c intramural/local-fixes
git -C "$source_repo" add --all
git -C "$source_repo" commit -m 'Apply locked navigation repairs'
# 修改、验证后，在该分支提交新改动。
python3 scripts/sources.py export pb2025_sentry_nav

# 导出后回到公开基线，并重新应用完整补丁；原本地分支保留。
git -C "$source_repo" switch --detach 9c124ebb0c0b6b64714390ea3c92f27490f63f9d
python3 scripts/prepare-sources.py
python3 scripts/sources.py status
git add patches/pb2025-terrain-shutdown.patch docker/sources.lock.json
```

若本机已有该分支，先检查完整锁定补丁，反向应用该补丁、执行 `git -C "$source_repo" reset --quiet` 清除补丁新增文件的 intent-to-add，再切换已有分支。以上只适用于没有额外改动且刚通过源码检查的工作树；遇到拒绝先保留并检查，不强制覆盖。根仓库不登记本地修复提交，版本升级需同时核对 gitlink、锁文件和补丁。
