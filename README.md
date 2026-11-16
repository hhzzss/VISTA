# 基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究

基于 [RACER](https://github.com/hhzzss/RACER) 的无人机自主探索研究工程。研究重点是局部 ESDF 更新范围、目标方向引导的轨迹截取，以及既有 B 样条优化与自主探索流程之间的接口。

**当前版本：第二版。** 2026-11-16 按用户指定的 `E:/VISTA` 完整替换源码树、配置、仿真资源和脚本；第一版不再作为当前实现依据。用户反馈第二版运行效果优于第一版，目前没有随版本提供可复算的定量对照数据。

项目展示名称使用上面的完整中文题名。`VISTA` 仅保留为 GitHub 仓库名和克隆目录标识；ROS 包名及节点接口沿用源码。它不是上游 RACER 论文的名称，也不将上游作者和获奖信息归为本课题成果。

## 内容导航

| 文档 | 内容 |
| --- | --- |
| [项目结构](docs/PROJECT_STRUCTURE.md) | 全目录与各 ROS 包职责、建议阅读顺序 |
| [相对 RACER 的改动](docs/RACER_CHANGES.md) | 地图窗口、椭球采样、新模块、实际接入状态 |
| [第二版替换说明](docs/VERSION_2.md) | 与此前发布版本的区别、完整同步规则 |
| [构建与运行](docs/REPRODUCIBILITY.md) | ROS 依赖、原始入口、当前运行限制 |
| [核验记录](docs/VALIDATION.md) | 本版本检查结果与未覆盖范围 |
| [配置说明](swarm_exploration/config_changes.md) | 真实参数、默认值和可视化 |
| [新增模块状态](swarm_exploration/PHASE2_SUMMARY.md) | 聚类、EOHDT、点云碰撞检测 |
| [交接说明](HANDOVER.md) | 当前版本边界与后续研究衔接 |
| [来源与第三方说明](THIRD_PARTY.md) | RACER 基线、继承组件及原始说明 |

## 当前实现

- **局部 ESDF 更新窗口**：在原有地图融合框架上增加可配置的、跟随探索规划位置的轴对齐计算范围。完整地图缓冲区仍保留。
- **目标方向椭球轨迹截取原型**：增加目标和方向接口、椭球内轨迹采样及球形回退分支。默认探索链路尚不能据此认定已经使用椭球采样。
- **B 样条优化接口**：沿用 RACER 的样条表示、主要代价与求解方法，距离代价通过地图查询接口受到局部窗口语义影响。
- **可视化与配置**：局部地图窗口 Cube、目标方向 Arrow、单机 launch 参数及 RViz 配置。
- **第二版新增源文件**：点云聚类/拓扑图、EOHDT 分层求解、KD-tree 点云碰撞检测。源码已经收录，但默认规划调用尚未接通；点云聚类源文件未加入其 CMake 目标。

第二版保留了自身的算法和脚本行为，没有从第一版拼回前沿回退、SWARM 条件开关或全局轨迹时长保护。详见[版本差异](docs/VERSION_2.md)。

## 构建与运行入口

使用 Linux + ROS Noetic/catkin。构建前检查 NLopt、PCL、Eigen、OpenCV、Armadillo 和外部 LKH，具体路径要求见[复现说明](docs/REPRODUCIBILITY.md)。

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/hhzzss/VISTA.git
cd ..
source /opt/ros/noetic/setup.bash
catkin_make
```

上述为构建步骤，不表示本版本已经完成整工程构建或闭环验收。依赖和已知代码限制以[核验记录](docs/VALIDATION.md)为准。

原有单机入口是 `roslaunch exploration_manager single_drone_exploration.xml simulation:=true`，RViz 入口是 `roslaunch exploration_manager rviz.launch`。单机入口并不自行发布全局地图，需配套地图节点；新带地图 XML 当前只有节点片段，不能直接作为完整启动入口。源码附带的两个 `test_*.sh` 含固定工作空间或全局 `pkill`，本次为忠实同步第二版保留，**不建议直接执行**。`apply_phase2.sh` 同样只作原始工具保留。

## 研究与证据边界

开题报告聚焦单无人机、局部 ESDF 与方向机制的核验、B 样条耦合和消融实验；多机框架仍由 RACER 继承。新增 Phase 2 源码不代表已经替换原 ATSP 或实现完整 ESDF-free 优化。现阶段不宣称内存降低倍数、固定毫秒加速、效率提升百分比或完整实验已经完成。

上游演示资源、论文和作者信息见[第三方说明](THIRD_PARTY.md)，不作为第二版改进实验的证据。
