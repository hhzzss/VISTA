# VISTA

基于 [RACER](https://github.com/hhzzss/RACER) 的无人机自主探索研究工程，围绕**局部 ESDF 更新范围**与**目标方向引导的局部轨迹采样**开展改进与验证。当前研究对象是单架无人机；仓库保留 RACER 的多机框架，但不宣称新增了多机协同算法。

**当前状态：研究原型。** 已有窗口接口、计算范围切换、椭球采样函数、可视化和单机适配代码；方向采样与默认探索主链路仍存在接入缺口，并发现全局引导轨迹状态初始化问题。当前版本不应作为已完成性能验证或可实飞版本使用。详见[实现对照与问题清单](docs/RACER_CHANGES.md)及[验证记录](docs/VALIDATION.md)。

## 阅读导航

| 文档 | 内容 |
| --- | --- |
| [项目结构](docs/PROJECT_STRUCTURE.md) | 文件夹用途、25 个有效 ROS 包、关键入口及数据流 |
| [相对 RACER 的改动](docs/RACER_CHANGES.md) | 逐项代码证据、实现边界、与开题报告的对应关系 |
| [运行与复现](docs/REPRODUCIBILITY.md) | 环境依赖、单机启动、配置方法与实验准备 |
| [验证记录](docs/VALIDATION.md) | 本次实际执行的检查、结果及尚未完成的验证 |
| [配置速查](swarm_exploration/config_changes.md) | 窗口参数、默认值与修改位置 |
| [后续交接](HANDOVER.md) | 继续研究时的优先工作 |
| [来源说明](THIRD_PARTY.md) | 上游提交、原论文与第三方内容 |

## 当前改动

1. **局部 ESDF 更新窗口。** 增加 `use_local_window`、窗口尺寸与中心更新接口。启用时，距离变换使用跟随当前位置的轴对齐窗口边界；关闭时使用原有的局部更新包围盒。地图缓冲区仍按完整地图尺寸分配。
2. **方向引导的椭球轨迹采样。** 增加长半轴指向目标、短半轴比例可配置的采样方法，并在 `paramLocalTraj()` 中选择球形或椭球形区域。该分支属于已有全局/拓扑轨迹处理链路，尚未接入默认的探索轨迹生成路径。
3. **单机适配与诊断。** 增加无网格分配时的前沿目标回退、空邻机轨迹时不启用 SWARM 代价的处理，以及局部窗口和目标方向 Marker。
4. **发布整理。** 更正 RACER 论文引用和项目归属，补充中文解释文档，去掉启动脚本中的机器专属路径及全局杀进程命令，并停止跟踪历史编译产物。

**贡献边界：** `bspline/` 与 `bspline_opt/` 的样条表示、主要目标函数及优化器继承自 RACER；本版本没有实现新的 B 样条求解算法、EOHDT 或 ESDF-free 避障。仓库中没有可支持内存、耗时或探索效率提升百分比的正式实验结果。

## 构建入口

原 RACER README 提供 Ubuntu 18.04 / ROS Melodic 与 Ubuntu 20.04 / ROS Noetic 的使用说明；这不代表 VISTA 已在两套环境中重新通过验证。优先使用 Ubuntu 20.04 / ROS Noetic，并先阅读[本次检查结果](docs/VALIDATION.md)。

项目是由多个 catkin 包组成的源码集合，放在工作空间的 `src/VISTA` 下：

```bash
mkdir -p ~/vista_ws/src
cd ~/vista_ws/src
git clone https://github.com/hhzzss/VISTA.git
cd ~/vista_ws
source /opt/ros/noetic/setup.bash
# 先按 docs/REPRODUCIBILITY.md 安装并检查依赖
catkin_make -j2
source devel/setup.bash
```

## 单机仿真入口

在正确构建和 source 工作空间后：

```bash
roslaunch exploration_manager single_drone_exploration_with_map.xml map_name:=pillar.pcd
```

该入口包含地图发布、单机规划、轻量仿真器和 RViz。用 RViz 的 `2D Nav Goal` 触发探索。`simulator_light.xml` 实际使用 `poscmd_2_odom` 与点云传感渲染，不能直接称为已完成 Gazebo 物理仿真验证。

可选地图：`pillar.pcd`、`office.pcd`、`office2.pcd`、`office3.pcd`。更换地图时还需核对地图尺寸、探索边界与起点，不能只替换文件名。具体命令、原多机入口和现有局限见[运行说明](docs/REPRODUCIBILITY.md)。

## 研究背景

对应课题为“基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究”。研究关注窗口改动怎样影响距离与梯度查询、局部规划及闭环探索。开题报告中的 G0 / G1 / G2 消融、45 次任务和数据采集方案属于后续实验计划；本仓库尚未提供完整实施结果。开题报告原件保留在本地论文资料目录，不作为本次公开代码发布的一部分。

## RACER 来源与引用

基线提交：`049c332e3634ef72d8beb155b4c13dc91ca52916`。2026-11-16 核对时，本地基线、`hhzzss/RACER` 与上游 `Robotics-STAR-Lab/RACER` 的 `main` 指向同一提交。原项目 README 原文保存在 [docs/upstream/README.RACER.md](docs/upstream/README.RACER.md)。

引用 RACER 时请保留原论文题名与作者，不能把 RACER 的论文、演示或获奖信息改署为 VISTA：

```bibtex
@article{zhou2023racer,
  title={RACER: Rapid Collaborative Exploration With a Decentralized Multi-UAV System},
  author={Zhou, Boyu and Xu, Hao and Shen, Shaojie},
  journal={IEEE Transactions on Robotics},
  year={2023},
  volume={39},
  number={3},
  pages={1816--1835}
}
```

[原论文](https://arxiv.org/abs/2209.08533)及[上游仓库](https://github.com/Robotics-STAR-Lab/RACER)。第三方说明见 [THIRD_PARTY.md](THIRD_PARTY.md)。
