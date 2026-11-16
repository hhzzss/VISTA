# VISTA 相对 RACER 的改动核验

核对日期为 2026-11-16。结论基于本地完整工作树、暂存区与基线提交的联合差异，并结合默认 launch 和实际调用路径核验。原有暂存与未暂存修改均已纳入，未只比较某一层 Git diff。

## 基线与研究范围

基线为 `049c332e3634ef72d8beb155b4c13dc91ca52916`。核对时，本地 `main`、用户仓库 `hhzzss/RACER` 和原作者仓库 `Robotics-STAR-Lab/RACER` 的 `main` 指向同一提交。VISTA 发布保留该提交的祖先历史，后续差异可直接复查。

开题报告将研究限定为单无人机的局部 ESDF 与规划接口验证。本工程保留多机相关组件，但任务分配、通信协调与新的全局探索求解器不属于当前新增贡献。本文区分“代码已存在”“默认链路已接入”和“实验已证明”，不能互相替代。

## 改动总表

以下为整理发布前已经存在的研究改动；本次没有更改这些 C++ 算法源文件。

| 模块 | RACER 基线 | VISTA 当前代码 | 核验结论 |
| --- | --- | --- | --- |
| ESDF 更新范围 | 使用 `local_bound_min_/max_`，由占据融合更新包围盒及膨胀量形成 | 可改用 `local_window_min_idx_/max_idx_` | 新增另一种局部范围策略；不是首次从全局变为局部 |
| 窗口位置 | 原更新包围盒随观测变化 | 探索规划时以传入 `pos` 为中心，尺寸可配置并裁剪到地图边界 | 轴对齐长方体平移，没有旋转或向目标偏置 |
| 地图存储 | 按完整地图体素数分配缓冲区 | 相同的完整地图分配规模 | 未实现局部滚动存储或稀疏分配 |
| 距离及梯度查询 | 地图外返回距离 0、梯度 0 | 地图外或窗口外返回 `default_dist_`、梯度 0 | 行为改变；缺少显式有效性状态，不能当作已证明安全策略 |
| 局部轨迹截取 | `getTrajInfoInSphere()` | 增加方向对齐的 `getTrajInfoInEllipsoid()` | 函数已实现，默认探索路径未调用该采样分支 |
| 方向状态 | 无对应接口 | 增加目标、方向及开关，尝试从全局轨迹状态计算方向 | 存在未初始化状态读取及链路接入问题 |
| 无网格分配 | `grid_ids.empty()` 时直接返回 `NO_GRID` | 进入原本不可达的前沿目标选择，补充空列表与索引检查 | 实际行为改动；会改变探索决策，需纳入公平基线 |
| SWARM 代价 | 两处位置轨迹优化都加入 SWARM 项 | 仅当存在有效邻机轨迹时加入 | 单机与空邻机状态适配，不是新的机间避碰算法 |
| 可视化 | 原地图、轨迹和前沿显示 | 增加窗口 Cube、目标方向 Arrow 和 RViz 配置 | 便于观察；不能证明采样机制已生效 |
| ESDF 临时数组 | `fillESDF()` 中使用变长栈数组 | 改为 `std::vector` | C++ 容器化修改，未改变距离变换核心算法 |

## 局部 ESDF 更新窗口

主要文件：[sdf_map.h](../swarm_exploration/plan_env/include/plan_env/sdf_map.h)、[sdf_map.cpp](../swarm_exploration/plan_env/src/sdf_map.cpp)、[fast_exploration_manager.cpp](../swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp)。

新增接口为 `setLocalWindowSize()`、`updateLocalWindowCenter()`、`isInLocalWindow()`、`getLocalWindow()` 和 `setLocalWindowEnabled()`。窗口为：

```text
候选下界 = 当前探索规划位置 - window_size / 2
候选上界 = 当前探索规划位置 + window_size / 2
实际范围 = 候选范围与完整地图边界的交集
```

当前 launch 的完整窗口尺寸是 `10 × 10 × 5 m`，不是三个半径。Z 方向通常会被实际地图高度裁剪。没有新建环形缓冲区，也没有释放窗口之外的地图存储。

`updateESDF3d()` 在启用窗口时改用该范围，关闭时恢复原 `local_bound_*` 范围。原范围已经局部化，固定窗口不保证比每次观测更新包围盒更小，需记录实际处理体素数和耗时才能判断收益。

窗口中心更新发生在 `planExploreMotion()` 开头，不是每帧里程计都更新。ESDF 仍由 `MapROS` 的 0.05 秒定时器在 `esdf_need_update_` 为真时执行。仅移动窗口没有直接触发或等待一次完成的距离变换，因此需要检查新范围对外可见与有效 ESDF 完成之间的时序。

初始化时窗口边界设为完整地图边界，直到第一次探索规划调用才收缩到当前位置附近。不能假定启动后的第一次 ESDF 计算已经只处理配置的小窗口。

## 距离查询与原 B 样条优化的关系

当前主要调用链为：

```text
BsplineOptimizer::calcDistanceCost()
  → EDTEnvironment::evaluateEDTWithGrad()
    → SDFMap::getDistWithGrad()
      → 栅格距离三线性插值与梯度
```

对应实现：[bspline_optimizer.cpp](../swarm_exploration/bspline_opt/src/bspline_optimizer.cpp)、[edt_environment.cpp](../swarm_exploration/plan_env/src/edt_environment.cpp)、[sdf_map.cpp](../swarm_exploration/plan_env/src/sdf_map.cpp)。这使地图窗口语义能够影响既有距离代价，但样条目标函数和求解算法本身没有重写。

需要先解决的查询问题：

- `getDistWithGrad()` 对窗口外和地图外统一返回默认距离及零梯度。C++ 默认距离为 5 m，而当前 launch 覆盖为 0.5 m；不能忽略配置差异或将该值视为真实安全间距。
- `getDistance()` 仍按完整地图读取，不检查局部窗口；不同接口可能读取默认值或过期值，缺少统一的“有效、未知、越界、过期”状态。
- 窗口边缘内的一个连续查询点，其八个插值邻点仍可能越出已更新区域；仅判断查询点中心在窗口内不足以保证插值有效。
- 距离变换只扫描窗口内部，边缘附近的窗口外障碍没有明确的距离传播保护带。需与同一已知占据状态下的参考距离场比较，检查是否高估距离。
- 地图缓冲区依旧为完整地图分配，故“窗口小 16 倍”不能推出“内存减少 16 倍”。

关闭窗口开关时，地图外返回值仍与原版不同。因此，“关闭两个新参数”只是诊断配置，不是精确原版 RACER。

## 目标方向椭球轨迹采样

核心函数位于 [plan_container.hpp](../swarm_exploration/plan_manage/include/plan_manage/plan_container.hpp) 的 `GlobalTrajData::getTrajInfoInEllipsoid()`；选择分支在 [planner_manager.cpp](../swarm_exploration/plan_manage/src/planner_manager.cpp) 的 `paramLocalTraj()`。

以采样起点 `p0` 为中心，将局部 Z 轴对齐传入方向 `d`。对位移 `Δp` 使用正交变换 `q = Rᵀ(p − p0)`，判断：

```text
(q.x² + q.y²) / b² + q.z² / a² ≤ 1
a = local_segment_length
b = local_segment_length × sliding_window_minor_ratio
```

当前参数对应长半轴 6 m、两个短半轴 3.6 m。沿全局引导轨迹以 0.2 秒步长寻找截取时间，再按控制点间距估计采样段数并调用原参数化流程。零方向退回球形函数；关闭动态开关或未满足状态条件也使用原球形函数。

**它调整的是轨迹采样/截取域，不是 ESDF 栅格窗口朝向，也不是传感器 FoV。** 不能称为“旋转局部 ESDF”或“提前感知目标方向障碍”已被实现。相同长半轴下，理想椭球体积是同半径球体的 `ratio²`，0.6 对应 0.36；这是几何关系，不是实际 ESDF 体素预算或性能测量。

### 默认链路与方向机制的接入缺口

静态调用关系显示：

```text
默认探索：
planExploreMotion → setSlidingWindowTarget → planTrajToView
  → planExploreTraj 或 kinodynamicReplan

椭球采样所在分支：
planGlobalTraj / topoReplan → paramLocalTraj → getTrajInfoInEllipsoid
```

默认 `single_drone_planner.xml` 还将 `manager/use_topo_path` 设为 `false`。默认探索的两个位置轨迹生成函数不调用 `paramLocalTraj()`，也不为该新分支设置 `global_data_` 的全局引导轨迹；`setGlobalTraj()` 的现有调用在另一条全局/拓扑规划路径。

更直接的问题是，`GlobalTrajData` 的默认构造函数为空，`global_duration_` 没有初始化，而新增的 `setSlidingWindowTarget()` 先读取它来判断是否存在全局轨迹。首次探索调用可能因此读取未初始化值；即使通过该判断，也未必有可用的轨迹供 `getState()` 访问。该问题优先级高于性能实验，不能把现有判断当作可靠保护。

其他需补查事项包括：`setSlidingWindowDirection()` 设置方向后却将 `use_sliding_window_target_` 置为 `false`，不满足选择椭球分支的条件；采样时间可包含首次越出椭球的点；半轴、比例、起始时间与零持续时间缺少完整合法性检查。这些均未在本次文档整理中修改。

## 单机适配与可视化

`planExploreMotion()` 移除了无网格时的立即 `NO_GRID` 返回，启用后续前沿目标回退，并检查空前沿列表及目标索引。它可能改变探索选择和终止行为，应作为单独的工程因素记录，不能把由此产生的差异全部归给局部 ESDF。

`kinodynamicReplan()` 与 `planExploreTraj()` 在 `getValidTrajs()` 返回非空时才加入 `BsplineOptimizer::SWARM` 并传入邻机轨迹。优化器内原有的机间避碰代价实现没有更改。

新增两个绝对话题：`/sdf_map/local_window` 显示实际地图窗口，`/planning_vis/sliding_window_dir` 显示当前位置到下一视点的箭头。Marker 可见只证明发布路径执行；后者不是完整椭球显示。多机发布还可能覆盖相同 Marker 身份。

## 与开题报告逐项对应

| 报告研究项 | 当前证据 | 下一步 |
| --- | --- | --- |
| 局部 ESDF 范围策略 | 已有接口、参数、范围选择及中心更新 | 验证新进入区域、边界、参考距离与时序 |
| 方向调整 | 已有目标设置与椭球函数 | 修复状态初始化并接入实际研究链路；证明调用发生 |
| B 样条优化耦合 | 既有距离代价通过 ESDF 接口查询 | 记录查询有效性、规划耗时和完整样条约束 |
| G0 / G1 / G2 嵌套消融 | 两个参数存在，统一实验入口未完成 | 保留精确 G0，并控制单机回退等额外因素 |
| 等预算诊断 | 尚无正式实现与结果 | 区分几何采样范围、ESDF 处理工作量和存储预算 |
| 3 类场景 × 3 组 × 5 次 | 报告中的 45 次任务计划 | 尚未提供完整任务数据和统计结论 |
| 数据采集与覆盖评价 | 有原有 ROS 文本耗时日志 | 增加单调墙钟计时、结构化记录与独立覆盖核验 |
| 仿真平台 | 当前新增入口使用轻量指令跟随仿真 | 若最终使用 Gazebo，需实际接入并单独验证 |

## 本次发布整理的改动

本次完成中文 README、项目结构、代码差异、运行说明和验证说明；更正旧 README 中被误改为 VISTA 的 RACER 题名、BibTeX、作者归属和无效仓库链接；保留原 README 原文以便追溯。

启动地图改用 `$(find map_generator)`，两个 shell 入口不再固定 `/home/hh/VISTA` 或 `~/catkin_ws`，不再全局 `pkill`。历史生成的 667 个文件仅移出 Git 索引，未从用户磁盘删除；新增忽略规则和 LF 换行规则。上述整理不构成新的规划算法贡献。

本地整理前的完整文件与 Git 状态已在论文项目的工作目录备份。公开仓库不包含带学号的 Word 原件、原始交接文件备份、凭据或本次内部工作目录。

## 可以使用的项目表述

VISTA 基于 RACER，增加了跟随无人机位置的局部 ESDF 更新范围策略，以及目标方向引导的椭球轨迹采样原型，并补充单机运行适配与窗口可视化。它保留了原有探索前端、B 样条表示和主要优化目标。当前仍需完成方向机制在默认探索链路中的有效接入、地图查询正确性验证和系统消融实验，尚无经验证的定量性能提升结论。
