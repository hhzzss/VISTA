# 基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究：第二版新增模块核验

本页替代第二版原“全部完成/100%”叙述，以实际源码和默认调用为准。未经验证的性能百分比不作为当前项目结论。原文见 [source-v2/PHASE2_SUMMARY.md](../docs/source-v2/PHASE2_SUMMARY.md)，仅为归档。

| 模块 | 源码内容 | 构建与调用状态 |
| --- | --- | --- |
| `PointCloudCluster` | 0.1 m 体素降采样、欧氏聚类、中心距离合并、PCA 法向、距离加权信息量、图节点/边与可视化 | `active_perception/CMakeLists.txt` 未加入 `pointcloud_cluster.cpp`；默认 manager 未实例化 |
| `EOHDTSolver` | 拓扑节点关联视点、贪心顺序、2-opt、节点内视点排序与总成本 | `eohdt_solver.cpp` 已加入 exploration/ground 节点目标；未发现默认 manager 调用 |
| `PointCloudCollision` | KD-tree 最近邻、净空、点/线段/轨迹采样检测和避障方向 | 已加入 plan_manage 库及节点目标；未接入 B 样条距离代价 |

## 点云聚类/拓扑图

FPFH 头文件和 `extractFPFHFeatures()` 声明存在，但 `.cpp` 未提供该函数定义，现有 `extractClusters()` 只做降采样与欧氏聚类。`geometricConsistencyGrouping()` 实际按聚类中心距离合并，不是已验证的特征匹配。不能描述为完整 FPFH 特征拓扑方案已运行。

## EOHDT

源码组织了拓扑层与视点层，但当前实现主要是访问顺序启发式；不据类名认定已实现完整双树探索系统。边成本为距离、`1/(1+information_gain)` 与固定为零的协同项之和。原 ATSP/LKH 路径仍保留为默认探索流程。

新增 `eohdt_solver.h` 与既有 `active_perception/graph_node.h` 都在 `fast_planner` 中定义 `ViewNode`，合并进同一编译单元会产生重定义，跨编译单元同名类也存在 ODR 风险。后续接入应先消除类型冲突，再处理生命周期与数据接口。

## 点云碰撞

这是独立几何检查工具，不是已经替换 ESDF 的优化器。空点云时不报碰撞，净空返回最大检查距离，不能将其当作已经保证安全的未知空间策略。`init()` 只读取安全半径和最大检查距离，未读取原摘要列出的 `pointcloud_collision/enabled`；该开关不能通过添加同名参数自动生效。

## 原始辅助脚本

`apply_phase2.sh` 含批量 `cp`、`sed`、`eval` 和删除构建目录操作。本次只作来源审阅和 shell 语法检查，不执行。它不是使用第二版的必要步骤，不应为了“启用 Phase 2”直接运行。

新增源文件的语法检查结果见[核验记录](../docs/VALIDATION.md)。独立源文件可解析不等于库链接、默认流程接入或闭环性能验证完成。
