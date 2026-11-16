# 基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究：相对 RACER 的改动

核对对象为第二版 `E:/VISTA` 的实际工作树文件，而非仅比较其 HEAD 或暂存区。RACER 基线为 `049c332e3634ef72d8beb155b4c13dc91ca52916`，其历史保留在本仓库中。本次完整替换实现树，没有将第一版研究代码合入第二版。

## 基础改造进展

局部 ESDF 窗口化与动态滑动窗口方向对齐两项基础改造已完成，已有 RViz 运行效果展示。主要实现分别位于 `sdf_map.cpp` 与 `planner_manager.cpp`，详见[项目进展与运行效果](PROGRESS.md)。基础实现完成与各规划分支的调用覆盖、地图查询正确性及定量性能验证分别记录。

## 变更与接入状态

| 模块 | RACER 基线 | 第二版的具体变化 | 当前边界 |
| --- | --- | --- | --- |
| ESDF 范围 | 观测更新包围盒 `local_bound_*` | 可改用固定尺寸、随位置移动的 `local_window_*` | 原版已是局部更新；不是首次实现局部 ESDF |
| 窗口中心 | 随融合更新包围盒变化 | `planExploreMotion()` 调用 `updateLocalWindowCenter(pos)` | 在探索规划时更新，非每帧里程计更新 |
| 地图存储 | 全地图体素缓冲区 | 保留原分配规模 | 不是环形滚动存储，不据此声称减少内存 |
| 距离查询 | 地图外距离/梯度为零 | 地图外或窗口外返回默认距离和零梯度 | 仍缺有效性、时效与边界邻点的统一约束 |
| 轨迹截取 | 球形范围 | 增加目标方向椭球截取与球形回退 | 基础接口已完成；不同规划分支中的实际调用覆盖仍需跟踪验证 |
| 方向状态 | 无新增目标接口 | `setSlidingWindowTarget()` 等 | 当前直接查询全局轨迹状态，初始化前调用有风险 |
| 可视化 | 原有地图/轨迹 | 局部窗口 Cube、目标 Arrow、RViz 配置 | 方向箭头不等于椭球采样生效 |
| 容器与名字 | 多处裸 `vector`、变长栈数组 | 部分改为 `std::vector` | 工程兼容性修改，不是新求解算法 |
| 点云聚类 | 无这些新增文件 | 欧氏聚类、中心合并、PCA 法向、拓扑图 | 尚未纳入 active_perception 构建或默认调用 |
| EOHDT | 原 ATSP/LKH 路线 | 分层拓扑/视点排序原型 | 源文件加入目标，但没有替换默认 ATSP |
| 点云碰撞 | 主要依赖原 ESDF 查询 | KD-tree 最近邻、线段采样、轨迹检查与方向 | 源文件加入目标，但尚未进入 B 样条距离代价 |

## 局部 ESDF 与 B 样条接口

核心文件为 [sdf_map.cpp](../swarm_exploration/plan_env/src/sdf_map.cpp)、[sdf_map.h](../swarm_exploration/plan_env/include/plan_env/sdf_map.h)、[map_ros.cpp](../swarm_exploration/plan_env/src/map_ros.cpp)。

窗口边界为 `pos ± window_size/2` 与完整地图边界的交集；当前单机配置尺寸为 `10 × 10 × 5 m`。`updateESDF3d()` 在启用开关时使用窗口索引，否则使用原观测更新范围。初始化窗口仍覆盖完整地图，第一次规划更新后才收缩。

原有距离代价链路继续为：`BsplineOptimizer::calcDistanceCost()` → `EDTEnvironment::evaluateEDTWithGrad()` → `SDFMap::getDistWithGrad()`。本版没有重写 B 样条表示、主要优化目标或 NLopt 求解器；变化主要在地图查询语义及轨迹截取原型。

窗口外 `getDistWithGrad()` 返回 `default_dist` 和零梯度，而 `getDistance()` 仍可读取完整地图。窗口内查询的插值邻点也可能越出已更新范围。窗口切换与 ESDF 定时更新之间没有同步完成保证，距离变换的窗口外障碍传播保护带也未明确处理。这些事项需要同输入参考距离场、窗口新进入区域和闭环碰撞检查，不能只比较更新时间。

## 方向引导椭球采样

实现位于 [plan_container.hpp](../swarm_exploration/plan_manage/include/plan_manage/plan_container.hpp) 的 `getTrajInfoInEllipsoid()`；调用选择位于 [planner_manager.cpp](../swarm_exploration/plan_manage/src/planner_manager.cpp) 的 `paramLocalTraj()`。

以采样起点为中心，将局部 Z 轴对齐目标方向，使用 `(qx²+qy²)/b² + qz²/a² ≤ 1` 判断轨迹点；`a=local_segment_length`，`b=a×minor_ratio`。当前配置对应半轴 6 m 与 3.6 m。该椭球用于轨迹截取，不旋转 ESDF 长方体，也不扩展传感器感知范围。

```text
默认探索：planExploreMotion → setSlidingWindowTarget → planTrajToView
                                   → planExploreTraj / kinodynamicReplan
椭球分支：planGlobalTraj / topoReplan → paramLocalTraj → getTrajInfoInEllipsoid
```

默认 `manager/use_topo_path=false`；默认探索两个位置轨迹生成函数不调用 `paramLocalTraj()`。第二版 `setSlidingWindowTarget()` 没有第一版的时长判断，直接从 `global_data_.getState()` 读取状态。全局轨迹是否已初始化需要核验，不能把目标设置成功视为采样接入成功。`setSlidingWindowDirection()` 还会将目标使能标记置为 false，与椭球分支条件不一致。

## 第二版新增模块

详细核验见 [PHASE2_SUMMARY.md](../swarm_exploration/PHASE2_SUMMARY.md)。特别需要区分：聚类代码声明了 FPFH 接口，但没有其定义，现有聚类路径也不调用 FPFH；EOHDT 协同成本当前为常数零；点云碰撞未连接既有 B 样条距离代价。三个类存在不等于形成完整新规划链路。

## 与开题报告的关系

开题报告关注单无人机局部窗口与方向机制，通过 G0/G1/G2 消融、地图接口测试、三类场景各配置五次的 45 次正式任务，验证耗时、覆盖、安全和资源变化。上述内容是研究安排，当前版本没有提供完成全部实验的记录。新增 Phase 2 模块应作为扩展原型单独说明，不能取代主课题的正确性与消融验证。

可使用的当前表述：本项目在 RACER 基础上增加局部 ESDF 更新范围策略、方向引导椭球轨迹截取原型及窗口可视化，并在第二版补充聚类、分层排序和点云碰撞检查源文件。用户反馈第二版效果较好；具体收益、归因和适用范围有待统一配置下的实验确认。
