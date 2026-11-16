# 基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究：项目结构

本页面向 2026-11-16 同步的第二版 `E:/VISTA`，不沿用第一版功能清单。

```text
VISTA/
├── swarm_exploration/           地图、探索、规划与求解器
│   ├── plan_env/               占据地图、ESDF、局部更新窗口及查询
│   ├── plan_manage/            规划管理、全局/局部轨迹与点云碰撞模块
│   ├── bspline/                B 样条表示、求导与参数化
│   ├── bspline_opt/            平滑、避障、动力学、时间及机间代价
│   ├── path_searching/         A*、动力学搜索及拓扑引导
│   ├── active_perception/     前沿、视点、网格、图结构及新增聚类模块
│   ├── exploration_manager/   探索状态机、目标选择、launch/RViz、EOHDT
│   ├── poly_traj/              多项式轨迹支持
│   ├── traj_utils/             轨迹消息与辅助功能
│   ├── utils/                  LKH、多机通信与辅助工具
│   ├── bag/                    原有录制脚本
│   ├── config_changes.md      本版本配置核验
│   ├── PHASE2_SUMMARY.md      新增模块真实状态
│   └── apply_phase2.sh        原始批量修改/构建脚本，未执行
├── uav_simulator/              感知、地图发布、仿真、控制及可视化
├── files/                      上游媒体资源
├── tmp/test_node_fixed.cpp     第二版附带的未集成试验示例
├── test_single_drone.sh        第二版原始脚本，含固定路径和 pkill
├── test_with_map.sh            第二版原始脚本，含全局 pkill
├── docs/                       当前解释文档、同步清单与来源记录
├── HANDOVER.md                 交接入口
└── THIRD_PARTY.md              来源与许可说明
```

目录树为职责导航，完整已同步文件见 [SOURCE_MANIFEST.sha256](SOURCE_MANIFEST.sha256)。临时示例虽然保留，但不进入默认构建：它引用本仓库未提供的 `trajectory_generator/kino_traj_generator.h`，不能视为可运行测试。

## 默认探索与研究接口

```text
传感器/里程计 → MapROS → SDFMap 占据融合与 ESDF
                     ↓ 距离/梯度查询
前沿与视点 → FastExplorationManager → FastPlannerManager
                                      ↓
                             B 样条优化 → 轨迹服务
```

`FastExplorationManager::planExploreMotion()` 更新地图窗口中心，并向规划管理器传入下一视点作为方向目标。椭球采样位于另一条全局/拓扑轨迹参数化分支；新 Phase 2 类也没有在默认 manager 中实例化。因此不能只根据参数存在或 Marker 显示判断所有算法已执行。

## 建议阅读顺序

1. [版本替换说明](VERSION_2.md)与[核验记录](VALIDATION.md)：先确定当前基准和可运行边界。
2. `exploration_manager/src/fast_exploration_manager.cpp`：探索主流程和窗口中心更新。
3. `plan_env/src/sdf_map.cpp`、`plan_env/include/plan_env/sdf_map.h`：局部更新与查询。
4. `plan_manage/src/planner_manager.cpp`、`plan_manage/include/plan_manage/plan_container.hpp`：目标方向、轨迹截取和样条参数化。
5. `bspline_opt/src/bspline_optimizer.cpp`：继承的优化代价。
6. 三组新增模块：`pointcloud_cluster.*`、`eohdt_solver.*`、`pointcloud_collision.*`，结合[状态说明](../swarm_exploration/PHASE2_SUMMARY.md)阅读。
