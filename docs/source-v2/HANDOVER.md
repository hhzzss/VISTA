# VISTA 项目交接文档

## 项目概述

**VISTA** = Vision-based Strategic Exploration  
基于RACER改进的多无人机协同探索框架，采用局部ESDF窗口+动态滑动窗口架构。

## 架构对比

| 组件 | 原RACER | VISTA (当前) | 未来Phase 2 |
|------|---------|--------------|-------------|
| 地图 | 全局ESDF (35×35m) | **局部ESDF窗口** (10×10m跟随) | 局部窗口 + 稀疏全局 |
| 全局规划 | ATSP求解器 | ATSP求解器 | **EOHDT求解器** |
| 局部规划 | 圆形滑动窗口 | **椭圆滑动窗口** (指向目标) | 点云聚类 + 自适应拓扑 |
| 避障 | ESDF依赖 | ESDF依赖 | 混合ESDF-free |

## Phase 1 已完成（本次）

### 1. 局部ESDF窗口化

**核心思想**: 以无人机位置为中心维护固定大小局部ESDF窗口，滚动更新。

**修改文件**:
- `plan_env/include/plan_env/sdf_map.h` (第68-75行): 添加窗口接口
- `plan_env/src/sdf_map.cpp` (已添加): 窗口更新、范围计算
- `plan_env/src/map_ros.cpp`: 添加绿色窗口可视化

**关键函数**:
```cpp
void updateLocalWindowCenter(const Eigen::Vector3d& center);  // 更新窗口中心
bool isInLocalWindow(const Eigen::Vector3d& pos);             // 判断是否在窗口内
void publishLocalWindow();                                     // RViz可视化
```

**ROS参数** (已添加到launch文件):
```xml
<param name="sdf_map/use_local_window" value="true"/>
<param name="sdf_map/local_window_x" value="10.0"/>
<param name="sdf_map/local_window_y" value="10.0"/>
<param name="sdf_map/local_window_z" value="5.0"/>
```

### 2. 动态滑动窗口

**核心思想**: 滑动窗口从圆形改为椭圆形，长轴指向拓扑目标方向。

**修改文件**:
- `plan_manage/include/plan_manage/plan_container.hpp` (第114-171行): `getTrajInfoInEllipsoid()`
- `plan_manage/include/plan_manage/planner_manager.h` (第46-50行): 接口声明
- `plan_manage/src/planner_manager.cpp` (已添加): 方向计算和椭圆参数
- `exploration_manager/src/fast_exploration_manager.cpp` (第292-320行): 可视化发布

**关键函数**:
```cpp
void setSlidingWindowTarget(const Eigen::Vector3d& target_pos);  // 设置目标
void getTrajInfoInEllipsoid(..., const Eigen::Vector3d& direction, ...); // 椭圆采样
```

**ROS参数** (已添加到launch文件):
```xml
<param name="manager/use_dynamic_sliding_window" value="true"/>
<param name="manager/sliding_window_minor_ratio" value="0.6"/>
```

### 3. 可视化 (新增)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/sdf_map/local_window` | Marker (Cube) | 绿色半透明，显示局部ESDF范围 |
| `/planning_vis/sliding_window_dir` | Marker (Arrow) | 橙色箭头，指向当前目标 |

## Phase 2 建议 (下一步)

### 优先级1: 点云聚类与拓扑图构建

**目标**: 用点云聚类替代/补充边界点(frontier)，构建自适应拓扑图。

**实现位置**:
- 新建: `active_perception/pointcloud_cluster.h/cpp`
- 修改: `active_perception/frontier_finder.cpp` 集成聚类
- 修改: `plan_manage/plan_container.hpp` 添加拓扑图数据结构

**思路**:
1. 从局部窗口点云提取FPFH特征
2. 欧氏聚类+几何一致性分组
3. 构建稀疏拓扑节点(每个聚类一个节点)
4. 边权重 = 路径成本 + 信息增益衰减

### 优先级2: EOHDT求解器

**目标**: 替代ATSP，求解分层双树结构。

**实现位置**:
- 新建: `exploration_manager/src/eohdt_solver.cpp`
- 修改: `exploration_manager/src/fast_exploration_manager.cpp` 替换findGridAndFrontierPath()

**思路**:
1. 上层树: 拓扑节点之间的转移(TSP-like)
2. 下层树: 每个节点内的视点优化(类似于现有的refineLocalTour)
3. 能量函数: 距离 + 不确定性 + 协同成本

### 优先级3: ESDF-free局部避障

**目标**: 在局部规划层减少对ESDF的依赖。

**实现位置**:
- 修改: `plan_manage/src/planner_manager.cpp` 在碰撞检测中加入点云直接查询
- 修改: `bspline_opt/bspline_optimizer.cpp` 添加基于点云的梯度计算

## 关键文件清单

```
VISTA/
├── swarm_exploration/
│   ├── plan_env/
│   │   ├── include/plan_env/sdf_map.h          [局部窗口接口]
│   │   ├── src/sdf_map.cpp                      [局部窗口实现]
│   │   ├── include/plan_env/map_ros.h           [可视化接口]
│   │   └── src/map_ros.cpp                      [窗口可视化]
│   ├── plan_manage/
│   │   ├── include/plan_manage/plan_container.hpp [椭圆滑动窗口]
│   │   ├── include/plan_manage/planner_manager.h  [接口]
│   │   └── src/planner_manager.cpp                [实现]
│   ├── exploration_manager/
│   │   ├── src/fast_exploration_manager.cpp     [集成+可视化]
│   │   └── launch/single_drone_planner.xml      [参数配置]
│   └── config_changes.md                        [配置说明]
└── HANDOVER.md                                  [本文档]
```

## 验证方法

1. **编译**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

2. **运行**:
   ```bash
   roslaunch exploration_manager swarm_exploration.launch
   roslaunch exploration_manager rviz.launch
   ```

3. **验证功能**:
   - RViz中能看到绿色局部窗口跟随无人机
   - 橙色箭头指向当前探索目标
   - 日志显示ESDF更新时间约5ms (原20ms)

## 性能预期

| 指标 | 原RACER | VISTA Phase 1 | 预期Phase 2 |
|------|---------|---------------|-------------|
| ESDF内存 | ~50MB | ~3MB (10×10m) | ~3MB |
| ESDF更新 | ~20ms | ~5ms | ~5ms |
| 重规划次数 | baseline | -20%~30% | -40%~50% |
| 探索时间 | baseline | -15%~25% | -30%~40% |

---

**作者**: Claude Code  
**日期**: 2026-04-14  
**状态**: Phase 1完成，准备Phase 2
