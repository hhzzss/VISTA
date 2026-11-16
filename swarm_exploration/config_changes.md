# 基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究：配置说明

以下对应第二版 `exploration_manager/launch/single_drone_planner.xml` 和实际 C++ 参数读取。

| 参数 | C++ 缺省值 | 当前单机 XML | 含义 |
| --- | --- | --- | --- |
| `sdf_map/use_local_window` | false | true | 使用新增 ESDF 更新范围 |
| `sdf_map/local_window_x` | 10.0 | 10.0 | 完整窗口 X 尺寸，米 |
| `sdf_map/local_window_y` | 10.0 | 10.0 | 完整窗口 Y 尺寸，米 |
| `sdf_map/local_window_z` | 5.0 | 5.0 | 完整窗口 Z 尺寸，米 |
| `sdf_map/default_dist` | 5.0 | 0.5 | 默认距离值，不代表真实净空 |
| `manager/use_dynamic_sliding_window` | false | true | 允许方向椭球采样分支 |
| `manager/sliding_window_minor_ratio` | 0.6 | 0.6 | 短半轴/长半轴 |
| `manager/local_segment_length` | 见规划参数读取 | 6.0 | 椭球分支长半轴，米 |
| `manager/use_topo_path` | 见规划参数读取 | false | 原拓扑规划路径开关 |

这些参数是探索节点的私有参数，不是同名 launch `<arg>`。调整时编辑该节点参数或通过正确节点命名空间加载；不能未经声明直接在 roslaunch 命令后加同名参数。

## 窗口与可视化

地图窗口是轴对齐长方体，中心在 `planExploreMotion()` 开头更新，地图边缘会裁剪尺寸。它没有按目标旋转，也未替换完整地图缓冲区。

- `/sdf_map/local_window`：窗口 Cube。
- `/planning_vis/sliding_window_dir`：当前位置指向下一视点的 Arrow。

当前发布的方向 Marker 不是椭球实体或采样点云。只有进入 `paramLocalTraj()` 中的椭球分支才能认为该采样函数被调用；默认探索调用路径的缺口见[改动说明](../docs/RACER_CHANGES.md)。

## 新模块参数

`PointCloudCluster::init()` 读取 `pointcloud_cluster/cluster_tolerance=0.5`、`min_cluster_size=50`、`max_cluster_size=10000`、`feature_radius=0.5`、`merge_threshold=1.0`。读取参数不代表它们全部影响现有处理链路；例如 FPFH 提取尚无实现。

`EOHDTSolver::init()` 读取 `eohdt/alpha=1.0`、`beta=0.5`、`gamma=0.3`、`max_edge_length=10.0`、`max_iterations=100`。这些类尚无默认 manager 调用，不能认为加入参数即可替换 ATSP。

`PointCloudCollision::init()` 读取 `pointcloud_collision/safety_radius=0.3`、`max_check_distance=5.0`，没有读取 `enabled`。它也尚未接入默认距离代价。

## 消融配置边界

关闭两个新增开关只能用作诊断，不能视为精确 RACER 基线：窗口外/地图外查询语义、容器兼容改动等仍在。正式 G0 应从保留的 RACER 提交构建，G1/G2 的差异需单独冻结并记录。当前未提供一键完成全部消融和 45 次正式实验的脚本。
