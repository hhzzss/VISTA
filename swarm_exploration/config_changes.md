# VISTA 窗口配置说明

以下值来自当前源码和 `exploration_manager/launch/single_drone_planner.xml`，核对日期为 2026-11-16。完整实现边界与问题见[相对 RACER 的改动](../docs/RACER_CHANGES.md)。

## 参数与实际默认值

| 参数 | C++ 读取默认值 | 当前单机 planner 配置 | 作用 |
| --- | --- | --- | --- |
| `sdf_map/use_local_window` | `false` | `true` | 切换 ESDF 更新范围与梯度查询窗口判断 |
| `sdf_map/local_window_x` | `10.0` | `10.0` | X 全尺寸，米 |
| `sdf_map/local_window_y` | `10.0` | `10.0` | Y 全尺寸，米 |
| `sdf_map/local_window_z` | `5.0` | `5.0` | Z 全尺寸，米；会裁剪到地图边界 |
| `manager/use_dynamic_sliding_window` | `false` | `true` | 允许 `paramLocalTraj()` 选择椭球采样 |
| `manager/sliding_window_minor_ratio` | `0.6` | `0.6` | 短半轴 / 长半轴 |
| `manager/local_segment_length` | `-1.0` | `6.0` | 局部轨迹采样长半轴来源，米 |
| `sdf_map/default_dist` | `5.0` | `0.5` | 窗口外或地图外梯度查询返回值，米 |

前四项对应地图更新范围，后两项方向机制参数对应轨迹采样范围。两者不是同一个几何窗口。默认椭球半轴为 `6.0 / 3.6 / 3.6 m`；ESDF 窗口半尺寸为 `5.0 / 5.0 / 2.5 m`，且 Z 范围会进一步裁剪。二者没有包含关系保证。

## 修改位置

参数定义在[单机 planner XML](exploration_manager/launch/single_drone_planner.xml)，位于 `exploration_node_$(arg drone_id)` 节点内，其实际参数名带节点私有命名空间，例如 `/exploration_node_1/sdf_map/use_local_window`。

当前上层 launch 没有把这些参数暴露为命令行 `arg`；不能直接在 `roslaunch ...` 后附加 `use_local_window:=false` 并假定生效。短期可复制 planner 配置及其引用关系后修改，正式实验应补充可追溯的配置入口，并使用 `rosparam get` 核验实际值：

```bash
rosparam get /exploration_node_1/sdf_map/use_local_window
rosparam get /exploration_node_1/sdf_map/local_window_x
rosparam get /exploration_node_1/manager/use_dynamic_sliding_window
```

关闭两个开关可用于局部行为诊断，但当前代码还包含边界返回值和单机回退等改动，因此不等于精确原版 RACER 基线。

## 可视化

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/sdf_map/local_window` | `visualization_msgs/Marker` Cube | 实际裁剪后的轴对齐更新窗口 |
| `/planning_vis/sliding_window_dir` | `visualization_msgs/Marker` Arrow | 当前位置指向下一视点的连线 |

箭头可见不能证明默认探索任务使用了椭球采样。当前没有新增显示完整椭球边界或采样点集的发布器。上述两个话题为绝对路径，多机同时发布时还需处理命名空间和 Marker 身份冲突。

## 已知边界

局部更新不改变全图缓冲区分配规模；原 RACER 已有局部包围盒更新。窗口仅在 `planExploreMotion()` 更新中心，ESDF 仍由原 0.05 秒定时器在需要更新时计算。方向分支的默认链路接入及未初始化全局轨迹状态需先修复。当前没有经过验证的性能提升百分比。
