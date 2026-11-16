# 基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究：构建与运行

本说明针对第二版源码。源码同步完成不代表已经完成整工程编译或仿真验收；实际检查结果见[VALIDATION.md](VALIDATION.md)。

## 构建前置条件

使用 Ubuntu 20.04 / ROS Noetic、catkin、C++14 工具链，准备 ROS 消息组件、Eigen、PCL、OpenCV、Armadillo 等依赖。项目继承了外部 NLopt 与 LKH 依赖：部分 CMake 直接引用 `/usr/local/include`、`/usr/local/lib/libnlopt.so`，外部 LKH 预期位于 `/usr/local/bin/LKH`。需与本机安装位置保持一致，不能只安装了其他位置的软件包就认定路径满足。

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/hhzzss/VISTA.git
cd ..
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

第二版新加入 EOHDT 和点云碰撞源文件，完整构建还需核对这些文件与既有头文件/库的兼容性。聚类 `.cpp` 没有加入 CMake；本次没有为了消除差异而擅改第二版构建清单。

## 原有单机入口

第二版的 `single_drone_exploration.xml` 保留完整探索节点、轨迹服务与可选轻量仿真，但不包含地图发布。运行前需要可用地图节点和 `/map_generator/global_cloud` 输入。

分别在已加载工作空间的终端启动地图发布、RViz 和单机探索：

```bash
rosrun map_generator map_pub "$(rospack find map_generator)/resource/pillar.pcd"
roslaunch exploration_manager rviz.launch
roslaunch exploration_manager single_drone_exploration.xml drone_id:=1 drone_num:=1 simulation:=true
```

以上命令分别在三个终端运行，前提是已成功构建并核实资源文件存在。RViz 可使用 `2D Nav Goal` 触发探索；这只是原有交互方式，不代表本次完成了触发后的闭环测试。轻量模拟器使用 `poscmd_2_odom`，不能表述成已经完成 Gazebo 或实机验证。

## 第二版原始入口的限制

- `single_drone_exploration_with_map.xml` 只有 `<node>` 片段，没有 `<launch>` 根元素，也没有声明 `map_name`；不能直接作为完整 launch 文件。
- `test_single_drone.sh` 固定 `~/catkin_ws`、`/home/hh/VISTA`，并按名称杀死 ROS 进程。
- `test_with_map.sh` 使用全局 `pkill -9 -f "ros|gz"`，会影响其他 ROS/Gazebo 会话，并指向上述不完整 XML。
- `apply_phase2.sh` 会批量修改文件和删除构建目录，不是复现所需命令。

本次保留这些第二版原件，未执行它们。建议使用上面的 ROS 原生入口，并先修复实际运行环境与已知地图/轨迹状态问题。不能把第一版可移植脚本的测试通过记录套用到本版。

## 实验记录

用户反馈这一版效果更好；目前没有统一对照任务数据。后续记录具体提交、场景/地图哈希、参数、随机种子、任务超时、节点日志和结果。先验证窗口新进入区域、距离/梯度有效性与方向采样调用，再开展 G0/G1/G2 对照。除 ESDF 与规划耗时外，同时统计覆盖率、安全距离、轨迹约束、任务结局和资源占用。
