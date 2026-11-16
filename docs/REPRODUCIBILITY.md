# VISTA 运行与复现说明

本说明给出源码对应的构建和启动入口。当前版本仍有默认探索链路接入与状态初始化问题，完整情况见[改动核验](RACER_CHANGES.md)及[验证记录](VALIDATION.md)。构建成功、launch 参数可解析、RViz Marker 可见与任务完整成功是不同检查层次。

## 环境与依赖

原 RACER 使用 ROS 1 / catkin。原 README 列出 Ubuntu 18.04 / ROS Melodic 和 Ubuntu 20.04 / ROS Noetic；VISTA 本次核验选择已有 Ubuntu 20.04 / ROS Noetic 环境，没有重新验证 Melodic。

主要依赖从源码和 CMake 可见：ROS Noetic、Eigen3、PCL、OpenCV、Armadillo、NLopt 以及 LKH 相关求解组件。先配置 ROS Noetic，再安装系统依赖，例如：

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git python3-rosdep libarmadillo-dev
```

NLopt 按原工程约定使用 2.7.1。当前 `bspline_opt/CMakeLists.txt` 直接使用 `/usr/local/include` 和 `/usr/local/lib/libnlopt.so`，只安装其他路径的系统包不一定满足该构建约定：

```bash
# 在单独的依赖源码目录执行
git clone --branch v2.7.1 --depth 1 https://github.com/stevengj/nlopt.git
cmake -S nlopt -B nlopt/build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
cmake --build nlopt/build -j2
sudo cmake --install nlopt/build
sudo ldconfig
```

RACER 原说明推荐 LKH-3.0.6。仓库内含相关求解源码，但 `lkh_mtsp_solver/src2/mtsp_node.cpp` 仍有调用 `/usr/local/bin/LKH` 的分支，需同时检查外部可执行文件。可从原作者的 [LKH-3 页面](http://akira.ruc.dk/~keld/research/LKH-3/) 获取所需版本，构建后确认该路径可执行。不要把“内部 LKH 库已编译”当作所有外部求解入口均已满足。

`uav_simulator/local_sensing/CMakeLists.txt` 当前使用 `ENABLE_CUDA false`，默认走 CPU 点云渲染分支；无需为了这个默认入口先开启 CUDA。

## 工作空间布局与构建

```bash
mkdir -p ~/vista_ws/src
git clone https://github.com/hhzzss/VISTA.git ~/vista_ws/src/VISTA
source /opt/ros/noetic/setup.bash
cd ~/vista_ws
# 若机器从未初始化 rosdep，需要按 ROS 环境要求初始化并更新它
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j2
source devel/setup.bash
```

当前源目录有 25 个 catkin 有效包，项目根目录本身不是一个独立 ROS 包。不要在仓库每个子目录重复运行 CMake，也不要复用随上游遗留的嵌套 `build/` 缓存。本次发布从 Git 索引中排除了这些生成物，新检出的代码应在新工作空间构建。

## 启动前检查

```bash
source /opt/ros/noetic/setup.bash
source ~/vista_ws/devel/setup.bash
rospack find exploration_manager
rospack find map_generator
test -f /usr/local/lib/libnlopt.so
test -x /usr/local/bin/LKH
```

检查窗口与探索边界参数，默认值不是经正式实验标定后的推荐最优值。特别是 `single_drone_planner.xml` 中探索盒为 X `[-7, 7]`、Y `[-15, 15]`、Z `[0, 1.7]`，默认带地图入口的完整地图大小为 `35 × 35 × 3.5 m`。地图尺寸和评价覆盖分母不是同一个概念。

## 单机带地图入口

默认方向开关当前为 `true`。执行完整探索前，应先修复[状态初始化与链路接入问题](RACER_CHANGES.md#默认链路与方向机制的接入缺口)；若仅诊断局部 ESDF，可先在 planner XML 中关闭方向开关并核验实际加载值。这不等于完成了 G2，也不能绕过地图查询正确性检查。

```bash
roslaunch exploration_manager single_drone_exploration_with_map.xml map_name:=pillar.pcd
```

该入口依次包含场景点云发布、`single_drone_exploration.xml` 和 RViz；单机配置再包含 planner、轨迹服务器及 `simulator_light.xml`。后者使用 `poscmd_2_odom` 与感知渲染，不是 Gazebo 物理仿真。

按场景实际边界调整参数，例如：

```bash
roslaunch exploration_manager single_drone_exploration_with_map.xml \
  map_name:=office.pcd init_x:=0 init_y:=0 init_z:=1 \
  map_size_x:=35 map_size_y:=35 map_size_z:=3.5
```

这只是参数写法示例，未证明该起点与边界适用于所有办公室地图。可选资源位于 [map_generator/resource](../uav_simulator/map_generator/resource)。用 RViz 的 `2D Nav Goal` 触发探索；停止时使用当前终端的 Ctrl+C。

两个兼容脚本都启动同一带地图入口，不自行 source 固定工作空间、不另起 roscore、不全局杀死其他 ROS / Gazebo 进程：

```bash
bash ~/vista_ws/src/VISTA/test_single_drone.sh map_name:=pillar.pcd
# 或
bash ~/vista_ws/src/VISTA/test_with_map.sh map_name:=office.pcd
```

ROS `roslaunch` 会处理本次启动树内节点的生命周期。脚本不替代环境 source，也不是自动化性能测试。

## 原多机入口

继承的 `swarm_exploration.launch` 仍在仓库中，另一个终端可运行 `rviz.launch`。但它声明 `drone_num=5`，当前未被注释的无人机 include 为 1 至 4，且新增两个可视化话题没有按无人机区分。它不是本课题已经完成验证的多机配置，正式研究优先使用单机入口。

## 参数与诊断

完整参数说明见[配置速查](../swarm_exploration/config_changes.md)。窗口开关当前写在 planner 节点内部，并未暴露为上层 launch 参数。需要制作相互独立的配置文件并检查实际加载的参数，不能假设传入一个未声明的命令行参数就已切换算法。

```bash
rosparam get /exploration_node_1/sdf_map/use_local_window
rosparam get /exploration_node_1/manager/use_dynamic_sliding_window
rostopic info /sdf_map/local_window
rostopic info /planning_vis/sliding_window_dir
```

可先用 `roslaunch --dump-params exploration_manager single_drone_exploration_with_map.xml` 检查包含关系和参数展开，而不启动节点。Marker 和参数均正常也不意味着椭球采样已进入默认探索路径。

## 如何建立有效对照

精确原版可在独立目录从固定基线提取，不在当前工作树上强制重置：

```bash
git clone https://github.com/hhzzss/RACER.git ~/racer-baseline
git -C ~/racer-baseline checkout 049c332e3634ef72d8beb155b4c13dc91ca52916
# 放入与 VISTA 分离的 catkin 工作空间构建，避免同名 ROS 包混用
```

后续配置目标为：G0 精确基线；G1 只改变 ESDF 范围策略；G2 在 G1 基础上启用经验证已生效的方向机制。当前单机回退、SWARM 开关、地图外返回值也有变化，若保留这些兼容性修订，应对各对照组一致应用并明确记录“共同修订基线”。

开题报告计划为三类场景、三配置、每组五次配对任务，共 45 次正式闭环任务。当前没有这些数据。实施前至少补齐：

- 有效距离查询、参考 ESDF 和完整样条的安全与动力学核验。
- 每次调用的单调墙钟计时、更新体素数、查询有效性、优化状态和运行元数据。
- 独立覆盖率分母及 T90 / T95、L90 / L95 的离线复算。
- 参数与提交哈希、机器信息、相同初始条件、停止规则与失败任务记录。
- 明确 ROS 仿真时间和真实墙钟时间；若更换为 Gazebo，重新核验平台与时间语义。

所有实验结论应来自归档数据。旧文档中的性能数字已从结果性叙述中撤下，不能直接用于论文实验表。
