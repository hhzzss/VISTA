# VISTA 本次核验记录

日期：2026-11-16。检查对象是本地 VISTA 完整工作树相对 RACER 固定提交的差异，以及本次发布整理后的文档与启动入口。以下检查不等于完成探索任务或性能实验。

## 已通过的检查

| 检查 | 实际结果 |
| --- | --- |
| 基线身份 | 本地基线、`hhzzss/RACER` 与 `Robotics-STAR-Lab/RACER` 的 `main` 均核对为 `049c332e3634ef72d8beb155b4c13dc91ca52916` |
| 目录与 ROS 包扫描 | `catkin_pkg.packages.find_packages()` 检出 25 个有效包；解释了 26 份包清单中的一份历史嵌套副本 |
| 原有算法源码保留 | 与整理前完整备份比较，639 个非历史 build 目录中的 C/C++ 源文件与头文件字节内容未改变 |
| 发布目录整理 | 667 个历史 build / bin / lib 生成文件移出 Git 索引，原文件仍在用户磁盘；未重写上游历史 |
| XML 基本结构 | `exploration_manager/launch` 下 16 个 XML / launch 文件可解析 |
| 新单机入口的 ROS 解析 | ROS Noetic `roslaunch --dump-params` 成功展开 207 个参数；地图路径可通过 ROS 包定位 |
| 实际参数核对 | 局部窗口 `true`、尺寸 `10/10/5`、方向开关 `true`、比例 `0.6`、拓扑开关 `false` 与源码说明一致 |
| 启动脚本语法 | 两个脚本均通过 `bash -n` |
| 启动脚本行为 | 使用模拟 `roslaunch` 检查参数逐项转发，包括带空格参数；检查退出码 0 和 7 原样返回；缺少 roslaunch 时明确退出 |
| 公开内容基础检查 | 候选文件中未命中 GitHub token、AWS access key、私钥头等所检查的常见凭据模式；没有超过 100 MB 的单个文件；该模式检查不替代所有形式的内容审查 |

脚本行为检查没有启动实际 ROS 节点，也没有使用全局杀进程命令。正式 Word 开题报告只作为研究范围的背景资料读取，未修改或加入公开仓库。

## 构建检查环境

使用已有 WSL `Ubuntu2004-E` 中的 Ubuntu 20.04 / ROS Noetic、GCC 和 CMake。在本地论文项目的独立工作目录创建源码副本和新的 catkin 工作空间，使用 `catkin_make -j2 -l2` 检查；没有在原项目中复用历史 CMake 缓存，也没有为了本次整理安装或升级系统依赖。

CMake 配置与生成成功；`plan_env`、`bspline`、`path_searching`、`active_perception`、内部 LKH 求解库及若干仿真组件已实际编译链接通过，包含当前修改过的 ESDF 地图模块。编译过程中也出现继承代码的警告，未在本次整理中逐项修复。

**完整工程尚未验证通过。** 环境检查确认缺少工程约定路径的 `/usr/local/include/nlopt.hpp`、`/usr/local/lib/libnlopt.so`，以及部分求解分支需要的 `/usr/local/bin/LKH`。内部 LKH 库能编译不表示该外部可执行文件存在；CMake 能生成也不表示硬编码依赖的头文件和库都已满足。应先按[运行说明](REPRODUCIBILITY.md)补齐依赖，再重新构建完整工程。

实际构建已在 `bspline_opt` 目标报告缺少依赖库：

```text
No rule to make target '/usr/local/lib/libnlopt.so', needed by '.../libbspline_opt.so'. Stop.
```

因此本次完整构建结论为失败，不能报告为全工程编译通过；后续依赖该优化库的规划管理与探索可执行目标也未完成本次完整构建验证。这一环境依赖缺失与源码静态核验发现的方向接入问题分别记录。

## 静态核验确认的问题

1. 目标方向接口读取默认构造时未初始化的 `global_duration_`，并依赖默认探索链路没有准备的全局轨迹状态。
2. 椭球采样只在 `paramLocalTraj()` 分支，默认探索的两个位置轨迹生成函数不调用它。参数为 true 或箭头可见不能证明机制已生效。
3. ESDF 更新范围改变但缓冲区分配仍按完整地图；原版已使用局部更新包围盒，因此原文档的内存和固定倍数加速结论缺少依据。
4. 窗口外默认距离、不同查询接口、边缘插值、新进入区域和更新时序仍需正确性验证。

代码证据、其他边界与报告对应关系见 [RACER_CHANGES](RACER_CHANGES.md)。这些问题在本次发布中按原算法状态保留，没有把未经验证的算法修补混入已完成改进的描述。

## 本次未完成的验证

- 全流程 RViz 交互、闭环探索完成、完整样条安全和动力学约束检查。
- 在 Gazebo 中运行并验证与当前轻量仿真的差异。
- 单机与多机全部运行分支；ROS Melodic 环境。
- G0 / G1 / G2 三组有效接入及共同因素控制。
- 45 次正式任务、覆盖率离线复算、计时与资源结构化采集。
- 任何内存、ESDF 耗时、重规划次数或探索时间的定量改善结论。

## 复查方法

从本仓库基线到当前版本的可复查差异：

```bash
git diff --stat 049c332e3634ef72d8beb155b4c13dc91ca52916..HEAD
git diff 049c332e3634ef72d8beb155b4c13dc91ca52916..HEAD -- swarm_exploration/plan_env
git diff 049c332e3634ef72d8beb155b4c13dc91ca52916..HEAD -- swarm_exploration/plan_manage
git diff 049c332e3634ef72d8beb155b4c13dc91ca52916..HEAD -- swarm_exploration/exploration_manager
```

完整原目录备份、原暂存区与工作树补丁、源码比对记录、launch 展开结果、脚本检查记录和构建日志保存在本地论文项目 `.document-work/vista-publication-20261116/`，不上传这些内部工作文件。
