# 来源与第三方内容说明

## RACER 来源

本仓库以 RACER 提交 `049c332e3634ef72d8beb155b4c13dc91ca52916` 为基线，保留原 Git 祖先历史。

- 用户提供的基线仓库：<https://github.com/hhzzss/RACER>
- 原作者上游仓库：<https://github.com/Robotics-STAR-Lab/RACER>
- 原论文：Boyu Zhou, Hao Xu, Shaojie Shen. RACER: Rapid Collaborative Exploration With a Decentralized Multi-UAV System. IEEE Transactions on Robotics, 2023, 39(3): 1816–1835.
- 原论文页面：<https://arxiv.org/abs/2209.08533>

原 README 原文见 [docs/upstream/README.RACER.md](docs/upstream/README.RACER.md)。其中相对链接与示例保留原样，应在原仓库上下文中阅读。`files/racer1.gif`、`racer2.gif`、`racer3.gif` 等演示资源来自原仓库，不作为 VISTA 新增实验结果。RACER 的作者、论文及获奖信息不归属于 VISTA。

## 继承的组件

探索前端、B 样条基础实现、主要优化代价、A* 与动力学搜索、拓扑规划、地图融合基础框架及大部分仿真组件均继承自上游。仓库还含 LKH 相关求解实现、ODE/odeint 等第三方源码及相关资源；外部运行依赖包括 ROS、Eigen、PCL、OpenCV、NLopt、Armadillo 等。

## 许可标记

本次核验的 RACER 根目录没有独立 `LICENSE` 文件；各 `package.xml` 和源码头部存在各自的许可或版权标记。本次保留这些标记，没有将整个仓库重新声明为 MIT、BSD 或其他统一许可，也没有改变第三方源码的许可。新增说明文档不替代原有声明。

具体新增研究代码与上游继承部分的边界见 [docs/RACER_CHANGES.md](docs/RACER_CHANGES.md)。
