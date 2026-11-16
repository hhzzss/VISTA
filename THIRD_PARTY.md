# 基于局部 ESDF 滑动窗口的 B 样条轨迹优化及无人机自主搜索研究：来源与第三方说明

## RACER 来源

本仓库保留 RACER 提交 `049c332e3634ef72d8beb155b4c13dc91ca52916` 的祖先历史。用户提供的基线为 [hhzzss/RACER](https://github.com/hhzzss/RACER)，原作者仓库为 [Robotics-STAR-Lab/RACER](https://github.com/Robotics-STAR-Lab/RACER)。原 README 见 [docs/upstream/README.RACER.md](docs/upstream/README.RACER.md)。

上游论文是 Boyu Zhou、Hao Xu、Shaojie Shen 的 *RACER: Rapid Collaborative Exploration With a Decentralized Multi-UAV System*。原作者、论文与获奖信息属于上游，不因本课题或仓库使用 VISTA 技术标识而改变。上游演示 GIF/视频不作为第二版性能实验。

## 继承组件

探索前端、B 样条基础实现、主要优化代价、A* 与动力学搜索、拓扑规划、占据融合、通信与大部分仿真组件来自原工程。部分子目录保留 Fast-Planner 原始 README、LKH 代码及 ODE/odeint 相关内容；应按各自来源阅读。项目更名不改变这些组件的名称和作者归属。

第二版原 README 曾将上游 RACER 论文误改为 VISTA 名称。该内容仅在 [docs/source-v2](docs/source-v2/README.md) 作为原始说明归档，不能按其错误题名引用文献。当前介绍以本页和保留的上游 README 为准。

## 许可

当前基线根目录未见独立 LICENSE 文件，包和源码存在各自版权/许可标记，本次保留。未将整个项目重新声明为 MIT、BSD 等统一许可，也未更改第三方许可。新增解释文档不代替这些标记。
