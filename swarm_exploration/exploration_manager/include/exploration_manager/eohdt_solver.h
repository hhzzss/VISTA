#ifndef EOHDT_SOLVER_H
#define EOHDT_SOLVER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <queue>
#include <active_perception/pointcloud_cluster.h>

namespace fast_planner {

// EOHDT: Energy-Optimized Hierarchical Dual Tree
// 分层双树结构

// 上层树节点 (拓扑层)
struct TopoNode {
    int id;
    Eigen::Vector3d position;
    double information_gain;
    std::vector<int> child_viewpoints;  // 关联的下层视点ID
    std::vector<int> neighbors;         // 邻接拓扑节点
};

// 下层树节点 (视点层)
struct ViewNode {
    int id;
    int parent_topo_id;                 // 所属拓扑节点
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
    double utility;                     // 视点效用
    double cost_from_parent;            // 从父节点转移成本
};

// EOHDT图结构
struct EOHDTGraph {
    std::vector<TopoNode> topo_nodes;       // 上层拓扑节点
    std::vector<ViewNode> view_nodes;       // 下层视点节点
    Eigen::MatrixXd topo_adj_matrix;        // 上层邻接矩阵
    std::vector<std::vector<int>> view_paths; // 每个拓扑节点内的视点路径
};

// 求解结果
struct EOHDTSolution {
    std::vector<int> topo_order;            // 拓扑节点访问顺序
    std::vector<std::vector<int>> view_orders; // 每个拓扑节点的视点顺序
    double total_cost;                      // 总成本
    double total_utility;                   // 总效用
};

class EOHDTSolver {
public:
    EOHDTSolver();
    ~EOHDTSolver();
    
    // 初始化
    void init(ros::NodeHandle& nh);
    
    // 从拓扑图构建EOHDT
    void buildFromTopologyGraph(const TopologyGraph& topo_graph,
                                const std::vector<Eigen::Vector3d>& viewpoints,
                                EOHDTGraph& eohdt);
    
    // 求解EOHDT (主要接口)
    bool solve(const EOHDTGraph& graph,
               const Eigen::Vector3d& start_pos,
               EOHDTSolution& solution);
    
    // 上层求解: 拓扑TSP
    void solveUpperLayer(const EOHDTGraph& graph,
                         const Eigen::Vector3d& start_pos,
                         std::vector<int>& topo_order);
    
    // 下层求解: 每个拓扑节点内的视点优化
    void solveLowerLayer(const EOHDTGraph& graph,
                         const std::vector<int>& topo_order,
                         std::vector<std::vector<int>>& view_orders);
    
    // 能量函数计算
    double computeEnergy(const EOHDTGraph& graph,
                         const EOHDTSolution& solution);
    
    // 边成本: 距离 + 不确定性 + 协同成本
    double computeEdgeCost(const TopoNode& from, const TopoNode& to);
    
    // 视点效用: 信息增益 - 移动成本
    double computeViewUtility(const ViewNode& view, 
                              const Eigen::Vector3d& current_pos);
    
    // 迭代优化
    void iterativeOptimization(EOHDTGraph& graph,
                               EOHDTSolution& solution,
                               int max_iterations);
    
    // 2-opt局部搜索改进拓扑路径
    void twoOptImprovement(std::vector<int>& tour, 
                           const Eigen::MatrixXd& adj_matrix);
    
    // 可视化
    void visualizeSolution(const EOHDTGraph& graph, 
                           const EOHDTSolution& solution);

private:
    // 参数
    double alpha_;          // 距离权重
    double beta_;           // 不确定性权重
    double gamma_;          // 协同成本权重
    double max_edge_length_;
    int max_iterations_;
    
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher solution_pub_;
};

} // namespace fast_planner

#endif // EOHDT_SOLVER_H
