#include "exploration_manager/eohdt_solver.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <cmath>

namespace fast_planner {

EOHDTSolver::EOHDTSolver() {}

EOHDTSolver::~EOHDTSolver() {}

void EOHDTSolver::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    nh.param("eohdt/alpha", alpha_, 1.0);
    nh.param("eohdt/beta", beta_, 0.5);
    nh.param("eohdt/gamma", gamma_, 0.3);
    nh.param("eohdt/max_edge_length", max_edge_length_, 10.0);
    nh.param("eohdt/max_iterations", max_iterations_, 100);
    
    solution_pub_ = nh.advertise<visualization_msgs::MarkerArray>("eohdt_solution", 1);
    
    ROS_INFO("[EOHDTSolver] Initialized");
    ROS_INFO("  alpha (distance): %.2f", alpha_);
    ROS_INFO("  beta (uncertainty): %.2f", beta_);
    ROS_INFO("  gamma (coordination): %.2f", gamma_);
}

void EOHDTSolver::buildFromTopologyGraph(const TopologyGraph& topo_graph,
                                          const std::vector<Eigen::Vector3d>& viewpoints,
                                          EOHDTGraph& eohdt) {
    
    eohdt.topo_nodes.clear();
    eohdt.view_nodes.clear();
    
    // 1. 构建上层拓扑节点
    for (size_t i = 0; i < topo_graph.nodes.size(); ++i) {
        TopoNode node;
        node.id = i;
        node.position = topo_graph.nodes[i].position;
        node.information_gain = topo_graph.nodes[i].information_gain;
        node.neighbors = topo_graph.nodes[i].neighbors;
        eohdt.topo_nodes.push_back(node);
    }
    
    // 2. 构建上层邻接矩阵
    int n = eohdt.topo_nodes.size();
    eohdt.topo_adj_matrix = topo_graph.adjacency_matrix;
    
    // 3. 构建下层视点节点 (将视点分配给最近的拓扑节点)
    for (size_t i = 0; i < viewpoints.size(); ++i) {
        ViewNode view;
        view.id = i;
        view.position = viewpoints[i];
        view.orientation = Eigen::Vector3d(0, 0, 0);  // 默认朝向
        
        // 找到最近的拓扑节点作为父节点
        double min_dist = std::numeric_limits<double>::max();
        int parent_id = -1;
        
        for (size_t j = 0; j < eohdt.topo_nodes.size(); ++j) {
            double dist = (view.position - eohdt.topo_nodes[j].position).norm();
            if (dist < min_dist) {
                min_dist = dist;
                parent_id = j;
            }
        }
        
        if (parent_id >= 0) {
            view.parent_topo_id = parent_id;
            view.utility = computeViewUtility(view, eohdt.topo_nodes[parent_id].position);
            eohdt.view_nodes.push_back(view);
            eohdt.topo_nodes[parent_id].child_viewpoints.push_back(i);
        }
    }
    
    // 4. 初始化视点路径
    eohdt.view_paths.resize(eohdt.topo_nodes.size());
    
    ROS_INFO("[EOHDTSolver] Built EOHDT graph: %zu topo nodes, %zu view nodes",
             eohdt.topo_nodes.size(), eohdt.view_nodes.size());
}

bool EOHDTSolver::solve(const EOHDTGraph& graph,
                        const Eigen::Vector3d& start_pos,
                        EOHDTSolution& solution) {
    
    if (graph.topo_nodes.empty()) {
        ROS_WARN("[EOHDTSolver] Empty graph, cannot solve");
        return false;
    }
    
    // 1. 求解上层: 拓扑TSP
    solveUpperLayer(graph, start_pos, solution.topo_order);
    
    // 2. 求解下层: 视点优化
    solveLowerLayer(graph, solution.topo_order, solution.view_orders);
    
    // 3. 计算总能量
    solution.total_cost = computeEnergy(graph, solution);
    
    ROS_INFO("[EOHDTSolver] Solution found: %zu topo nodes, cost=%.2f",
             solution.topo_order.size(), solution.total_cost);
    
    return true;
}

void EOHDTSolver::solveUpperLayer(const EOHDTGraph& graph,
                                   const Eigen::Vector3d& start_pos,
                                   std::vector<int>& topo_order) {
    
    int n = graph.topo_nodes.size();
    if (n == 0) return;
    
    // 贪心最近邻算法求解TSP
    std::vector<bool> visited(n, false);
    topo_order.clear();
    
    // 找到最近的起始节点
    int current = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
        double dist = (graph.topo_nodes[i].position - start_pos).norm();
        if (dist < min_dist) {
            min_dist = dist;
            current = i;
        }
    }
    
    topo_order.push_back(current);
    visited[current] = true;
    
    // 贪心选择最近未访问节点
    while (topo_order.size() < static_cast<size_t>(n)) {
        int next = -1;
        double min_cost = std::numeric_limits<double>::max();
        
        for (int i = 0; i < n; ++i) {
            if (visited[i]) continue;
            
            double cost = computeEdgeCost(graph.topo_nodes[current], graph.topo_nodes[i]);
            if (cost < min_cost) {
                min_cost = cost;
                next = i;
            }
        }
        
        if (next < 0) break;
        
        current = next;
        topo_order.push_back(current);
        visited[current] = true;
    }
    
    // 2-opt改进
    twoOptImprovement(topo_order, graph.topo_adj_matrix);
}

void EOHDTSolver::solveLowerLayer(const EOHDTGraph& graph,
                                   const std::vector<int>& topo_order,
                                   std::vector<std::vector<int>>& view_orders) {
    
    view_orders.clear();
    view_orders.resize(topo_order.size());
    
    for (size_t i = 0; i < topo_order.size(); ++i) {
        int topo_id = topo_order[i];
        const auto& view_ids = graph.topo_nodes[topo_id].child_viewpoints;
        
        if (view_ids.empty()) continue;
        
        // 对每个拓扑节点内的视点进行贪心排序
        std::vector<bool> visited(view_ids.size(), false);
        std::vector<int>& order = view_orders[i];
        
        // 从第一个视点开始
        order.push_back(view_ids[0]);
        visited[0] = true;
        
        while (order.size() < view_ids.size()) {
            int current_view = order.back();
            int next_idx = -1;
            double min_cost = std::numeric_limits<double>::max();
            
            for (size_t j = 0; j < view_ids.size(); ++j) {
                if (visited[j]) continue;
                
                double dist = (graph.view_nodes[current_view].position - 
                              graph.view_nodes[view_ids[j]].position).norm();
                if (dist < min_cost) {
                    min_cost = dist;
                    next_idx = j;
                }
            }
            
            if (next_idx < 0) break;
            
            order.push_back(view_ids[next_idx]);
            visited[next_idx] = true;
        }
    }
}

double EOHDTSolver::computeEnergy(const EOHDTGraph& graph,
                                   const EOHDTSolution& solution) {
    
    double total_cost = 0.0;
    
    // 上层拓扑路径成本
    for (size_t i = 0; i + 1 < solution.topo_order.size(); ++i) {
        int from = solution.topo_order[i];
        int to = solution.topo_order[i + 1];
        total_cost += computeEdgeCost(graph.topo_nodes[from], graph.topo_nodes[to]);
    }
    
    // 下层视点路径成本
    for (size_t i = 0; i < solution.view_orders.size(); ++i) {
        const auto& view_order = solution.view_orders[i];
        for (size_t j = 0; j + 1 < view_order.size(); ++j) {
            total_cost += (graph.view_nodes[view_order[j]].position - 
                          graph.view_nodes[view_order[j + 1]].position).norm();
        }
    }
    
    return total_cost;
}

double EOHDTSolver::computeEdgeCost(const TopoNode& from, const TopoNode& to) {
    // 边成本 = 距离 + 不确定性 + 协同成本
    double dist = (from.position - to.position).norm();
    double uncertainty = 1.0 / (1.0 + from.information_gain);
    double coordination_cost = 0.0;  // 多机协同成本，单机时为0
    
    return alpha_ * dist + beta_ * uncertainty + gamma_ * coordination_cost;
}

double EOHDTSolver::computeViewUtility(const ViewNode& view, 
                                        const Eigen::Vector3d& current_pos) {
    double dist = (view.position - current_pos).norm();
    double utility = view.utility - 0.1 * dist;  // 效用减去移动成本
    return utility;
}

void EOHDTSolver::iterativeOptimization(EOHDTGraph& graph,
                                         EOHDTSolution& solution,
                                         int max_iterations) {
    
    double prev_cost = solution.total_cost;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 上层优化: 2-opt
        twoOptImprovement(solution.topo_order, graph.topo_adj_matrix);
        
        // 下层优化: 重新求解视点顺序
        solveLowerLayer(graph, solution.topo_order, solution.view_orders);
        
        // 计算新成本
        solution.total_cost = computeEnergy(graph, solution);
        
        // 检查收敛
        if (std::abs(prev_cost - solution.total_cost) < 0.01) {
            break;
        }
        prev_cost = solution.total_cost;
    }
}

void EOHDTSolver::twoOptImprovement(std::vector<int>& tour, 
                                     const Eigen::MatrixXd& adj_matrix) {
    
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 0; i < tour.size(); ++i) {
            for (size_t j = i + 2; j < tour.size(); ++j) {
                int a = tour[i];
                int b = tour[(i + 1) % tour.size()];
                int c = tour[j];
                int d = tour[(j + 1) % tour.size()];
                
                double before = adj_matrix(a, b) + adj_matrix(c, d);
                double after = adj_matrix(a, c) + adj_matrix(b, d);
                
                if (after < before) {
                    // 翻转i+1到j之间的路径
                    std::reverse(tour.begin() + i + 1, tour.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

void EOHDTSolver::visualizeSolution(const EOHDTGraph& graph, 
                                     const EOHDTSolution& solution) {
    
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    
    // 可视化上层拓扑路径
    visualization_msgs::Marker topo_path_marker;
    topo_path_marker.header.frame_id = "world";
    topo_path_marker.header.stamp = ros::Time::now();
    topo_path_marker.ns = "eohdt_topo_path";
    topo_path_marker.id = marker_id++;
    topo_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    topo_path_marker.action = visualization_msgs::Marker::ADD;
    topo_path_marker.scale.x = 0.05;
    topo_path_marker.color.r = 1.0;
    topo_path_marker.color.g = 0.0;
    topo_path_marker.color.b = 0.0;
    topo_path_marker.color.a = 1.0;
    
    for (int node_id : solution.topo_order) {
        geometry_msgs::Point p;
        p.x = graph.topo_nodes[node_id].position.x();
        p.y = graph.topo_nodes[node_id].position.y();
        p.z = graph.topo_nodes[node_id].position.z();
        topo_path_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(topo_path_marker);
    
    // 可视化下层视点路径
    for (size_t i = 0; i < solution.view_orders.size(); ++i) {
        if (solution.view_orders[i].empty()) continue;
        
        visualization_msgs::Marker view_path_marker;
        view_path_marker.header = topo_path_marker.header;
        view_path_marker.ns = "eohdt_view_paths";
        view_path_marker.id = marker_id++;
        view_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        view_path_marker.action = visualization_msgs::Marker::ADD;
        view_path_marker.scale.x = 0.02;
        view_path_marker.color.r = 0.0;
        view_path_marker.color.g = 0.0;
        view_path_marker.color.b = 1.0;
        view_path_marker.color.a = 0.8;
        
        for (int view_id : solution.view_orders[i]) {
            geometry_msgs::Point p;
            p.x = graph.view_nodes[view_id].position.x();
            p.y = graph.view_nodes[view_id].position.y();
            p.z = graph.view_nodes[view_id].position.z();
            view_path_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(view_path_marker);
    }
    
    solution_pub_.publish(marker_array);
}

} // namespace fast_planner
