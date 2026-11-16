#ifndef POINTCLOUD_CLUSTER_H
#define POINTCLOUD_CLUSTER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Eigen>
#include <vector>
#include <memory>

namespace fast_planner {

// 拓扑图节点
struct TopologyNode {
    int id;
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    double information_gain;
    std::vector<int> neighbors;  // 邻接节点ID
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // 聚类点云
};

// 拓扑图
struct TopologyGraph {
    std::vector<TopologyNode> nodes;
    std::vector<std::pair<int, int>> edges;  // (node1_id, node2_id)
    Eigen::MatrixXd adjacency_matrix;  // 边权重矩阵
};

class PointCloudCluster {
public:
    PointCloudCluster();
    ~PointCloudCluster();
    
    // 初始化
    void init(ros::NodeHandle& nh);
    
    // 从局部窗口点云提取聚类
    void extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
    
    // 构建拓扑图
    void buildTopologyGraph(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
                            const Eigen::Vector3d& current_pos,
                            TopologyGraph& graph);
    
    // 提取FPFH特征
    void extractFPFHFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             pcl::PointCloud<pcl::FPFHSignature33>::Ptr& features);
    
    // 欧氏聚类
    void euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             std::vector<pcl::PointIndices>& cluster_indices);
    
    // 几何一致性分组
    void geometricConsistencyGrouping(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& grouped_clusters);
    
    // 获取聚类中心
    Eigen::Vector3d getClusterCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    
    // 获取聚类法向量
    Eigen::Vector3d getClusterNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    
    // 计算信息增益
    double computeInformationGain(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                                  const Eigen::Vector3d& viewpoint);
    
    // 可视化
    void visualizeClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
    void visualizeTopologyGraph(const TopologyGraph& graph);

private:
    // 参数
    double cluster_tolerance_;      // 聚类容差
    int min_cluster_size_;          // 最小聚类点数
    int max_cluster_size_;          // 最大聚类点数
    double feature_radius_;         // FPFH特征半径
    double merge_threshold_;        // 合并阈值
    double normal_estimation_radius_; // 法向量估计半径
    
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher cluster_pub_;
    ros::Publisher graph_pub_;
    
    // PCL
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
};

} // namespace fast_planner

#endif // POINTCLOUD_CLUSTER_H
