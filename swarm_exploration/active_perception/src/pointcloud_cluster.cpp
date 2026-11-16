#include "active_perception/pointcloud_cluster.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace fast_planner {

PointCloudCluster::PointCloudCluster() {
    kdtree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
}

PointCloudCluster::~PointCloudCluster() {}

void PointCloudCluster::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    nh.param("pointcloud_cluster/cluster_tolerance", cluster_tolerance_, 0.5);
    nh.param("pointcloud_cluster/min_cluster_size", min_cluster_size_, 50);
    nh.param("pointcloud_cluster/max_cluster_size", max_cluster_size_, 10000);
    nh.param("pointcloud_cluster/feature_radius", feature_radius_, 0.5);
    nh.param("pointcloud_cluster/merge_threshold", merge_threshold_, 1.0);
    
    cluster_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pointcloud_clusters", 1);
    graph_pub_ = nh.advertise<visualization_msgs::MarkerArray>("topology_graph", 1);
    
    ROS_INFO("[PointCloudCluster] Initialized");
}

void PointCloudCluster::extractClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
    
    if (cloud->empty()) {
        ROS_WARN("[PointCloudCluster] Input cloud is empty");
        return;
    }
    
    clusters.clear();
    
    // 降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_grid.filter(*cloud_filtered);
    
    // 欧氏聚类
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClustering(cloud_filtered, cluster_indices);
    
    // 提取聚类
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : indices.indices) {
            cluster->push_back((*cloud_filtered)[idx]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        clusters.push_back(cluster);
    }
    
    ROS_INFO("[PointCloudCluster] Extracted %zu clusters", clusters.size());
}

void PointCloudCluster::euclideanClustering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<pcl::PointIndices>& cluster_indices) {
    
    kdtree_->setInputCloud(cloud);
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(kdtree_);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

void PointCloudCluster::geometricConsistencyGrouping(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& grouped_clusters) {
    
    grouped_clusters.clear();
    if (clusters.empty()) return;
    
    std::vector<bool> merged(clusters.size(), false);
    
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (merged[i]) continue;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr group(new pcl::PointCloud<pcl::PointXYZ>);
        *group += *clusters[i];
        merged[i] = true;
        
        Eigen::Vector3d center_i = getClusterCenter(clusters[i]);
        
        for (size_t j = i + 1; j < clusters.size(); ++j) {
            if (merged[j]) continue;
            
            Eigen::Vector3d center_j = getClusterCenter(clusters[j]);
            if ((center_i - center_j).norm() < merge_threshold_) {
                *group += *clusters[j];
                merged[j] = true;
                center_i = getClusterCenter(group);
            }
        }
        
        grouped_clusters.push_back(group);
    }
}

void PointCloudCluster::buildTopologyGraph(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
    const Eigen::Vector3d& current_pos,
    TopologyGraph& graph) {
    
    graph.nodes.clear();
    graph.edges.clear();
    
    if (clusters.empty()) return;
    
    // 构建节点
    for (size_t i = 0; i < clusters.size(); ++i) {
        TopologyNode node;
        node.id = i;
        node.position = getClusterCenter(clusters[i]);
        node.normal = getClusterNormal(clusters[i]);
        node.information_gain = computeInformationGain(clusters[i], current_pos);
        node.cloud = clusters[i];
        graph.nodes.push_back(node);
    }
    
    // 构建边
    int n = graph.nodes.size();
    graph.adjacency_matrix = Eigen::MatrixXd::Zero(n, n);
    
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double dist = (graph.nodes[i].position - graph.nodes[j].position).norm();
            double weight = dist;
            
            graph.adjacency_matrix(i, j) = weight;
            graph.adjacency_matrix(j, i) = weight;
            
            if (dist < 10.0) {
                graph.edges.push_back({i, j});
                graph.nodes[i].neighbors.push_back(j);
                graph.nodes[j].neighbors.push_back(i);
            }
        }
    }
    
    ROS_INFO("[PointCloudCluster] Built topology graph: %zu nodes, %zu edges", 
             graph.nodes.size(), graph.edges.size());
}

Eigen::Vector3d PointCloudCluster::getClusterCenter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
    
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    return centroid.head<3>();
}

Eigen::Vector3d PointCloudCluster::getClusterNormal(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
    
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cluster);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    return eigen_vectors.col(2).cast<double>();
}

double PointCloudCluster::computeInformationGain(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    const Eigen::Vector3d& viewpoint) {
    
    double gain = 0.0;
    for (const auto& pt : cluster->points) {
        Eigen::Vector3d point(pt.x, pt.y, pt.z);
        gain += 1.0 / (1.0 + (point - viewpoint).norm());
    }
    return gain;
}

void PointCloudCluster::visualizeClusters(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
    
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < clusters.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "clusters";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = static_cast<float>(rand()) / RAND_MAX;
        marker.color.g = static_cast<float>(rand()) / RAND_MAX;
        marker.color.b = static_cast<float>(rand()) / RAND_MAX;
        marker.color.a = 1.0;
        
        for (const auto& pt : clusters[i]->points) {
            geometry_msgs::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    cluster_pub_.publish(marker_array);
}

void PointCloudCluster::visualizeTopologyGraph(const TopologyGraph& graph) {
    
    visualization_msgs::MarkerArray marker_array;
    
    // 节点
    for (const auto& node : graph.nodes) {
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = "world";
        node_marker.header.stamp = ros::Time::now();
        node_marker.ns = "topology_nodes";
        node_marker.id = node.id;
        node_marker.type = visualization_msgs::Marker::SPHERE;
        node_marker.action = visualization_msgs::Marker::ADD;
        
        node_marker.pose.position.x = node.position.x();
        node_marker.pose.position.y = node.position.y();
        node_marker.pose.position.z = node.position.z();
        
        double scale = 0.2 + 0.1 * std::min(node.information_gain / 100.0, 1.0);
        node_marker.scale.x = scale;
        node_marker.scale.y = scale;
        node_marker.scale.z = scale;
        node_marker.color.r = 0.0;
        node_marker.color.g = 0.8;
        node_marker.color.b = 0.2;
        node_marker.color.a = 0.8;
        
        marker_array.markers.push_back(node_marker);
    }
    
    // 边
    int edge_id = 0;
    for (const auto& edge : graph.edges) {
        visualization_msgs::Marker edge_marker;
        edge_marker.header.frame_id = "world";
        edge_marker.header.stamp = ros::Time::now();
        edge_marker.ns = "topology_edges";
        edge_marker.id = edge_id++;
        edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
        edge_marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point p1, p2;
        p1.x = graph.nodes[edge.first].position.x();
        p1.y = graph.nodes[edge.first].position.y();
        p1.z = graph.nodes[edge.first].position.z();
        p2.x = graph.nodes[edge.second].position.x();
        p2.y = graph.nodes[edge.second].position.y();
        p2.z = graph.nodes[edge.second].position.z();
        
        edge_marker.points.push_back(p1);
        edge_marker.points.push_back(p2);
        edge_marker.scale.x = 0.02;
        edge_marker.color.r = 0.5;
        edge_marker.color.g = 0.5;
        edge_marker.color.b = 0.5;
        edge_marker.color.a = 0.5;
        
        marker_array.markers.push_back(edge_marker);
    }
    
    graph_pub_.publish(marker_array);
}

} // namespace fast_planner
