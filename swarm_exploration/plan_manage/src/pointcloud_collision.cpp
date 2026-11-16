#include "plan_manage/pointcloud_collision.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace fast_planner {

PointCloudCollision::PointCloudCollision() 
    : safety_radius_(0.3), max_check_distance_(5.0), enabled_(true) {
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
}

PointCloudCollision::~PointCloudCollision() {}

void PointCloudCollision::init(ros::NodeHandle& nh) {
    nh_ = nh;
    nh.param("pointcloud_collision/safety_radius", safety_radius_, 0.3);
    nh.param("pointcloud_collision/max_check_distance", max_check_distance_, 5.0);
    
    ROS_INFO("[PointCloudCollision] Initialized");
    ROS_INFO("  safety_radius: %.2f", safety_radius_);
    ROS_INFO("  max_check_distance: %.2f", max_check_distance_);
}

void PointCloudCollision::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        ROS_WARN("[PointCloudCollision] Empty cloud received");
        return;
    }
    
    *cloud_ = *cloud;
    kdtree_->setInputCloud(cloud_);
    
    ROS_DEBUG("[PointCloudCollision] Cloud updated: %zu points", cloud_->size());
}

bool PointCloudCollision::checkCollision(const Eigen::Vector3d& point, double clearance) {
    if (!enabled_ || !cloud_ || cloud_->empty()) {
        return false;  // 无法检测时默认无碰撞
    }
    
    pcl::PointXYZ search_point;
    search_point.x = point.x();
    search_point.y = point.y();
    search_point.z = point.z();
    
    std::vector<int> point_idx;
    std::vector<float> point_distance;
    
    // 搜索最近点
    if (kdtree_->nearestKSearch(search_point, 1, point_idx, point_distance) > 0) {
        double distance = std::sqrt(point_distance[0]);
        return distance < clearance;
    }
    
    return false;
}

bool PointCloudCollision::checkCollision(const Eigen::Vector3d& start, 
                                          const Eigen::Vector3d& end, 
                                          double clearance) {
    if (!enabled_ || !cloud_ || cloud_->empty()) {
        return false;
    }
    
    // 线段采样检查
    double segment_length = (end - start).norm();
    int num_checks = std::max(2, static_cast<int>(segment_length / 0.1));  // 每10cm检查一次
    
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        Eigen::Vector3d point = start + t * (end - start);
        
        if (checkCollision(point, clearance)) {
            return true;  // 有碰撞
        }
    }
    
    return false;
}

double PointCloudCollision::getClearance(const Eigen::Vector3d& point) {
    if (!enabled_ || !cloud_ || cloud_->empty()) {
        return max_check_distance_;  // 无法检测时返回最大距离
    }
    
    pcl::PointXYZ search_point;
    search_point.x = point.x();
    search_point.y = point.y();
    search_point.z = point.z();
    
    std::vector<int> point_idx;
    std::vector<float> point_distance;
    
    if (kdtree_->nearestKSearch(search_point, 1, point_idx, point_distance) > 0) {
        return std::sqrt(point_distance[0]);
    }
    
    return max_check_distance_;
}

Eigen::Vector3d PointCloudCollision::getCollisionGradient(const Eigen::Vector3d& point, 
                                                           double clearance) {
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    
    if (!enabled_ || !cloud_ || cloud_->empty()) {
        return gradient;
    }
    
    // 找到最近的障碍物
    Eigen::Vector3d nearest;
    double distance;
    
    if (getNearestObstacle(point, nearest, distance)) {
        if (distance < clearance) {
            // 梯度方向为远离障碍物的方向
            gradient = (point - nearest).normalized();
            // 距离越近，梯度越大
            double strength = (clearance - distance) / clearance;
            gradient *= strength;
        }
    }
    
    return gradient;
}

void PointCloudCollision::checkTrajectory(const std::vector<Eigen::Vector3d>& trajectory,
                                           std::vector<bool>& collisions,
                                           std::vector<double>& clearances) {
    collisions.clear();
    clearances.clear();
    
    collisions.resize(trajectory.size());
    clearances.resize(trajectory.size());
    
    for (size_t i = 0; i < trajectory.size(); ++i) {
        clearances[i] = getClearance(trajectory[i]);
        collisions[i] = (clearances[i] < safety_radius_);
    }
}

bool PointCloudCollision::getNearestObstacle(const Eigen::Vector3d& point, 
                                              Eigen::Vector3d& nearest,
                                              double& distance) {
    if (!cloud_ || cloud_->empty()) {
        return false;
    }
    
    pcl::PointXYZ search_point;
    search_point.x = point.x();
    search_point.y = point.y();
    search_point.z = point.z();
    
    std::vector<int> point_idx;
    std::vector<float> point_distance;
    
    if (kdtree_->nearestKSearch(search_point, 1, point_idx, point_distance) > 0) {
        const auto& pt = (*cloud_)[point_idx[0]];
        nearest.x() = pt.x;
        nearest.y() = pt.y;
        nearest.z() = pt.z;
        distance = std::sqrt(point_distance[0]);
        return true;
    }
    
    return false;
}

} // namespace fast_planner
