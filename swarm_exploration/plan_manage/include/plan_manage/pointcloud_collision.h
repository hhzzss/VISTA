#ifndef POINTCLOUD_COLLISION_H
#define POINTCLOUD_COLLISION_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Eigen>
#include <vector>

namespace fast_planner {

// 点云碰撞检测类
class PointCloudCollision {
public:
    PointCloudCollision();
    ~PointCloudCollision();
    
    // 初始化
    void init(ros::NodeHandle& nh);
    
    // 设置点云 (来自局部窗口)
    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // 碰撞检测: 检查点是否与点云碰撞
    bool checkCollision(const Eigen::Vector3d& point, double clearance);
    
    // 碰撞检测: 检查线段是否与点云碰撞
    bool checkCollision(const Eigen::Vector3d& start, 
                        const Eigen::Vector3d& end, 
                        double clearance);
    
    // 距离查询: 返回点到最近障碍物的距离
    double getClearance(const Eigen::Vector3d& point);
    
    // 梯度计算: 返回碰撞避免的梯度方向
    Eigen::Vector3d getCollisionGradient(const Eigen::Vector3d& point, 
                                         double clearance);
    
    // 批量检查轨迹点
    void checkTrajectory(const std::vector<Eigen::Vector3d>& trajectory,
                         std::vector<bool>& collisions,
                         std::vector<double>& clearances);
    
    // 获取最近的障碍物点
    bool getNearestObstacle(const Eigen::Vector3d& point, 
                            Eigen::Vector3d& nearest,
                            double& distance);
    
    // 启用/禁用点云碰撞检测
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

private:
    // 参数
    double safety_radius_;      // 安全半径
    double max_check_distance_; // 最大检测距离
    bool enabled_;
    
    // PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
    
    // ROS
    ros::NodeHandle nh_;
};

} // namespace fast_planner

#endif // POINTCLOUD_COLLISION_H
