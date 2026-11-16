#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// 确保包含 KinoTrajGenerator 的头文件
#include "trajectory_generator/kino_traj_generator.h"

class TestNode {
public:
    TestNode() : nh_("~"), process_(0) {  // 初始化 process_
        path_pub_ = nh_.advertise<nav_msgs::Path>("optimized_path", 1);
        
        // 初始化轨迹生成器（如果需要）
        // kino_traj_generator_ = std::make_unique<KinoTrajGenerator>(process_);
    }
    
    void run() {
        ros::Rate rate(10);
        
        while (ros::ok()) {
            // 测试轨迹生成
            testTrajectoryGeneration();
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    
    // 修复：声明 process_ 成员变量
    int process_;
    
    // 如果需要使用智能指针管理
    std::unique_ptr<KinoTrajGenerator> kino_traj_generator_;
    
    void testTrajectoryGeneration() {
        // 局部使用的话在这里声明
        // auto kino_traj_generator = KinoTrajGenerator(process_);
        
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        
        // 添加路径点...
        
        path_pub_.publish(path);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    
    try {
        TestNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}
