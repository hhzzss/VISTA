#!/bin/bash
# 单机仿真测试脚本（带地图）

cd ~/catkin_ws
source devel/setup.bash

echo "=== 启动单机仿真测试（带地图）==="
echo "可用地图: pillar.pcd, office.pcd, office2.pcd, office3.pcd"
echo "默认使用: pillar.pcd"
echo ""
echo "请在RViz中点击 '2D Nav Goal' 触发探索"
echo ""

# 杀死已有的ROS进程
pkill -9 -f "ros|gz" 2>/dev/null
sleep 2

# 启动单机仿真（带地图）
echo "[1/1] 启动单机仿真（地图: pillar.pcd）..."
roslaunch exploration_manager single_drone_exploration_with_map.xml

# 清理
echo "正在清理..."
pkill -9 -f "ros|gz" 2>/dev/null
