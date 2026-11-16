#!/bin/bash
# 单机仿真测试脚本

cd ~/catkin_ws
source devel/setup.bash

echo "=== 启动单机仿真测试 ==="
echo "步骤1: 启动仿真器 (地图 + 里程计)..."
echo "步骤2: 启动探索节点..."
echo "步骤3: 请在RViz中点击 '2D Nav Goal' 触发探索"
echo ""

# 杀死已有的ROS进程
pkill -f "rosrun\|roslaunch" 2>/dev/null
sleep 2

# 启动roscore
roscore &
sleep 3

# 启动单机探索节点 (包含内置仿真器)
echo "[1/2] 启动单机仿真 (包含地图和里程计)..."
roslaunch /home/hh/VISTA/swarm_exploration/exploration_manager/launch/single_drone_exploration.xml drone_id:=1 simulation:=true &
sleep 5

# 启动RViz
echo "[2/2] 启动RViz..."
echo "请在RViz中点击 '2D Nav Goal' (快捷键G) 来触发探索"
roslaunch exploration_manager rviz.launch

# 清理
echo "正在清理..."
pkill -f "rosrun\|roslaunch" 2>/dev/null
