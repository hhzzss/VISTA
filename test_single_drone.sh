#!/usr/bin/env bash
# 单机入口；调用前先 source ROS 和已编译的工作空间。
set -e

if ! command -v roslaunch >/dev/null 2>&1; then
  echo "未找到 roslaunch，请先 source ROS 和工作空间的 setup.bash。" >&2
  exit 1
fi

echo "启动地图、单机轻量仿真及 RViz；使用 2D Nav Goal 触发探索。"
exec roslaunch exploration_manager single_drone_exploration_with_map.xml "$@"
