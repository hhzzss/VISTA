#!/bin/bash

# VISTA Phase 2 一键安装脚本
# 自动复制所有Phase 2文件到正确位置并编译

echo "=========================================="
echo "VISTA Phase 2 安装脚本"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 查找工作空间
WORKSPACE=""
for ws in "$HOME/catkin_ws" "$HOME/vista1_ws" "$HOME/racer_ws"; do
    if [ -d "$ws/src/swarm_exploration" ]; then
        WORKSPACE="$ws"
        break
    fi
done

if [ -z "$WORKSPACE" ]; then
    echo -e "${RED}错误: 未找到包含 swarm_exploration 的工作空间${NC}"
    read -p "请输入工作空间路径 (例如 /home/hh/catkin_ws): " WORKSPACE
    WORKSPACE=$(eval echo "$WORKSPACE")
fi

if [ ! -d "$WORKSPACE/src/swarm_exploration" ]; then
    echo -e "${RED}错误: $WORKSPACE 中没有 swarm_exploration 包${NC}"
    exit 1
fi

echo -e "${GREEN}找到工作空间: $WORKSPACE${NC}"
echo ""

# 确认安装
echo "将安装以下Phase 2组件:"
echo "  1. 点云聚类与拓扑图 (pointcloud_cluster)"
echo "  2. EOHDT求解器 (eohdt_solver)"
echo "  3. ESDF-free避障 (pointcloud_collision)"
echo ""
read -p "确认安装? (y/n): " confirm
if [ "$confirm" != "y" ]; then
    echo "取消安装"
    exit 0
fi

echo ""
echo "[1/4] 复制文件..."

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SWARM_DIR="$WORKSPACE/src/swarm_exploration"

# 1. 点云聚类
cp "$SCRIPT_DIR/active_perception/include/active_perception/pointcloud_cluster.h" \
   "$SWARM_DIR/active_perception/include/active_perception/" 2>/dev/null || \
   echo -e "${YELLOW}警告: 头文件已存在或路径错误${NC}"
   
cp "$SCRIPT_DIR/active_perception/src/pointcloud_cluster.cpp" \
   "$SWARM_DIR/active_perception/src/" 2>/dev/null || \
   echo -e "${YELLOW}警告: 源文件已存在或路径错误${NC}"

# 2. EOHDT求解器
cp "$SCRIPT_DIR/exploration_manager/include/exploration_manager/eohdt_solver.h" \
   "$SWARM_DIR/exploration_manager/include/exploration_manager/" 2>/dev/null || \
   echo -e "${YELLOW}警告: 头文件已存在或路径错误${NC}"
   
cp "$SCRIPT_DIR/exploration_manager/src/eohdt_solver.cpp" \
   "$SWARM_DIR/exploration_manager/src/" 2>/dev/null || \
   echo -e "${YELLOW}警告: 源文件已存在或路径错误${NC}"

# 3. ESDF-free避障
cp "$SCRIPT_DIR/plan_manage/include/plan_manage/pointcloud_collision.h" \
   "$SWARM_DIR/plan_manage/include/plan_manage/" 2>/dev/null || \
   echo -e "${YELLOW}警告: 头文件已存在或路径错误${NC}"
   
cp "$SCRIPT_DIR/plan_manage/src/pointcloud_collision.cpp" \
   "$SWARM_DIR/plan_manage/src/" 2>/dev/null || \
   echo -e "${YELLOW}警告: 源文件已存在或路径错误${NC}"

echo -e "${GREEN}文件复制完成${NC}"
echo ""

# 更新 CMakeLists.txt
echo "[2/4] 更新 CMakeLists.txt..."

# active_perception
if ! grep -q "pointcloud_cluster.cpp" "$SWARM_DIR/active_perception/CMakeLists.txt"; then
    sed -i 's/src\/hgrid.cpp/src\/hgrid.cpp\n    src\/pointcloud_cluster.cpp/' \
        "$SWARM_DIR/active_perception/CMakeLists.txt"
    echo "  ✓ active_perception/CMakeLists.txt 已更新"
else
    echo "  ✓ active_perception/CMakeLists.txt 已包含 pointcloud_cluster"
fi

# exploration_manager
if ! grep -q "eohdt_solver.cpp" "$SWARM_DIR/exploration_manager/CMakeLists.txt"; then
    sed -i 's/src\/fast_exploration_manager.cpp/src\/fast_exploration_manager.cpp\n  src\/eohdt_solver.cpp/' \
        "$SWARM_DIR/exploration_manager/CMakeLists.txt"
    echo "  ✓ exploration_manager/CMakeLists.txt 已更新"
else
    echo "  ✓ exploration_manager/CMakeLists.txt 已包含 eohdt_solver"
fi

# plan_manage
if ! grep -q "pointcloud_collision.cpp" "$SWARM_DIR/plan_manage/CMakeLists.txt"; then
    sed -i 's/src\/planner_manager.cpp/src\/planner_manager.cpp\n    src\/pointcloud_collision.cpp/' \
        "$SWARM_DIR/plan_manage/CMakeLists.txt"
    echo "  ✓ plan_manage/CMakeLists.txt 已更新"
else
    echo "  ✓ plan_manage/CMakeLists.txt 已包含 pointcloud_collision"
fi

echo ""
echo "[3/4] 编译..."
cd "$WORKSPACE"

# 清理旧编译文件
rm -rf build/swarm_exploration
rm -rf build/active_perception
rm -rf build/exploration_manager
rm -rf build/plan_manage

# 编译
catkin_make 2>&1 | tee /tmp/phase2_build.log

if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo ""
    echo -e "${GREEN}==========================================${NC}"
    echo -e "${GREEN}Phase 2 安装成功!${NC}"
    echo -e "${GREEN}==========================================${NC}"
    echo ""
    echo "使用说明:"
    echo "  1. 启动仿真:"
    echo "     source $WORKSPACE/devel/setup.bash"
    echo "     roslaunch exploration_manager single_drone_exploration_with_map.xml"
    echo ""
    echo "  2. RViz可视化话题:"
    echo "     /pointcloud_clusters    - 聚类结果"
    echo "     /topology_graph         - 拓扑图"
    echo "     /eohdt_solution         - EOHDT路径"
    echo ""
    echo "  3. 查看文档:"
    echo "     cat $SCRIPT_DIR/PHASE2_SUMMARY.md"
else
    echo ""
    echo -e "${RED}==========================================${NC}"
    echo -e "${RED}编译失败!${NC}"
    echo -e "${RED}==========================================${NC}"
    echo ""
    echo "请检查错误日志: /tmp/phase2_build.log"
    echo ""
    echo "常见问题:"
    echo "  1. 缺少PCL库: sudo apt-get install libpcl-dev"
    echo "  2. 缺少Eigen3: sudo apt-get install libeigen3-dev"
    echo "  3. 头文件路径错误: 检查 CMakeLists.txt 中的 include_directories"
    exit 1
fi
