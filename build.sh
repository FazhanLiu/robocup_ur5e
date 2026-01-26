#!/bin/bash
# ============================================================================
# 工作区编译脚本
# ============================================================================

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}"
echo "========================================"
echo "  Building RoboCup UR5e Workspace"
echo "========================================"
echo -e "${NC}"

# Source ROS
source /opt/ros/noetic/setup.bash

# 初始化 catkin workspace（如果需要）
if [ ! -f src/CMakeLists.txt ]; then
    echo -e "${YELLOW}初始化 catkin 工作区...${NC}"
    catkin init
fi

# 配置 catkin
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# 构建选项
echo "请选择构建选项："
echo "  1) 完整构建（推荐）"
echo "  2) 仅构建 common_msgs"
echo "  3) 仅构建 robocup_brain"
echo "  4) 仅构建 perception_yolo"
echo "  5) 仅构建 perception_grasp"
echo "  6) 清理并重新构建"
read -p "请输入选项 [1-6]: " choice

case $choice in
    1)
        echo -e "${GREEN}构建所有包...${NC}"
        catkin build
        ;;
    2)
        echo -e "${GREEN}构建 common_msgs...${NC}"
        catkin build common_msgs
        ;;
    3)
        echo -e "${GREEN}构建 robocup_brain...${NC}"
        catkin build robocup_brain
        ;;
    4)
        echo -e "${GREEN}构建 perception_yolo...${NC}"
        catkin build perception_yolo
        ;;
    5)
        echo -e "${GREEN}构建 perception_grasp...${NC}"
        catkin build perception_grasp
        ;;
    6)
        echo -e "${YELLOW}清理并重新构建...${NC}"
        catkin clean -y
        catkin build
        ;;
    *)
        echo -e "${RED}无效选项！${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}构建完成！${NC}"
echo ""
echo -e "${YELLOW}运行以下命令激活工作区:${NC}"
echo "  source devel/setup.bash"
echo ""
