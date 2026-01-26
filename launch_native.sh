#!/bin/bash
# ============================================================================
# 原生 ROS 启动脚本（不使用 Docker）
# ============================================================================

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 加载环境变量
if [ -f .env ]; then
    source .env
fi

# Source ROS
source /opt/ros/noetic/setup.bash

# 检查工作区是否已编译
if [ ! -f devel/setup.bash ]; then
    echo -e "${YELLOW}工作区未编译，正在编译...${NC}"
    catkin build
fi

# Source 工作区
source devel/setup.bash

echo -e "${GREEN}"
echo "========================================"
echo "  RoboCup UR5e Native Launch"
echo "========================================"
echo -e "${NC}"
echo "ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "ROS_IP: ${ROS_IP}"
echo ""

# 启动节点
echo "请选择要启动的节点："
echo "  1) Brain (行为树)"
echo "  2) YOLO (物体检测)"
echo "  3) Grasp (抓取估计)"
echo "  4) 全部"
read -p "请输入选项 [1-4]: " choice

case $choice in
    1)
        roslaunch robocup_brain brain.launch
        ;;
    2)
        roslaunch perception_yolo yolo_detector.launch
        ;;
    3)
        roslaunch perception_grasp grasp_estimator.launch
        ;;
    4)
        # 使用 roslaunch 的 multi-launch 功能
        echo -e "${YELLOW}启动所有节点...${NC}"
        echo -e "${YELLOW}注意：建议使用 tmux 或多个终端分别启动${NC}"
        roslaunch robocup_brain brain.launch &
        sleep 2
        roslaunch perception_yolo yolo_detector.launch &
        sleep 2
        roslaunch perception_grasp grasp_estimator.launch
        ;;
    *)
        echo "无效选项！"
        exit 1
        ;;
esac
