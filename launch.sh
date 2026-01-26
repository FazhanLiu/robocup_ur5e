#!/bin/bash
# ============================================================================
# RoboCup UR5e 快速启动脚本
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查环境变量
if [ -f .env ]; then
    source .env
else
    echo -e "${RED}错误: .env 文件未找到！${NC}"
    exit 1
fi

# 打印横幅
echo -e "${GREEN}"
echo "========================================"
echo "  RoboCup UR5e Workspace Launcher"
echo "========================================"
echo -e "${NC}"

# 检查 Docker 和 docker-compose
if ! command -v docker &> /dev/null; then
    echo -e "${RED}错误: Docker 未安装！${NC}"
    exit 1
fi

if ! command -v docker-compose &> /dev/null; then
    echo -e "${RED}错误: docker-compose 未安装！${NC}"
    exit 1
fi

# 检查 NVIDIA Docker Runtime
if ! docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi &> /dev/null; then
    echo -e "${YELLOW}警告: NVIDIA Docker Runtime 可能未正确配置！${NC}"
    echo -e "${YELLOW}GPU 节点可能无法正常工作。请检查 nvidia-docker2 安装。${NC}"
fi

# 选择启动模式
echo "请选择启动模式："
echo "  1) 启动所有服务（Brain + YOLO + Grasp）"
echo "  2) 仅启动 Brain"
echo "  3) 仅启动 YOLO"
echo "  4) 仅启动 Grasp"
echo "  5) 自定义组合"
echo "  6) 停止所有服务"
echo "  7) 重新构建镜像"
read -p "请输入选项 [1-7]: " choice

case $choice in
    1)
        echo -e "${GREEN}启动所有服务...${NC}"
        docker-compose up -d
        ;;
    2)
        echo -e "${GREEN}启动 Brain 服务...${NC}"
        docker-compose up -d robocup_brain
        ;;
    3)
        echo -e "${GREEN}启动 YOLO 服务...${NC}"
        docker-compose up -d perception_yolo
        ;;
    4)
        echo -e "${GREEN}启动 Grasp 服务...${NC}"
        docker-compose up -d perception_grasp
        ;;
    5)
        echo "可用服务: robocup_brain, perception_yolo, perception_grasp"
        read -p "请输入要启动的服务（空格分隔）: " services
        docker-compose up -d $services
        ;;
    6)
        echo -e "${YELLOW}停止所有服务...${NC}"
        docker-compose down
        echo -e "${GREEN}所有服务已停止${NC}"
        exit 0
        ;;
    7)
        echo -e "${YELLOW}重新构建所有镜像...${NC}"
        docker-compose build --no-cache
        echo -e "${GREEN}构建完成！${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项！${NC}"
        exit 1
        ;;
esac

# 等待容器启动
echo -e "${YELLOW}等待容器启动...${NC}"
sleep 3

# 显示运行状态
echo ""
echo -e "${GREEN}当前运行的容器:${NC}"
docker-compose ps

echo ""
echo -e "${GREEN}查看实时日志:${NC}"
echo "  docker-compose logs -f"
echo ""
echo -e "${GREEN}进入容器:${NC}"
echo "  docker exec -it robocup_brain bash"
echo "  docker exec -it perception_yolo bash"
echo "  docker exec -it perception_grasp bash"
echo ""
echo -e "${GREEN}停止所有服务:${NC}"
echo "  docker-compose down"
echo ""
