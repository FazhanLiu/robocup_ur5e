#!/bin/bash
# 检查当前构建状态

echo "=========================================="
echo "RoboCup UR5e 构建状态检查"
echo "=========================================="
echo ""

# 检查 Dockerfile 是否已修复
echo "1. 检查 Dockerfile 修复状态..."
echo ""

if grep -q "ros-noetic-moveit-msgs" docker/Dockerfile.brain; then
    echo "   ✓ Dockerfile.brain 已修复（包含 moveit_msgs）"
else
    echo "   ✗ Dockerfile.brain 需要修复"
fi

if grep -q "\-\-ignore-installed PyYAML" docker/Dockerfile.grasp; then
    echo "   ✓ Dockerfile.grasp 已修复（PyYAML 冲突）"
else
    echo "   ✗ Dockerfile.grasp 需要修复"
fi

echo ""
echo "2. 检查已构建的镜像..."
echo ""

if docker images | grep -q "robocup_ur5e/brain"; then
    echo "   ✓ robocup_ur5e/brain 镜像存在"
    docker images | grep "robocup_ur5e/brain"
else
    echo "   ✗ robocup_ur5e/brain 镜像不存在"
fi

if docker images | grep -q "robocup_ur5e/perception_yolo"; then
    echo "   ✓ robocup_ur5e/perception_yolo 镜像存在"
    docker images | grep "robocup_ur5e/perception_yolo"
else
    echo "   ✗ robocup_ur5e/perception_yolo 镜像不存在"
fi

if docker images | grep -q "robocup_ur5e/perception_grasp"; then
    echo "   ✓ robocup_ur5e/perception_grasp 镜像存在"
    docker images | grep "robocup_ur5e/perception_grasp"
else
    echo "   ✗ robocup_ur5e/perception_grasp 镜像不存在"
fi

echo ""
echo "3. 检查运行中的容器..."
echo ""

if docker ps | grep -q "robocup_brain\|perception_yolo\|perception_grasp"; then
    echo "   运行中的容器:"
    docker ps | grep "robocup_brain\|perception_yolo\|perception_grasp"
else
    echo "   ✗ 没有运行中的容器"
fi

echo ""
echo "=========================================="
echo "建议操作："
echo "=========================================="
echo ""

if ! docker images | grep -q "robocup_ur5e"; then
    echo "⚠️  镜像尚未构建"
    echo "   运行: ./fix_build.sh"
elif ! docker ps | grep -q "robocup_brain\|perception_yolo\|perception_grasp"; then
    echo "⚠️  容器未启动"
    echo "   运行: docker-compose up -d"
else
    echo "✅ 系统正常运行！"
    echo ""
    echo "查看日志:"
    echo "   docker-compose logs -f"
    echo ""
    echo "进入容器:"
    echo "   docker exec -it robocup_brain bash"
fi

echo ""
