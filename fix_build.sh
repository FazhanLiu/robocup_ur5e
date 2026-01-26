#!/bin/bash
# 快速修复脚本 - 修复构建错误

echo "=========================================="
echo "修复 Docker 构建错误"
echo "=========================================="
echo ""

echo "已修复的问题："
echo ""
echo "1. perception_grasp - PyYAML 冲突"
echo "   问题: 系统预装的 PyYAML 5.3.1 无法卸载"
echo "   修复: 使用 --ignore-installed PyYAML"
echo ""
echo "2. robocup_brain - py_trees_ros 不存在"
echo "   问题: PyPI 上没有 py_trees_ros==2.2.2"
echo "   修复: 使用 ROS 仓库中的 ros-noetic-py-trees-ros"
echo ""
echo "3. robocup_brain - moveit_msgs 缺失"
echo "   问题: CMake 找不到 moveit_msgs"
echo "   修复: 添加 ros-noetic-moveit-msgs 依赖"
echo ""

echo "=========================================="
echo "开始重新构建..."
echo "=========================================="
echo ""

# 清理旧的构建缓存
echo "清理旧的构建缓存..."
docker-compose down 2>/dev/null
docker system prune -f

echo ""
echo "重新构建镜像（这可能需要 10-30 分钟）..."
echo ""

# 重新构建
docker-compose build --no-cache

echo ""
echo "=========================================="
echo "构建完成！"
echo "=========================================="
echo ""

# 检查构建结果
if docker images | grep -q "robocup_ur5e"; then
    echo "✓ 镜像已创建："
    docker images | grep "robocup_ur5e"
    echo ""
    echo "启动容器："
    echo "  docker-compose up -d"
else
    echo "✗ 构建可能失败，请检查错误日志"
    echo ""
    echo "查看详细日志："
    echo "  docker-compose build robocup_brain"
    echo "  docker-compose build perception_yolo"
    echo "  docker-compose build perception_grasp"
fi
