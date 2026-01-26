#!/bin/bash
# 验证构建成功并启动系统

echo "=========================================="
echo "🔍 验证 Docker 镜像"
echo "=========================================="
echo ""

# 检查镜像
echo "已构建的镜像："
docker images | grep robocup_ur5e

echo ""
echo "=========================================="
echo "📊 镜像统计"
echo "=========================================="
echo ""

brain=$(docker images robocup_ur5e/brain --format "{{.Size}}")
yolo=$(docker images robocup_ur5e/perception_yolo --format "{{.Size}}")
grasp=$(docker images robocup_ur5e/perception_grasp --format "{{.Size}}")

echo "✓ Brain:           $brain"
echo "✓ Perception YOLO: $yolo"
echo "✓ Perception Grasp: $grasp"

echo ""
echo "=========================================="
echo "🚀 启动容器"
echo "=========================================="
echo ""

# 检查 .env 文件
if [ ! -f ".env" ]; then
    echo "⚠️  警告：.env 文件不存在，使用默认配置"
    echo "ROS_MASTER_URI=http://192.168.56.101:11311" > .env
    echo "ROS_IP=192.168.56.1" >> .env
fi

echo "ROS 配置："
cat .env | grep ROS_

echo ""
echo "启动所有服务..."
docker-compose up -d

echo ""
echo "等待容器启动（5秒）..."
sleep 5

echo ""
echo "=========================================="
echo "📋 容器状态"
echo "=========================================="
echo ""

docker-compose ps

echo ""
echo "=========================================="
echo "✅ 系统启动完成！"
echo "=========================================="
echo ""

echo "查看日志："
echo "  docker-compose logs -f              # 所有服务"
echo "  docker-compose logs -f brain        # 仅 Brain"
echo "  docker-compose logs -f perception_yolo"
echo "  docker-compose logs -f perception_grasp"
echo ""

echo "停止系统："
echo "  docker-compose down"
echo ""

echo "重启单个服务："
echo "  docker-compose restart brain"
echo ""

# 检查容器健康状态
echo "检查 ROS 连接（10秒后）..."
sleep 10

echo ""
echo "容器内部测试："
docker-compose exec -T brain bash -c "source /opt/ros/noetic/setup.bash && rostopic list" 2>/dev/null || echo "⚠️  ROS Master 未就绪，请确保 VirtualBox VM 中的 Gazebo 已启动"

echo ""
echo "=========================================="
echo "下一步："
echo "=========================================="
echo "1. 确保 VirtualBox VM (192.168.56.101) 中 Gazebo 已启动"
echo "2. 在 VM 中运行: roscore"
echo "3. 检查本机容器日志: docker-compose logs -f"
echo "4. 测试 ROS 通信: docker-compose exec brain rostopic list"
echo ""
