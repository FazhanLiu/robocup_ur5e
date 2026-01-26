#!/bin/bash
# 快速测试 - 验证镜像是否存在

echo "检查 Docker 镜像..."
echo ""

docker images | grep robocup_ur5e

echo ""
image_count=$(docker images | grep "robocup_ur5e/" | wc -l)
echo "找到 $image_count 个 robocup_ur5e 镜像"

if [ "$image_count" -ge 3 ]; then
    echo "✅ 镜像完整，可以启动系统"
    echo ""
    echo "运行以下命令启动："
    echo "  ./start.sh"
else
    echo "❌ 镜像不完整，需要 3 个镜像"
    echo ""
    echo "运行以下命令重建："
    echo "  ./rebuild_all.sh"
fi
