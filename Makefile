# RoboCup UR5e Makefile
# 简化常用命令

.PHONY: help build up down logs clean test

# 默认目标
help:
	@echo "RoboCup UR5e 快速命令"
	@echo ""
	@echo "构建和启动:"
	@echo "  make build       - 构建所有 Docker 镜像"
	@echo "  make up          - 启动所有容器"
	@echo "  make down        - 停止所有容器"
	@echo "  make restart     - 重启所有容器"
	@echo ""
	@echo "单独操作:"
	@echo "  make build-brain - 构建 Brain 镜像"
	@echo "  make build-yolo  - 构建 YOLO 镜像"
	@echo "  make build-grasp - 构建 Grasp 镜像"
	@echo ""
	@echo "调试:"
	@echo "  make logs        - 查看所有日志"
	@echo "  make logs-brain  - 查看 Brain 日志"
	@echo "  make logs-yolo   - 查看 YOLO 日志"
	@echo "  make logs-grasp  - 查看 Grasp 日志"
	@echo "  make shell-brain - 进入 Brain 容器"
	@echo "  make shell-yolo  - 进入 YOLO 容器"
	@echo "  make shell-grasp - 进入 Grasp 容器"
	@echo ""
	@echo "测试:"
	@echo "  make test        - 运行所有测试"
	@echo "  make test-gpu    - 测试 GPU 访问"
	@echo ""
	@echo "清理:"
	@echo "  make clean       - 清理编译产物"
	@echo "  make clean-docker - 清理 Docker 资源"

# ============================================================================
# 构建
# ============================================================================

build:
	@echo "🔨 构建所有 Docker 镜像..."
	docker-compose build

build-brain:
	@echo "🔨 构建 Brain 镜像..."
	docker-compose build robocup_brain

build-yolo:
	@echo "🔨 构建 YOLO 镜像..."
	docker-compose build perception_yolo

build-grasp:
	@echo "🔨 构建 Grasp 镜像..."
	docker-compose build perception_grasp

# ============================================================================
# 启动/停止
# ============================================================================

up:
	@echo "🚀 启动所有容器..."
	docker-compose up -d
	@echo ""
	@echo "✅ 容器已启动！使用 'make logs' 查看日志"

down:
	@echo "🛑 停止所有容器..."
	docker-compose down

restart:
	@echo "🔄 重启所有容器..."
	docker-compose restart

ps:
	@echo "📊 容器状态:"
	@docker-compose ps

# ============================================================================
# 日志
# ============================================================================

logs:
	docker-compose logs -f

logs-brain:
	docker-compose logs -f robocup_brain

logs-yolo:
	docker-compose logs -f perception_yolo

logs-grasp:
	docker-compose logs -f perception_grasp

# ============================================================================
# 进入容器
# ============================================================================

shell-brain:
	docker exec -it robocup_brain bash

shell-yolo:
	docker exec -it perception_yolo bash

shell-grasp:
	docker exec -it perception_grasp bash

# ============================================================================
# 测试
# ============================================================================

test:
	@echo "🧪 运行测试..."
	@./scripts/run_tests.sh

test-gpu:
	@echo "🎮 测试 GPU 访问..."
	docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi

test-ros:
	@echo "🤖 测试 ROS 连接..."
	docker exec robocup_brain bash -c "source /workspace/devel/setup.bash && rostopic list"

# ============================================================================
# 清理
# ============================================================================

clean:
	@echo "🧹 清理编译产物..."
	rm -rf build/ devel/ logs/
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete

clean-docker:
	@echo "🧹 清理 Docker 资源..."
	docker-compose down -v
	docker system prune -f

# ============================================================================
# 开发辅助
# ============================================================================

format:
	@echo "📝 格式化 Python 代码..."
	find src -name "*.py" -exec black {} \;

lint:
	@echo "🔍 检查代码风格..."
	find src -name "*.py" -exec pylint {} \;

# ============================================================================
# GitHub 工作流
# ============================================================================

git-status:
	@git status

git-push:
	@echo "📤 推送到 GitHub..."
	@git push origin $$(git branch --show-current)

# ============================================================================
# Leader 专用命令
# ============================================================================

push-dockerhub:
	@echo "📦 推送镜像到 Docker Hub..."
	@read -p "Docker Hub 用户名: " username; \
	docker tag robocup_ur5e/brain:latest $$username/brain:latest; \
	docker tag robocup_ur5e/perception_yolo:latest $$username/perception_yolo:latest; \
	docker tag robocup_ur5e/perception_grasp:latest $$username/perception_grasp:latest; \
	docker push $$username/brain:latest; \
	docker push $$username/perception_yolo:latest; \
	docker push $$username/perception_grasp:latest

verify-all:
	@echo "✅ 验证所有容器..."
	@make build
	@make up
	@sleep 5
	@make ps
	@make test-ros
	@echo ""
	@echo "✅ 验证完成！"
