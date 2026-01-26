#!/bin/bash
# ============================================================================
# RoboCup UR5e - Leader 一键验证脚本
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[!]${NC} $1"
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# 打印横幅
print_banner() {
    echo -e "${GREEN}"
    cat << "EOF"
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║     RoboCup UR5e Workspace - Leader Verification         ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

# 检查依赖
check_dependencies() {
    log_info "检查依赖..."
    
    if ! command -v docker &> /dev/null; then
        log_error "Docker 未安装！"
        exit 1
    fi
    log_success "Docker 已安装"
    
    if ! command -v docker-compose &> /dev/null; then
        log_error "docker-compose 未安装！"
        exit 1
    fi
    log_success "docker-compose 已安装"
    
    # 检查 NVIDIA Docker
    if docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi &> /dev/null; then
        log_success "NVIDIA Docker Runtime 正常"
    else
        log_warn "NVIDIA Docker Runtime 可能有问题（如果您有 GPU，请检查 nvidia-docker2）"
    fi
}

# 检查环境配置
check_env() {
    log_info "检查环境配置..."
    
    if [ ! -f .env ]; then
        log_error ".env 文件不存在！"
        exit 1
    fi
    
    source .env
    
    if [ -z "$ROS_MASTER_URI" ]; then
        log_error "ROS_MASTER_URI 未设置！"
        exit 1
    fi
    
    log_success "ROS_MASTER_URI: $ROS_MASTER_URI"
}

# 构建所有镜像
build_all() {
    log_info "开始构建所有 Docker 镜像..."
    echo ""
    
    log_info "构建 1/3: robocup_brain (CPU only)..."
    docker-compose build robocup_brain 2>&1 | tee /tmp/build_brain.log
    if grep -q "ERROR\|failed to build\|Service.*failed to build" /tmp/build_brain.log; then
        log_error "robocup_brain 构建失败！查看 /tmp/build_brain.log"
        exit 1
    fi
    log_success "robocup_brain 构建成功"
    
    log_info "构建 2/3: perception_yolo (CUDA 12.0)..."
    docker-compose build perception_yolo 2>&1 | tee /tmp/build_yolo.log
    if grep -q "ERROR\|failed to build\|Service.*failed to build" /tmp/build_yolo.log; then
        log_error "perception_yolo 构建失败！查看 /tmp/build_yolo.log"
        exit 1
    fi
    log_success "perception_yolo 构建成功"
    
    log_info "构建 3/3: perception_grasp (CUDA 11.3)..."
    docker-compose build perception_grasp 2>&1 | tee /tmp/build_grasp.log
    if grep -q "ERROR\|failed to build\|Service.*failed to build" /tmp/build_grasp.log; then
        log_error "perception_grasp 构建失败！查看 /tmp/build_grasp.log"
        exit 1
    fi
    log_success "perception_grasp 构建成功"
    
    echo ""
    log_success "所有镜像构建成功！"
}

# 启动容器
start_containers() {
    log_info "启动所有容器..."
    
    docker-compose up -d
    
    log_info "等待容器启动..."
    sleep 5
    
    # 检查容器状态
    if [ $(docker-compose ps | grep -c "Up") -eq 3 ]; then
        log_success "所有容器启动成功"
    else
        log_warn "部分容器启动失败，查看状态:"
        docker-compose ps
    fi
}

# 测试容器功能
test_containers() {
    log_info "测试容器功能..."
    echo ""
    
    # 测试 Brain
    log_info "测试 robocup_brain..."
    if docker exec robocup_brain bash -c "python3 -c 'import py_trees, py_trees_ros; print(\"OK\")'" &> /dev/null; then
        log_success "Brain: py_trees 可用"
    else
        log_error "Brain: py_trees 导入失败"
    fi
    
    # 测试 YOLO
    log_info "测试 perception_yolo..."
    if docker exec perception_yolo bash -c "python3 -c 'import torch; print(\"CUDA:\", torch.cuda.is_available())'" 2>&1 | grep -q "True"; then
        log_success "YOLO: CUDA 可用"
    else
        log_warn "YOLO: CUDA 不可用（将使用 CPU）"
    fi
    
    if docker exec perception_yolo bash -c "python3 -c 'from ultralytics import YOLO; print(\"OK\")'" &> /dev/null; then
        log_success "YOLO: ultralytics 可用"
    else
        log_error "YOLO: ultralytics 导入失败"
    fi
    
    # 测试 Grasp
    log_info "测试 perception_grasp..."
    if docker exec perception_grasp bash -c "python3 -c 'import torch; print(torch.version.cuda)'" 2>&1 | grep -q "11.3"; then
        log_success "Grasp: CUDA 11.3 正确"
    else
        log_warn "Grasp: CUDA 版本不是 11.3"
    fi
    
    if docker exec perception_grasp bash -c "python3 -c 'import open3d; print(\"OK\")'" &> /dev/null; then
        log_success "Grasp: Open3D 可用"
    else
        log_error "Grasp: Open3D 导入失败"
    fi
}

# 显示镜像大小
show_image_sizes() {
    echo ""
    log_info "Docker 镜像大小:"
    docker images | grep "robocup_ur5e" | awk '{printf "  %-35s %10s\n", $1":"$2, $7" "$8}'
}

# 生成报告
generate_report() {
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                   验证完成报告                             ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    log_success "所有检查已完成！"
    echo ""
    
    echo "📊 容器状态:"
    docker-compose ps
    echo ""
    
    echo "📦 下一步操作:"
    echo ""
    echo "  1. 查看日志:"
    echo "     docker-compose logs -f"
    echo ""
    echo "  2. 进入容器调试:"
    echo "     docker exec -it robocup_brain bash"
    echo "     docker exec -it perception_yolo bash"
    echo "     docker exec -it perception_grasp bash"
    echo ""
    echo "  3. 停止所有服务:"
    echo "     docker-compose down"
    echo ""
    echo "  4. 推送到 GitHub:"
    echo "     git init"
    echo "     git add ."
    echo "     git commit -m \"Initial commit: RoboCup UR5e Monorepo\""
    echo "     git remote add origin https://github.com/YOUR_USERNAME/robocup-ur5e-ws.git"
    echo "     git push -u origin main"
    echo ""
    echo "  5. (可选) 推送镜像到 Docker Hub:"
    echo "     阅读 LEADER_WORKFLOW.md 的步骤 2"
    echo ""
    
    log_info "团队成员可以按照 CONTRIBUTING.md 开始开发"
    echo ""
}

# 主函数
main() {
    print_banner
    
    # 检查是否在正确的目录
    if [ ! -f "docker-compose.yml" ]; then
        log_error "请在工作区根目录运行此脚本！"
        exit 1
    fi
    
    # 执行验证流程
    check_dependencies
    check_env
    
    echo ""
    read -p "开始构建 Docker 镜像？这可能需要 10-30 分钟。(y/n) " -n 1 -r
    echo ""
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_warn "构建已取消"
        exit 0
    fi
    
    build_all
    show_image_sizes
    start_containers
    test_containers
    generate_report
    
    log_success "所有验证完成！系统已就绪。🎉"
}

# 运行主函数
main "$@"
