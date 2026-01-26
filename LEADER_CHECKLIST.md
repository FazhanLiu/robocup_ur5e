# Leader 快速检查清单 ✅

## 阶段 1: 本地验证（您现在要做的）

### 步骤 1: 运行一键验证脚本
```bash
cd /home/suhang/robocup_ur5e_ws
./verify_system.sh
```

这个脚本会自动：
- ✅ 检查 Docker 和 nvidia-docker2
- ✅ 构建所有 3 个镜像
- ✅ 启动容器
- ✅ 测试每个容器的功能
- ✅ 生成验证报告

**预计时间**: 10-30 分钟（取决于网络和 CPU）

---

### 步骤 2: 手动验证（可选，如果脚本运行成功可跳过）

#### 2.1 构建镜像
```bash
# 方法 A: 使用脚本
./launch.sh  # 选项 7: 重新构建镜像

# 方法 B: 手动构建
docker-compose build
```

#### 2.2 启动容器
```bash
docker-compose up -d
```

#### 2.3 检查状态
```bash
docker-compose ps

# 应该看到:
# robocup_brain         ... Up
# perception_yolo       ... Up
# perception_grasp      ... Up
```

#### 2.4 查看日志
```bash
# 所有容器
docker-compose logs

# 单个容器
docker-compose logs -f robocup_brain
docker-compose logs -f perception_yolo
docker-compose logs -f perception_grasp
```

#### 2.5 测试功能
```bash
# Brain: 测试 py_trees
docker exec robocup_brain python3 -c "import py_trees, py_trees_ros; print('OK')"

# YOLO: 测试 CUDA + ultralytics
docker exec perception_yolo python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"
docker exec perception_yolo python3 -c "from ultralytics import YOLO; print('OK')"

# Grasp: 测试 CUDA 11.3 + Open3D
docker exec perception_grasp python3 -c "import torch; print('CUDA:', torch.version.cuda)"
docker exec perception_grasp python3 -c "import open3d; print('OK')"
```

#### 2.6 测试 GPU
```bash
# 在容器内查看 GPU
docker exec perception_yolo nvidia-smi
docker exec perception_grasp nvidia-smi

# 应该看到您的 RTX 5070 Ti
```

---

## 阶段 2: 推送到 GitHub

### 步骤 1: 初始化 Git
```bash
cd /home/suhang/robocup_ur5e_ws
git init
```

### 步骤 2: 第一次提交
```bash
git add .
git commit -m "Initial commit: RoboCup UR5e Monorepo

- Add common_msgs interface contract
- Add robocup_brain (behavior tree, Suhang Xia)
- Add perception_yolo (CUDA 12.0, Fazhan & Ruiyi)
- Add perception_grasp (CUDA 11.3, Muye Yuan)
- Add Docker orchestration with GPU support
- Add comprehensive documentation"
```

### 步骤 3: 创建 GitHub 仓库
1. 访问: https://github.com/new
2. 仓库名: `robocup-ur5e-ws`
3. 可见性: Private（推荐）或 Public
4. **不要** 勾选 "Initialize with README"

### 步骤 4: 推送代码
```bash
# 替换 YOUR_USERNAME 为您的 GitHub 用户名
git remote add origin https://github.com/YOUR_USERNAME/robocup-ur5e-ws.git
git branch -M main
git push -u origin main
```

### 步骤 5: 添加协作者
1. 仓库页面 → Settings → Collaborators
2. 添加:
   - Fazhan (GitHub 用户名)
   - Ruiyi (GitHub 用户名)
   - Muye Yuan (GitHub 用户名)

---

## 阶段 3: 通知团队（可选：先做阶段 4）

### 给团队发邮件
```
主题: RoboCup UR5e 工作区已就绪 🚀

大家好！

RoboCup UR5e 的开发环境已经搭建完成，请按照以下步骤开始：

1. 克隆仓库:
   git clone https://github.com/YOUR_USERNAME/robocup-ur5e-ws.git
   cd robocup-ur5e-ws

2. 阅读文档:
   - README.md: 完整系统文档
   - CONTRIBUTING.md: 开发指南
   - QUICKSTART.md: 快速参考

3. 启动环境:
   docker-compose build  # 首次运行
   docker-compose up -d

4. 各模块职责:
   - Fazhan & Ruiyi: src/perception_yolo/ (YOLO 检测)
   - Muye Yuan: src/perception_grasp/ (抓取估计)
   - Suhang (我): src/robocup_brain/ (行为树)

5. 开发流程:
   详见 CONTRIBUTING.md

有问题随时在 GitHub Issues 中讨论或直接联系我。

Best,
Suhang Xia
```

---

## 阶段 4: （可选但推荐）推送镜像到 Docker Hub

**为什么要推送？**
- 团队成员可以直接拉取镜像（不需要等待 10-30 分钟构建）
- 特别适合 Mac/Windows 用户（本地构建可能有问题）

### 步骤 1: 注册 Docker Hub
访问: https://hub.docker.com/signup

### 步骤 2: 创建仓库
在 Docker Hub 创建 3 个公开仓库:
- `robocup_brain`
- `perception_yolo`
- `perception_grasp`

### 步骤 3: 登录
```bash
docker login
# 输入用户名和密码
```

### 步骤 4: 打标签并推送
```bash
# 替换 YOUR_DOCKERHUB_USERNAME
export DOCKER_USER=YOUR_DOCKERHUB_USERNAME

# 打标签
docker tag robocup_ur5e/brain:latest $DOCKER_USER/robocup_brain:latest
docker tag robocup_ur5e/perception_yolo:latest $DOCKER_USER/perception_yolo:latest
docker tag robocup_ur5e/perception_grasp:latest $DOCKER_USER/perception_grasp:latest

# 推送（这会花一些时间，镜像较大）
docker push $DOCKER_USER/robocup_brain:latest
docker push $DOCKER_USER/perception_yolo:latest
docker push $DOCKER_USER/perception_grasp:latest
```

### 步骤 5: 修改 docker-compose.yml
```yaml
services:
  robocup_brain:
    image: YOUR_DOCKERHUB_USERNAME/robocup_brain:latest
    # build: ...  # 注释掉这些行
    # context: .
    # dockerfile: docker/Dockerfile.brain
    # ... 其余配置保持不变

  perception_yolo:
    image: YOUR_DOCKERHUB_USERNAME/perception_yolo:latest
    # build: ...

  perception_grasp:
    image: YOUR_DOCKERHUB_USERNAME/perception_grasp:latest
    # build: ...
```

### 步骤 6: 提交更改
```bash
git add docker-compose.yml
git commit -m "Update: Use pre-built Docker images from Docker Hub"
git push
```

### 步骤 7: 更新邮件
告诉团队：
```
3. 启动环境:
   docker-compose pull  # 拉取预构建镜像（推荐）
   docker-compose up -d

   # 或本地构建（如果需要修改 Dockerfile）
   docker-compose build
   docker-compose up -d
```

---

## 快速命令参考

### 启动/停止
```bash
docker-compose up -d      # 启动所有容器
docker-compose down       # 停止所有容器
docker-compose restart    # 重启所有容器
docker-compose ps         # 查看状态
```

### 查看日志
```bash
docker-compose logs -f                # 所有日志
docker-compose logs -f robocup_brain  # 单个容器
```

### 进入容器
```bash
docker exec -it robocup_brain bash
docker exec -it perception_yolo bash
docker exec -it perception_grasp bash
```

### 重新构建
```bash
docker-compose build robocup_brain   # 单个
docker-compose build                 # 全部
docker-compose build --no-cache      # 强制重新构建
```

---

## 故障排除

### 问题 1: GPU 不可用
```bash
# 检查 NVIDIA Docker
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi

# 如果失败，安装 nvidia-docker2
sudo apt update
sudo apt install nvidia-docker2
sudo systemctl restart docker
```

### 问题 2: 构建失败
```bash
# 查看详细日志
docker-compose build --no-cache --progress=plain

# 检查磁盘空间
df -h

# 清理旧镜像
docker system prune -a
```

### 问题 3: 容器启动失败
```bash
# 查看具体错误
docker-compose logs <服务名>

# 检查端口占用
sudo netstat -tulpn | grep LISTEN
```

---

## 验证成功的标志 ✅

- [ ] `docker-compose ps` 显示 3 个容器都是 "Up" 状态
- [ ] `docker exec perception_yolo nvidia-smi` 显示 GPU 信息
- [ ] `docker exec perception_grasp nvidia-smi` 显示 GPU 信息
- [ ] 所有容器的日志没有严重错误
- [ ] 代码已推送到 GitHub
- [ ] 团队成员已被添加为协作者
- [ ] （可选）镜像已推送到 Docker Hub

---

## 预计时间表

| 阶段 | 任务 | 时间 |
|------|------|------|
| 阶段 1 | 本地构建和验证 | 10-30 分钟 |
| 阶段 2 | 推送到 GitHub | 5 分钟 |
| 阶段 3 | 通知团队 | 5 分钟 |
| 阶段 4 | 推送到 Docker Hub (可选) | 20-60 分钟 |
| **总计** | | **40-100 分钟** |

---

## 下一步

构建完成后，您可以：
1. ✅ 让团队成员开始并行开发
2. ✅ 您自己完善 `robocup_brain` 的行为树逻辑
3. ✅ 在 GitHub Issues 中分配具体任务

---

需要帮助？请查看:
- `README.md`: 完整文档
- `LEADER_WORKFLOW.md`: Leader 详细工作流
- `CONTRIBUTING.md`: 团队开发指南
