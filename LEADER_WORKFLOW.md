# Leader 工作流程指南

## 🎯 作为 Leader 的职责

在团队成员开始工作之前，您需要：
1. ✅ 构建并测试所有 Docker 镜像
2. ✅ 验证容器间通信正常
3. ✅ （可选）将镜像推送到 Docker Hub
4. ✅ 提交代码到 GitHub
5. ✅ 编写团队使用文档

---

## 📋 步骤 1: 本地构建和测试

### 1.1 修改环境配置
```bash
# 编辑 .env 文件，设置您的 VirtualBox VM IP
vim .env

# 确保以下变量正确:
# export ROS_MASTER_URI=http://192.168.56.101:11311
# export ROS_IP=$(hostname -I | awk '{print $1}')
```

### 1.2 构建所有镜像
```bash
# 方法 1: 一次性构建所有镜像
docker-compose build

# 方法 2: 单独构建（用于调试）
docker-compose build robocup_brain
docker-compose build perception_yolo
docker-compose build perception_grasp

# 查看构建的镜像
docker images | grep robocup_ur5e
```

**预期输出:**
```
robocup_ur5e/brain            latest    <ID>    <时间>    <大小>
robocup_ur5e/perception_yolo  latest    <ID>    <时间>    <大小>
robocup_ur5e/perception_grasp latest    <ID>    <时间>    <大小>
```

### 1.3 测试 GPU 访问
```bash
# 测试 NVIDIA Docker Runtime
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi

# 如果失败，安装 nvidia-docker2:
sudo apt install nvidia-docker2
sudo systemctl restart docker
```

### 1.4 启动所有容器
```bash
# 后台启动
docker-compose up -d

# 查看状态
docker-compose ps

# 预期看到 3 个容器都是 "Up" 状态
```

### 1.5 验证容器功能

#### 检查 Brain 容器
```bash
# 进入容器
docker exec -it robocup_brain bash

# 在容器内执行
source /workspace/devel/setup.bash
rostopic list  # 应该能看到 ROS 话题

# 检查 py_trees 是否安装
python3 -c "import py_trees, py_trees_ros; print('OK')"

# 退出
exit
```

#### 检查 YOLO 容器
```bash
# 进入容器
docker exec -it perception_yolo bash

# 检查 CUDA
nvidia-smi
python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"

# 检查 ultralytics
python3 -c "from ultralytics import YOLO; print('OK')"

# 退出
exit
```

#### 检查 Grasp 容器
```bash
# 进入容器
docker exec -it perception_grasp bash

# 检查 CUDA 11.3
nvidia-smi
python3 -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.version.cuda}')"

# 检查 Open3D
python3 -c "import open3d; print('OK')"

# 退出
exit
```

### 1.6 查看日志
```bash
# 所有容器的日志
docker-compose logs

# 单个容器的实时日志
docker-compose logs -f robocup_brain
docker-compose logs -f perception_yolo
docker-compose logs -f perception_grasp
```

### 1.7 停止容器
```bash
docker-compose down
```

---

## 📦 步骤 2: （可选）推送镜像到 Docker Hub

如果您想让团队成员直接拉取预构建的镜像（不需要本地构建），可以推送到 Docker Hub：

### 2.1 创建 Docker Hub 账号
访问 https://hub.docker.com/ 注册账号（假设用户名为 `robocup_team`）

### 2.2 登录 Docker Hub
```bash
docker login
# 输入用户名和密码
```

### 2.3 给镜像打标签
```bash
# 方法 1: 在 docker-compose.yml 中修改 image 名称
# 将 robocup_ur5e/brain 改为 robocup_team/brain

# 方法 2: 手动打标签
docker tag robocup_ur5e/brain:latest robocup_team/brain:latest
docker tag robocup_ur5e/perception_yolo:latest robocup_team/perception_yolo:latest
docker tag robocup_ur5e/perception_grasp:latest robocup_team/perception_grasp:latest
```

### 2.4 推送镜像
```bash
docker push robocup_team/brain:latest
docker push robocup_team/perception_yolo:latest
docker push robocup_team/perception_grasp:latest

# 注意：感知镜像较大（~5-10GB），上传需要时间
```

### 2.5 更新 docker-compose.yml
```yaml
services:
  robocup_brain:
    image: robocup_team/brain:latest  # 添加这行
    # build: ...  # 注释掉 build，让团队直接拉取镜像
```

---

## 🐙 步骤 3: 提交到 GitHub

### 3.1 初始化 Git 仓库
```bash
cd /home/suhang/robocup_ur5e_ws
git init
```

### 3.2 创建 .gitignore（已存在）
```bash
# 已经创建，确保以下内容被忽略：
# - devel/
# - build/
# - *.pyc
# - models/*.pt
# - graspnet_checkpoints/*.tar
```

### 3.3 第一次提交
```bash
git add .
git commit -m "Initial commit: RoboCup UR5e Monorepo

- Add common_msgs interface contract
- Add robocup_brain (behavior tree)
- Add perception_yolo (CUDA 12.0)
- Add perception_grasp (CUDA 11.3)
- Add Docker orchestration
- Add documentation"
```

### 3.4 创建 GitHub 仓库
1. 访问 https://github.com/new
2. 仓库名称: `robocup-ur5e-ws`
3. 选择 Private（如果不想公开）
4. 不要初始化 README（本地已有）

### 3.5 推送到 GitHub
```bash
# 添加远程仓库（替换为您的 GitHub 用户名）
git remote add origin https://github.com/YOUR_USERNAME/robocup-ur5e-ws.git

# 推送
git branch -M main
git push -u origin main
```

---

## 👥 步骤 4: 邀请团队成员

### 4.1 在 GitHub 上添加协作者
1. 进入仓库设置 → Collaborators
2. 添加团队成员:
   - Fazhan (fazhan@...)
   - Ruiyi (ruiyi@...)
   - Muye Yuan (muye@...)

### 4.2 发送使用说明
给团队发送以下信息：

---

**📧 给团队的邮件模板:**

```
主题: RoboCup UR5e 工作区已就绪

大家好！

我已经搭建好了 RoboCup UR5e 的开发环境，请按照以下步骤开始工作：

1. 克隆仓库:
   git clone https://github.com/YOUR_USERNAME/robocup-ur5e-ws.git
   cd robocup-ur5e-ws

2. 阅读文档:
   - README.md: 完整系统说明
   - QUICKSTART.md: 快速参考
   - CONTRIBUTING.md: 开发指南

3. 启动系统:
   # 方法 1: 如果我已经推送镜像到 Docker Hub
   docker-compose pull  # 拉取预构建镜像
   docker-compose up -d

   # 方法 2: 本地构建
   docker-compose build
   docker-compose up -d

4. 各模块负责人:
   - Fazhan & Ruiyi: perception_yolo (物体检测)
   - Muye Yuan: perception_grasp (抓取估计)

5. 开发流程:
   - 修改代码后运行: docker-compose build <服务名>
   - 重启容器: docker-compose restart <服务名>
   - 查看日志: docker-compose logs -f <服务名>

有问题随时联系我！

Best,
Suhang
```

---

## 🔍 步骤 5: 验证团队成员可以运行

### 5.1 模拟团队成员的环境
```bash
# 在另一个目录测试
cd /tmp
git clone https://github.com/YOUR_USERNAME/robocup-ur5e-ws.git test_ws
cd test_ws

# 如果推送了镜像到 Docker Hub
docker-compose pull
docker-compose up -d

# 如果没有推送镜像
docker-compose build
docker-compose up -d
```

### 5.2 检查是否成功
```bash
docker-compose ps  # 应该看到 3 个容器运行
docker-compose logs  # 查看是否有错误
```

---

## 📝 步骤 6: 创建 Issues 分配任务

在 GitHub 仓库创建 Issues:

### Issue 1: YOLO 模型集成
```
标题: [perception_yolo] 集成和优化 YOLO 模型
分配给: Fazhan, Ruiyi

任务:
- [ ] 下载/训练适合 RoboCup 的 YOLO 模型
- [ ] 将模型放入 models/ 目录
- [ ] 调整置信度阈值 (config/yolo_config.yaml)
- [ ] 测试检测精度和速度
- [ ] 更新文档

环境: perception_yolo 容器 (CUDA 12.0)
```

### Issue 2: GraspNet 集成
```
标题: [perception_grasp] 集成 GraspNet-Baseline
分配给: Muye Yuan

任务:
- [ ] 克隆 graspnet-baseline 仓库
- [ ] 下载预训练权重到 graspnet_checkpoints/
- [ ] 替换 grasp_estimator_node.py 中的占位符代码
- [ ] 测试抓取质量评分
- [ ] 调整点云预处理参数

环境: perception_grasp 容器 (CUDA 11.3)
```

### Issue 3: 行为树完善
```
标题: [robocup_brain] 完善行为树逻辑
分配给: Suhang Xia (你自己)

任务:
- [ ] 实现 SearchBehavior 的具体搜索策略
- [ ] 完善 ExecuteGraspBehavior 的 MoveIt 请求
- [ ] 添加错误处理和超时机制
- [ ] 实现 RecoveryBehavior 的恢复逻辑
- [ ] 添加单元测试

环境: robocup_brain 容器
```

---

## ✅ 检查清单

在通知团队之前，确保：

- [ ] 所有 Docker 镜像构建成功
- [ ] 容器可以正常启动和通信
- [ ] GPU 访问正常（nvidia-smi 在容器内可用）
- [ ] ROS_MASTER_URI 配置正确
- [ ] 文档完整（README.md, QUICKSTART.md, CONTRIBUTING.md）
- [ ] .gitignore 正确（不提交编译产物和大文件）
- [ ] 代码已推送到 GitHub
- [ ] 团队成员已被添加为协作者
- [ ] （可选）Docker 镜像已推送到 Docker Hub

---

## 🚨 常见问题处理

### 构建失败
```bash
# 查看详细构建日志
docker-compose build --no-cache <服务名>

# 单独构建并查看错误
docker build -f docker/Dockerfile.yolo -t test_yolo .
```

### 容器启动失败
```bash
# 查看容器日志
docker logs robocup_brain

# 进入容器调试
docker run -it --rm robocup_ur5e/brain:latest bash
```

### GPU 不可用
```bash
# 确保安装了 nvidia-docker2
sudo apt install nvidia-docker2
sudo systemctl restart docker

# 测试
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi
```

---

完成上述步骤后，您的团队就可以开始并行开发了！🎉
