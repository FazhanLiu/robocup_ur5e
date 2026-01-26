# 🚀 Quick Reference - RoboCup UR5e

## ⚡ 常用命令

### 系统启动
```bash
./scripts/start.sh              # 启动所有服务
./scripts/status.sh             # 检查系统状态
./scripts/check_running.sh      # 查看运行日志
```

### 开发流程
```bash
# 1. 下载模型 (首次)
./scripts/download_models.sh

# 2. 构建镜像 (首次或Dockerfile改动后)
./scripts/rebuild_all.sh

# 3. 启动系统
./scripts/start.sh

# 4. 修改代码后重启容器
docker-compose restart <service>

# 5. 查看日志
docker-compose logs -f <service>
```

### 容器操作
```bash
# 进入容器
docker-compose exec brain bash
docker-compose exec perception_yolo bash
docker-compose exec perception_grasp bash

# 重启服务
docker-compose restart brain
docker-compose restart perception_yolo

# 停止所有服务
docker-compose down

# 查看容器状态
docker-compose ps
```

### ROS命令 (在容器内)
```bash
# 进入容器
docker-compose exec brain bash

# 激活ROS环境
source /workspace/devel/setup.bash

# 查看话题
rostopic list
rostopic echo /perception/detected_objects

# 查看节点
rosnode list

# 运行节点
rosrun robocup_brain brain_node.py
```

---

## 📂 项目结构速查

```
robocup_ur5e/
├── docs/                       # 📚 所有文档
│   ├── SETUP_GUIDE.md         # 安装指南 ⭐
│   ├── TEAM_README.md         # 团队任务 ⭐
│   └── MODELS_AND_DATASETS.md # 模型下载 ⭐
│
├── scripts/                    # 🔧 所有脚本
│   ├── start.sh               # 启动系统
│   ├── download_models.sh     # 下载模型
│   └── rebuild_all.sh         # 重新构建
│
├── weights/                    # 🤖 模型权重
│   ├── yolo/                  # YOLO模型
│   └── graspnet/              # 抓取模型
│
├── data/                       # 📊 数据集
│   ├── datasets/              # 训练数据
│   └── ycb_objects/           # YCB物体
│
└── src/                        # 📦 ROS包
    ├── robocup_brain/         # 系统FSM (Suhang)
    ├── motion_control/        # 运动控制 (Jiaxin)
    ├── path_planning/         # 路径规划 (Sarvin & Chang)
    ├── perception_yolo/       # 物体检测 (Fazhan & Ruiyi)
    └── perception_grasp/      # 抓取估计 (Muye)
```

---

## 🎯 团队成员任务

| 成员 | 包 | 文件 |
|------|-----|------|
| **Suhang** | robocup_brain | `src/robocup_brain/nodes/brain_node.py` |
| **Jiaxin** | motion_control | `src/motion_control/nodes/motion_control_node.py` |
| **Sarvin & Chang** | path_planning | `src/path_planning/nodes/path_planner_node.py` |
| **Fazhan & Ruiyi** | perception_yolo | `src/perception_yolo/nodes/yolo_detector_node.py` |
| **Muye** | perception_grasp | `src/perception_grasp/nodes/grasp_estimator_node.py` |

**详细任务**: 见 `docs/TEAM_README.md`

---

## 🐛 常见问题

### 容器启动失败
```bash
# 检查镜像
./scripts/status.sh

# 重新构建
./scripts/rebuild_all.sh

# 查看错误日志
docker-compose logs brain
```

### ROS Master连接失败
```bash
# 检查.env配置
cat .env

# 确认VM运行中
ping 192.168.56.101

# 检查端口
nc -zv 192.168.56.101 11311
```

### 模型文件缺失
```bash
# 下载所有模型
./scripts/download_models.sh

# 检查模型文件
ls -lh weights/yolo/
ls -lh weights/graspnet/
```

### GPU不可用
```bash
# 检查NVIDIA驱动
nvidia-smi

# 检查Docker GPU支持
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi

# CPU fallback会自动启用
```

---

## 📖 完整文档

- **安装**: `docs/SETUP_GUIDE.md`
- **开发**: `docs/TEAM_README.md`
- **模型**: `docs/MODELS_AND_DATASETS.md`
- **贡献**: `docs/CONTRIBUTING.md`
- **依赖**: `docs/DEPENDENCIES.md`

---

## 📞 联系方式

- **系统架构**: Suhang Xia - suhang@robocup.org
- **GitHub Issues**: https://github.com/SuhangXia/robocup_ur5e/issues

---

**Last Updated**: January 26, 2026
