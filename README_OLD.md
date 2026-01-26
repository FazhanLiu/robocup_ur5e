# RoboCup UR5e Workspace

**模块化 ROS 1 Noetic Monorepo for RoboCup UR5e Project**

---

## 📋 系统架构

### 硬件环境
- **主机**: Ubuntu 22.04 (物理机，Suhang Xia 的主要计算节点)
- **主 GPU**: NVIDIA RTX 5070 Ti (140W 解锁)
- **仿真环境**: `arm_gazebo` (黑盒) 运行在 VirtualBox VM 的 Docker 容器中
- **团队硬件**: Mac (M-chip), Windows, NVIDIA 30/40 系列 GPU

### 软件架构
```
┌─────────────────────────────────────────────────────────┐
│              Ubuntu 22.04 物理主机                        │
│  ┌───────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ robocup_brain │  │ perception   │  │ perception   │ │
│  │ (行为树)      │  │ _yolo        │  │ _grasp       │ │
│  │ CPU only     │  │ CUDA 12.0    │  │ CUDA 11.3    │ │
│  └───────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
│          │                 │                  │          │
│          └─────────────────┴──────────────────┘          │
│                      │ (ROS Topics)                      │
│                      │                                   │
│          ┌───────────▼───────────┐                       │
│          │    common_msgs        │                       │
│          │ (接口契约)             │                       │
│          └───────────────────────┘                       │
└────────────────────┬────────────────────────────────────┘
                     │ ROS_MASTER_URI
                     │ (network_mode: host)
                     ▼
        ┌────────────────────────────┐
        │   VirtualBox VM            │
        │   ┌────────────────────┐   │
        │   │ arm_gazebo         │   │
        │   │ (Docker 容器)      │   │
        │   │ MoveIt /move_group │   │
        │   └────────────────────┘   │
        │   IP: 192.168.56.101:11311 │
        └────────────────────────────┘
```

---

## 📦 模块说明

### 1. `common_msgs` (接口契约)
**强制性消息接口**，所有模块必须遵守。

#### 消息类型
- **`DetectedObject.msg`**: 检测到的物体信息
  ```
  string label                      # 物体类别
  float32 score                     # 置信度
  sensor_msgs/RegionOfInterest roi  # 图像区域
  ```

- **`GraspCandidate.msg`**: 抓取候选姿态
  ```
  geometry_msgs/PoseStamped pose  # 抓取姿态
  float32 quality                  # 质量评分
  ```

---

### 2. `robocup_brain` (Suhang Xia)
**决策中心** - 基于 `py_trees_ros` 的行为树架构。

#### 架构
```
Selector (根选择器)
├── Sequence (主序列)
│   ├── Search (搜索环境)
│   ├── Detect (等待检测结果)
│   ├── PlanGrasp (规划抓取)
│   └── ExecuteGrasp (执行抓取 - MoveIt 客户端)
└── Recovery (恢复行为)
```

#### 关键特性
- **MoveIt 客户端**: 通过 Action Client 连接到 VM 中的 `/move_group`
- **无 GPU 依赖**: CPU-only，轻量级部署
- **ROS_MASTER_URI**: 动态配置指向 VirtualBox VM

#### 运行
```bash
# Docker
docker-compose up -d robocup_brain

# Native
roslaunch robocup_brain brain.launch
```

---

### 3. `perception_yolo` (Fazhan & Ruiyi)
**物体检测模块** - YOLOv8 + CUDA 12.0。

#### 关键特性
- **多设备支持**: 
  - CUDA (NVIDIA GPU)
  - MPS (Apple M-chip)
  - CPU (回退模式)
- **自动设备检测**: `torch.cuda.is_available()`
- **实时推理**: 订阅 `/camera/color/image_raw`，发布 `/perception/detected_objects`

#### 依赖
```
torch==2.5.1
ultralytics==8.3.0
```

#### 运行
```bash
# Docker
docker-compose up -d perception_yolo

# Native
roslaunch perception_yolo yolo_detector.launch
```

---

### 4. `perception_grasp` (Muye Yuan)
**抓取姿态估计模块** - GraspNet-1Billion + CUDA 11.3。

#### 关键特性
- **点云处理**: Open3D 下采样、统计滤波
- **GraspNet 集成**: （需要手动安装 `graspnet-baseline`）
- **CUDA 11.3**: 为遗留代码兼容性

#### 依赖
```
torch==1.12.1+cu113
open3d==0.17.0
```

#### 运行
```bash
# Docker
docker-compose up -d perception_grasp

# Native
roslaunch perception_grasp grasp_estimator.launch
```

---

## 🚀 快速开始

### 方法 1: Docker Compose (推荐)

#### 1. 配置环境变量
编辑 `.env` 文件，设置 VirtualBox VM IP：
```bash
export ROS_MASTER_URI=http://192.168.56.101:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
```

#### 2. 构建镜像
```bash
# 构建所有镜像
docker-compose build

# 或单独构建
docker-compose build robocup_brain
docker-compose build perception_yolo
docker-compose build perception_grasp
```

#### 3. 启动服务
```bash
# 使用快速启动脚本
./launch.sh

# 或手动启动
docker-compose up -d
```

#### 4. 查看日志
```bash
# 所有服务
docker-compose logs -f

# 单个服务
docker-compose logs -f robocup_brain
```

#### 5. 进入容器
```bash
docker exec -it robocup_brain bash
docker exec -it perception_yolo bash
docker exec -it perception_grasp bash
```

#### 6. 停止服务
```bash
docker-compose down
```

---

### 方法 2: 原生编译

#### 1. 安装依赖
```bash
# 安装 ROS Noetic (Ubuntu 20.04)
sudo apt update
sudo apt install ros-noetic-desktop-full

# 安装 Python 依赖
pip3 install -r src/perception_yolo/requirements.txt
pip3 install -r src/perception_grasp/requirements.txt
pip3 install py_trees==2.2.3 py_trees_ros==2.2.2
```

#### 2. 编译工作区
```bash
# 使用构建脚本
./build.sh

# 或手动编译
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

#### 3. 启动节点
```bash
# 使用启动脚本
./launch_native.sh

# 或手动启动（在多个终端中）
roslaunch robocup_brain brain.launch
roslaunch perception_yolo yolo_detector.launch
roslaunch perception_grasp grasp_estimator.launch
```

---

## 🌐 网络配置

### ROS_MASTER_URI
- **默认**: `http://192.168.56.101:11311` (VirtualBox VM)
- **配置文件**: `.env`
- **Docker**: 使用 `network_mode: host` 最小化延迟

### 端口说明
- `11311`: ROS Master (在 VM 中)
- 所有 ROS 节点通过 Host 网络通信

---

## 🎯 开发工作流

### 1. 修改代码
```bash
# 编辑文件
vim src/robocup_brain/nodes/brain_node.py

# 重新编译
catkin build robocup_brain

# 或重新构建 Docker 镜像
docker-compose build robocup_brain
```

### 2. 测试
```bash
# 查看话题
rostopic list
rostopic echo /perception/detected_objects

# 查看 TF 树
rosrun tf2_tools view_frames.py

# 查看行为树状态
rostopic echo /behavior_tree/status
```

### 3. 调试
```bash
# 进入容器调试
docker exec -it perception_yolo bash
python3 -c "import torch; print(torch.cuda.is_available())"

# 查看 GPU 使用
nvidia-smi

# 查看 ROS 日志
rqt_console
```

---

## 📊 性能优化

### GPU 优化
1. **NVIDIA RTX 5070 Ti**: 
   - 设置 `NVIDIA_VISIBLE_DEVICES=all`
   - 使用 `runtime: nvidia` (Docker)

2. **多 GPU 调度**: 
   - 修改 `docker-compose.yml` 中的 `device_ids`

### 网络优化
- **Host 模式**: 避免 Docker 网络桥接开销
- **大缓冲区**: `buff_size=2**24` (16MB)

---

## 🔧 故障排除

### 1. 连接不到 ROS Master
```bash
# 检查 VM IP
ping 192.168.56.101

# 检查 ROS Master
rostopic list
```

### 2. GPU 不可用
```bash
# 检查 NVIDIA Docker
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi

# 安装 nvidia-docker2
sudo apt install nvidia-docker2
sudo systemctl restart docker
```

### 3. 消息类型未找到
```bash
# 重新编译 common_msgs
catkin build common_msgs --force-cmake
source devel/setup.bash
```

---

## 📁 目录结构

```
robocup_ur5e_ws/
├── src/
│   ├── common_msgs/               # 接口契约
│   │   ├── msg/
│   │   │   ├── DetectedObject.msg
│   │   │   └── GraspCandidate.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robocup_brain/             # 决策模块 (Suhang)
│   │   ├── nodes/
│   │   │   └── brain_node.py
│   │   ├── launch/
│   │   │   └── brain.launch
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── perception_yolo/           # 检测模块 (Fazhan & Ruiyi)
│   │   ├── nodes/
│   │   │   └── yolo_detector_node.py
│   │   ├── launch/
│   │   ├── config/
│   │   ├── requirements.txt
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── perception_grasp/          # 抓取模块 (Muye)
│       ├── nodes/
│       │   └── grasp_estimator_node.py
│       ├── launch/
│       ├── config/
│       ├── requirements.txt
│       ├── CMakeLists.txt
│       └── package.xml
├── docker/
│   ├── Dockerfile.brain
│   ├── Dockerfile.yolo
│   ├── Dockerfile.grasp
│   └── entrypoint.sh
├── docker-compose.yml
├── .env
├── launch.sh
├── launch_native.sh
├── build.sh
└── README.md
```

---

## 👥 团队成员

| 模块 | 负责人 | 环境 |
|------|--------|------|
| `robocup_brain` | Suhang Xia | CPU-only |
| `perception_yolo` | Fazhan & Ruiyi | CUDA 12.0 |
| `perception_grasp` | Muye Yuan | CUDA 11.3 |

---

## 📝 TODO

- [ ] 集成真实的 GraspNet-1Billion 模型
- [ ] 添加可视化工具（RViz 配置）
- [ ] 实现机械臂运动规划详细逻辑
- [ ] 添加单元测试
- [ ] 性能基准测试
- [ ] 文档国际化（英文版）

---

## 📄 许可证

MIT License

---

## 🙏 致谢

- ROS Noetic
- py_trees_ros
- Ultralytics YOLOv8
- GraspNet-1Billion
- Open3D
