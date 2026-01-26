# RoboCup UR5e 快速参考卡

## 🚀 启动命令

### Docker 方式（推荐）
```bash
# 1. 配置环境变量（首次运行）
vim .env  # 修改 ROS_MASTER_URI 为你的 VM IP

# 2. 构建镜像
docker-compose build

# 3. 启动所有服务
./launch.sh
# 或
docker-compose up -d

# 4. 查看日志
docker-compose logs -f robocup_brain
docker-compose logs -f perception_yolo
docker-compose logs -f perception_grasp

# 5. 停止服务
docker-compose down
```

### 原生方式
```bash
# 1. 编译工作区
./build.sh

# 2. 激活环境
source devel/setup.bash

# 3. 启动节点
./launch_native.sh
```

---

## 📡 ROS 话题

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/perception/detected_objects` | `DetectedObject` | YOLO | Brain | 检测到的物体 |
| `/perception/grasp_candidates` | `GraspCandidate` | Grasp | Brain | 抓取候选姿态 |
| `/camera/color/image_raw` | `Image` | Camera | YOLO | 相机图像 |
| `/camera/depth/points` | `PointCloud2` | Camera | Grasp | 点云数据 |
| `/move_group` | `MoveGroupAction` | MoveIt (VM) | Brain | 运动规划 |

---

## 🔧 常用调试命令

```bash
# 查看所有话题
rostopic list

# 监听检测结果
rostopic echo /perception/detected_objects

# 监听抓取候选
rostopic echo /perception/grasp_candidates

# 查看消息类型
rosmsg show common_msgs/DetectedObject

# 检查 ROS 连接
rostopic list
rosnode list

# 进入容器调试
docker exec -it perception_yolo bash
docker exec -it perception_grasp bash

# 检查 GPU
nvidia-smi
python3 -c "import torch; print(torch.cuda.is_available())"
```

---

## ⚠️ 常见问题

### 1. 连接不到 ROS Master
```bash
# 检查 VM IP
ping 192.168.56.101

# 检查环境变量
echo $ROS_MASTER_URI

# 测试连接
rostopic list
```

### 2. GPU 不可用
```bash
# 检查 NVIDIA Docker
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi

# 重启 Docker
sudo systemctl restart docker
```

### 3. 消息未定义
```bash
# 重新编译 common_msgs
catkin build common_msgs --force-cmake
source devel/setup.bash
```

---

## 📊 模块职责

| 模块 | 负责人 | 输入 | 输出 | GPU |
|------|--------|------|------|-----|
| `robocup_brain` | Suhang | DetectedObject, GraspCandidate | MoveIt Goals | ❌ |
| `perception_yolo` | Fazhan & Ruiyi | Image | DetectedObject | ✅ CUDA 12.0 |
| `perception_grasp` | Muye | PointCloud2 | GraspCandidate | ✅ CUDA 11.3 |

---

## 📝 配置文件

| 文件 | 用途 |
|------|------|
| `.env` | 环境变量（ROS_MASTER_URI）|
| `docker-compose.yml` | Docker 编排 |
| `src/*/config/*.yaml` | 模块配置 |

---

## 🔄 工作流

```
1. 相机发布图像/点云
   ↓
2. YOLO 检测物体 → DetectedObject
   ↓
3. Grasp 估计抓取 → GraspCandidate
   ↓
4. Brain 决策执行 → MoveIt
```

---

## 📞 联系方式

- Brain: Suhang Xia
- YOLO: Fazhan & Ruiyi  
- Grasp: Muye Yuan

---

更多详情请参阅 `README.md`
