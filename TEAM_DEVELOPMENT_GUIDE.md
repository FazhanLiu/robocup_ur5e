# 🎯 容器运行状态说明与团队开发指南

## ✅ 当前系统状态

从您的终端日志确认：

```
perception_grasp   /entrypoint.sh roslaunch p ...   Up           
perception_yolo    /entrypoint.sh roslaunch p ...   Up           
robocup_brain      /entrypoint.sh roslaunch r ...   Up  
```

**所有 3 个容器已成功启动！** ✅

---

## 🔍 **Q1: 如何确认容器真的跑起来了？**

### 方法 1: 使用检查脚本（推荐）

```bash
cd /home/suhang/robocup_ur5e_ws
./check_running.sh
```

这个脚本会显示：
- ✅ 容器状态
- ✅ 容器日志
- ✅ ROS 节点列表
- ✅ ROS 话题列表

### 方法 2: 手动检查

```bash
# 1. 查看容器状态
docker-compose ps

# 2. 查看实时日志
docker-compose logs -f

# 3. 查看单个容器日志
docker-compose logs -f brain

# 4. 进入容器查看
docker-compose exec brain bash
source /workspace/devel/setup.bash
rostopic list
rosnode list
```

---

## 📦 **Q2: 容器里有代码吗？还是只有依赖？**

### ✅ **容器里既有依赖，也有代码骨架！**

每个容器都包含：

### 1️⃣ **robocup_brain 容器** (Suhang 负责)

**已有内容：**
```
/workspace/src/robocup_brain/
├── nodes/brain_node.py          ✅ 完整的行为树骨架（233 行）
├── launch/brain.launch          ✅ ROS 启动文件
├── package.xml                  ✅ 包配置
└── CMakeLists.txt               ✅ 编译配置
```

**代码骨架包含：**
- ✅ py_trees_ros 行为树结构
- ✅ 5 个行为类（Search, Detect, PlanGrasp, ExecuteGrasp, Recovery）
- ✅ MoveIt 客户端集成
- ✅ ROS 订阅者和发布者
- ✅ TODO 标记，提示需要实现的部分

**需要开发的部分（已有 TODO 标记）：**
```python
# TODO: 实现搜索逻辑（例如：发送关节目标或视觉扫描指令）
# TODO: 实现抓取规划逻辑（选择最佳抓取候选）
# TODO: 根据实际返回值调整逻辑（检查抓取是否成功）
# TODO: 实现恢复策略（例如：调整位置、重新扫描）
```

---

### 2️⃣ **perception_yolo 容器** (Fazhan & Ruiyi 负责)

**已有内容：**
```
/workspace/src/perception_yolo/
├── nodes/yolo_detector_node.py  ✅ YOLO 检测节点骨架（169 行）
├── config/yolo_config.yaml      ✅ 配置文件
├── launch/yolo_detector.launch  ✅ ROS 启动文件
├── requirements.txt             ✅ Python 依赖
├── package.xml                  ✅ 包配置
└── CMakeLists.txt               ✅ 编译配置
```

**代码骨架包含：**
- ✅ 设备选择（CUDA 12.0 / MPS / CPU 自动回退）
- ✅ YOLOv8 模型加载框架
- ✅ ROS 图像订阅器
- ✅ DetectedObject 消息发布器
- ✅ 完整的推理流程框架
- ✅ TODO 标记，提示需要实现的部分

**需要开发的部分（已有 TODO 标记）：**
```python
# TODO: 下载并集成 YOLOv8 训练好的权重
# TODO: 根据实际类别映射调整
# TODO: 实现更复杂的检测逻辑（例如：非极大值抑制、多尺度检测）
# TODO: 添加结果可视化发布
```

---

### 3️⃣ **perception_grasp 容器** (Muye Yuan 负责)

**已有内容：**
```
/workspace/src/perception_grasp/
├── nodes/grasp_estimator_node.py  ✅ GraspNet 节点骨架（226 行）
├── config/grasp_config.yaml       ✅ 配置文件
├── launch/grasp_estimator.launch  ✅ ROS 启动文件
├── requirements.txt               ✅ Python 依赖（CUDA 11.3）
├── package.xml                    ✅ 包配置
└── CMakeLists.txt                 ✅ 编译配置
```

**代码骨架包含：**
- ✅ CUDA 11.3 设备检查
- ✅ GraspNet 模型加载框架
- ✅ 点云数据订阅器
- ✅ Open3D 点云处理框架
- ✅ GraspCandidate 消息发布器
- ✅ TODO 标记，提示需要实现的部分

**需要开发的部分（已有 TODO 标记）：**
```python
# TODO: 下载并集成 GraspNet 预训练模型
# TODO: 实现 GraspNet 推理管道（点云预处理、抓取检测、后处理）
# TODO: 根据物体类别或场景优化抓取候选
# TODO: 添加碰撞检测或可达性分析
```

---

## 👥 **Q3: 团队成员如何开发算法？**

### ✅ **完全正确！每个成员按照对应的容器开发算法**

---

## 📝 **团队开发流程**

### **Step 1: 成员克隆仓库（您先推送到 GitHub）**

```bash
# Leader（您）先推送
cd /home/suhang/robocup_ur5e_ws
git init
git add .
git commit -m "feat: initial ROS Noetic monorepo with Docker"
git remote add origin https://github.com/your-username/robocup_ur5e.git
git push -u origin main

# 团队成员克隆
git clone https://github.com/your-username/robocup_ur5e.git
cd robocup_ur5e
```

---

### **Step 2: 每个成员开发自己的算法**

#### **Suhang（您）- robocup_brain**

**开发位置：**
```bash
src/robocup_brain/nodes/brain_node.py
```

**开发任务：**
1. 实现 `SearchBehavior` 的搜索逻辑
2. 实现 `PlanGraspBehavior` 的抓取选择算法
3. 实现 `ExecuteGraspBehavior` 的 MoveIt 调用逻辑
4. 实现 `RecoveryBehavior` 的错误恢复策略
5. 调整行为树结构（添加更多行为节点）

**测试方式：**
```bash
# 方法 1: 在容器内测试
docker-compose exec brain bash
source /workspace/devel/setup.bash
rosrun robocup_brain brain_node.py

# 方法 2: 重启容器测试
docker-compose restart brain
docker-compose logs -f brain
```

---

#### **Fazhan & Ruiyi - perception_yolo**

**开发位置：**
```bash
src/perception_yolo/nodes/yolo_detector_node.py
src/perception_yolo/config/yolo_config.yaml
```

**开发任务：**
1. 下载 YOLOv8 预训练权重（如 `yolov8n.pt`）
2. 实现 `_load_model()` 中的模型加载逻辑
3. 在 `image_callback()` 中完善检测推理
4. 调整类别映射（根据 RoboCup 物体）
5. 添加可视化发布（可选）

**测试方式：**
```bash
# 方法 1: 在容器内测试
docker-compose exec perception_yolo bash
source /workspace/devel/setup.bash
rosrun perception_yolo yolo_detector_node.py

# 方法 2: 查看检测结果
docker-compose exec brain bash
source /workspace/devel/setup.bash
rostopic echo /perception/detected_objects
```

---

#### **Muye Yuan - perception_grasp**

**开发位置：**
```bash
src/perception_grasp/nodes/grasp_estimator_node.py
src/perception_grasp/config/grasp_config.yaml
```

**开发任务：**
1. 下载 GraspNet-1Billion 预训练模型
2. 实现 `_load_graspnet_model()` 中的模型加载
3. 在 `pointcloud_callback()` 中完善抓取检测
4. 实现点云预处理（滤波、裁剪）
5. 调整抓取质量评分算法

**测试方式：**
```bash
# 方法 1: 在容器内测试
docker-compose exec perception_grasp bash
source /workspace/devel/setup.bash
rosrun perception_grasp grasp_estimator_node.py

# 方法 2: 查看抓取候选
docker-compose exec brain bash
source /workspace/devel/setup.bash
rostopic echo /perception/grasp_candidates
```

---

### **Step 3: 开发时的工作流程**

#### **本地开发（推荐）**

```bash
# 1. 在宿主机修改代码
cd /home/suhang/robocup_ur5e_ws/src/robocup_brain
vim nodes/brain_node.py

# 2. 重启容器以应用更改（因为代码被挂载到容器）
docker-compose restart brain

# 3. 查看日志验证
docker-compose logs -f brain
```

**为什么可以这样做？**

因为 `docker-compose.yml` 中已配置了卷挂载：

```yaml
volumes:
  - ./src:/workspace/src:ro  # 源码只读挂载
```

这意味着：
- ✅ 宿主机修改代码 → 容器内立即生效
- ✅ 不需要重新构建镜像
- ✅ 只需重启容器即可

---

#### **容器内开发（不推荐，但可以临时调试）**

```bash
# 1. 进入容器
docker-compose exec brain bash

# 2. 修改代码
cd /workspace/src/robocup_brain/nodes
vim brain_node.py

# 3. 测试运行
source /workspace/devel/setup.bash
rosrun robocup_brain brain_node.py
```

**注意：** 容器内的修改会在容器重启后**丢失**，所以最终要复制回宿主机。

---

### **Step 4: 提交代码**

```bash
# 1. 修改完成后，在宿主机提交
cd /home/suhang/robocup_ur5e_ws
git add src/robocup_brain/
git commit -m "feat(brain): implement search behavior"
git push origin main

# 2. 其他成员拉取更新
git pull origin main

# 3. 重启容器应用更改
docker-compose restart
```

---

## 🎯 **典型的团队协作场景**

### 场景 1: Fazhan 完成了 YOLO 检测

```bash
# Fazhan 在他的电脑上：
cd robocup_ur5e_ws
vim src/perception_yolo/nodes/yolo_detector_node.py
# ... 实现检测逻辑 ...

# 测试
docker-compose restart perception_yolo
docker-compose logs -f perception_yolo

# 提交
git add src/perception_yolo/
git commit -m "feat(yolo): implement YOLOv8 detection"
git push origin main
```

```bash
# Suhang（您）在您的电脑上：
cd /home/suhang/robocup_ur5e_ws
git pull origin main  # 拉取 Fazhan 的更新

# 重启容器以使用新代码
docker-compose restart perception_yolo

# 在 brain 中测试接收检测结果
docker-compose exec brain bash
source /workspace/devel/setup.bash
rostopic echo /perception/detected_objects
```

---

### 场景 2: 需要修改 common_msgs

```bash
# Leader（您）修改消息定义
vim src/common_msgs/msg/DetectedObject.msg
# 添加新字段，例如：
# float32 distance

# 重新构建镜像（因为消息定义需要编译）
docker-compose build --no-cache

# 通知团队成员
git add src/common_msgs/
git commit -m "feat(common_msgs): add distance field to DetectedObject"
git push origin main

# 团队成员拉取并重建
git pull origin main
docker-compose build --no-cache
```

---

## 📚 **开发参考文档**

每个成员应该阅读：

| 成员 | 相关文档 |
|------|----------|
| 所有人 | `CONTRIBUTING.md` - 开发规范 |
| 所有人 | `QUICKSTART.md` - 命令速查 |
| Suhang | `src/robocup_brain/README.md` (如果创建) |
| Fazhan & Ruiyi | `src/perception_yolo/README.md` (可创建) |
| Muye Yuan | `src/perception_grasp/README.md` (可创建) |

---

## 🐛 **常见问题**

### Q: 修改代码后容器没反应？

**A:** 重启容器：
```bash
docker-compose restart brain
```

### Q: 需要安装新的 Python 包？

**A:** 修改 `requirements.txt` 并重建镜像：
```bash
vim src/perception_yolo/requirements.txt
docker-compose build --no-cache perception_yolo
```

### Q: 如何调试代码？

**A:** 进入容器使用 `python3` 交互式调试：
```bash
docker-compose exec brain bash
source /workspace/devel/setup.bash
python3 -m pdb /workspace/src/robocup_brain/nodes/brain_node.py
```

---

## ✅ **总结**

### **当前状态：**
- ✅ 所有 3 个容器已启动
- ✅ 每个容器都有完整的代码骨架（200+ 行）
- ✅ 所有依赖已安装
- ✅ ROS 节点框架已就绪

### **下一步：**
1. **您（Leader）:** 推送代码到 GitHub
2. **团队成员:** 克隆仓库并启动容器
3. **各自开发:** 按照 TODO 标记实现算法
4. **联调测试:** 确保 ROS 话题通信正常

### **检查运行状态：**
```bash
./check_running.sh
```

**所有容器都在运行，等待您和团队成员填充算法逻辑！** 🚀
