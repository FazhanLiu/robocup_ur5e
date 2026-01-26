# 开发指南 (Contributing Guide)

欢迎加入 RoboCup UR5e 项目！本文档将指导您如何参与开发。

---

## 🚀 快速开始

### 1. 克隆仓库
```bash
git clone https://github.com/YOUR_USERNAME/robocup-ur5e-ws.git
cd robocup-ur5e-ws
```

### 2. 配置环境
```bash
# 编辑 .env 文件（如果 Leader 没有配置好）
vim .env

# 设置您本地的 ROS_MASTER_URI
export ROS_MASTER_URI=http://192.168.56.101:11311
```

### 3. 启动开发环境

#### 方法 A: 使用预构建镜像（推荐）
```bash
# 如果 Leader 已推送镜像到 Docker Hub
docker-compose pull
docker-compose up -d
```

#### 方法 B: 本地构建
```bash
# 构建所有镜像
docker-compose build

# 或只构建您负责的模块
docker-compose build perception_yolo   # Fazhan & Ruiyi
docker-compose build perception_grasp  # Muye
```

---

## 👥 团队分工

| 模块 | 负责人 | 容器名 | 开发重点 |
|------|--------|--------|----------|
| `robocup_brain` | Suhang Xia | `robocup_brain` | 行为树逻辑、MoveIt 接口 |
| `perception_yolo` | Fazhan & Ruiyi | `perception_yolo` | YOLO 检测、模型训练 |
| `perception_grasp` | Muye Yuan | `perception_grasp` | GraspNet 集成、抓取评估 |

---

## 🛠️ 开发工作流

### 1. 创建功能分支
```bash
# 从 main 分支创建您的功能分支
git checkout -B feature/your-module-name

# 例如:
git checkout -B feature/yolo-optimization      # Fazhan & Ruiyi
git checkout -B feature/graspnet-integration   # Muye
git checkout -B feature/behavior-tree-logic    # Suhang
```

### 2. 修改代码

#### 对于 Python 代码
```bash
# 直接编辑源码
vim src/perception_yolo/nodes/yolo_detector_node.py

# 保存后，重新构建容器
docker-compose build perception_yolo

# 重启容器以应用更改
docker-compose restart perception_yolo

# 查看日志
docker-compose logs -f perception_yolo
```

#### 对于配置文件
```bash
# 编辑配置
vim src/perception_yolo/config/yolo_config.yaml

# 配置文件通过卷挂载，直接生效（无需重启）
# 如果不生效，重启容器:
docker-compose restart perception_yolo
```

### 3. 实时调试

#### 进入容器调试
```bash
# 进入您负责的容器
docker exec -it perception_yolo bash

# 在容器内:
source /workspace/devel/setup.bash
rostopic list
rostopic echo /perception/detected_objects

# 手动运行节点（用于调试）
rosrun perception_yolo yolo_detector_node.py
```

#### 使用 Python 调试器
在代码中添加断点:
```python
import pdb; pdb.set_trace()
```

然后以交互模式运行:
```bash
docker exec -it perception_yolo bash
python3 /workspace/src/perception_yolo/nodes/yolo_detector_node.py
```

### 4. 测试您的更改

#### 检查 ROS 话题
```bash
# 查看发布的消息
rostopic echo /perception/detected_objects
rostopic echo /perception/grasp_candidates

# 检查消息频率
rostopic hz /perception/detected_objects

# 查看消息内容
rosmsg show common_msgs/DetectedObject
```

#### GPU 性能测试
```bash
# 在容器内查看 GPU 使用
docker exec -it perception_yolo nvidia-smi

# 或持续监控
watch -n 1 nvidia-smi
```

### 5. 提交更改

#### 提交前检查
```bash
# 查看修改的文件
git status

# 查看具体更改
git diff

# 不要提交编译产物
# 确保 .gitignore 正确
```

#### 提交代码
```bash
# 添加修改的文件
git add src/perception_yolo/nodes/yolo_detector_node.py
git add src/perception_yolo/config/yolo_config.yaml

# 提交（使用清晰的消息）
git commit -m "[perception_yolo] 优化 YOLO 检测阈值

- 将置信度阈值从 0.5 调整到 0.6
- 添加类别过滤器
- 提升检测速度 15%"

# 推送到您的分支
git push origin feature/yolo-optimization
```

### 6. 创建 Pull Request

1. 访问 GitHub 仓库
2. 点击 "New Pull Request"
3. 选择您的分支 → `main`
4. 填写 PR 描述:

```markdown
## 概述
优化 YOLO 检测性能

## 更改内容
- 调整置信度阈值
- 添加类别过滤
- 优化推理速度

## 测试
- [x] 本地测试通过
- [x] 容器构建成功
- [x] GPU 加速正常
- [x] ROS 话题发布正常

## 性能
- 检测速度: 30 FPS → 35 FPS (+15%)
- GPU 内存: 2.1GB → 1.8GB

## 截图
（可选）添加测试截图
```

5. 请求 Leader (Suhang) Review

---

## 📦 模块开发指南

### 🎯 perception_yolo (Fazhan & Ruiyi)

#### 开发重点
1. **模型选择和训练**
   - 选择合适的 YOLO 模型（YOLOv8n/s/m）
   - 在 RoboCup 数据集上微调
   - 平衡精度和速度

2. **优化推理性能**
   - 调整输入分辨率
   - 使用 TensorRT 加速（可选）
   - 批处理优化

3. **配置参数**
   ```yaml
   # src/perception_yolo/config/yolo_config.yaml
   detection:
     confidence_threshold: 0.5  # 调整这个
     classes:
       filter: ['bottle', 'cup', 'bowl']  # 只检测相关物体
   ```

4. **测试检查清单**
   - [ ] 检测精度 > 85%
   - [ ] FPS > 25
   - [ ] GPU 内存 < 3GB
   - [ ] CPU 回退正常（Mac 队友测试）

#### 文件位置
- 主节点: `src/perception_yolo/nodes/yolo_detector_node.py`
- 配置: `src/perception_yolo/config/yolo_config.yaml`
- 依赖: `src/perception_yolo/requirements.txt`

---

### 🤖 perception_grasp (Muye Yuan)

#### 开发重点
1. **集成 GraspNet-Baseline**
   ```bash
   # 在容器内
   cd /tmp
   git clone https://github.com/graspnet/graspnet-baseline.git
   cd graspnet-baseline
   pip install -e .
   ```

2. **下载预训练模型**
   - 从 GraspNet 官方下载权重
   - 放入 `/workspace/graspnet_checkpoints/`

3. **替换占位符代码**
   找到 `grasp_estimator_node.py` 中的 TODO 注释并实现:
   ```python
   def _load_graspnet_model(self):
       from graspnetAPI import GraspNet
       model = GraspNet(checkpoint_path=self.checkpoint_path)
       model.to(self.device)
       return model
   ```

4. **调优点云预处理**
   ```yaml
   # src/perception_grasp/config/grasp_config.yaml
   pointcloud:
     voxel_size: 0.005  # 调整下采样
   ```

5. **测试检查清单**
   - [ ] 抓取候选生成成功
   - [ ] 质量评分合理（0.3-1.0）
   - [ ] 推理时间 < 2 秒
   - [ ] CUDA 11.3 兼容性

#### 文件位置
- 主节点: `src/perception_grasp/nodes/grasp_estimator_node.py`
- 配置: `src/perception_grasp/config/grasp_config.yaml`

---

### 🧠 robocup_brain (Suhang Xia)

#### 开发重点
1. **完善行为树节点**
   - `SearchBehavior`: 实现环境扫描逻辑
   - `ExecuteGraspBehavior`: 完善 MoveIt 请求
   - `RecoveryBehavior`: 失败恢复策略

2. **MoveIt 接口**
   ```python
   # 构造完整的 MoveGroupGoal
   goal = MoveGroupGoal()
   goal.request.group_name = "manipulator"
   goal.request.num_planning_attempts = 5
   # ... 填充更多字段
   ```

3. **添加状态机监控**
   - 使用 py_trees 的 Blackboard 共享状态
   - 添加日志记录
   - 实现超时和重试

---

## 🔄 持续集成建议

### 本地测试脚本
创建 `test.sh`:
```bash
#!/bin/bash
# 本地测试脚本

# 1. 构建您的模块
docker-compose build perception_yolo

# 2. 启动容器
docker-compose up -d perception_yolo

# 3. 等待启动
sleep 5

# 4. 检查容器状态
docker-compose ps | grep perception_yolo

# 5. 查看日志
docker-compose logs --tail=50 perception_yolo

# 6. 检查 ROS 话题
docker exec perception_yolo bash -c "source /workspace/devel/setup.bash && rostopic list"
```

---

## 📝 代码规范

### Python 风格
- 遵循 PEP 8
- 使用类型提示（Python 3.6+）
- 添加 docstrings

```python
def process_image(self, image: np.ndarray) -> List[DetectedObject]:
    """
    处理图像并返回检测结果
    
    Args:
        image: 输入图像 (H, W, 3)
        
    Returns:
        检测到的物体列表
    """
    pass
```

### ROS 节点规范
- 使用 `rospy.loginfo/warn/error` 记录日志
- 在 `__init__` 中初始化 ROS 接口
- 使用 `rospy.spin()` 保持节点运行

### Git 提交消息
```
[模块名] 简短描述（50字以内）

详细说明:
- 更改了什么
- 为什么更改
- 影响范围

测试: 如何测试
```

---

## 🐛 调试技巧

### 1. 查看完整日志
```bash
# 所有容器
docker-compose logs -f

# 单个容器（最近 100 行）
docker-compose logs --tail=100 perception_yolo
```

### 2. 进入容器交互调试
```bash
docker exec -it perception_yolo bash

# 测试 Python 导入
python3 -c "import torch, ultralytics"

# 手动运行节点
source /workspace/devel/setup.bash
rosrun perception_yolo yolo_detector_node.py
```

### 3. 检查 ROS 连接
```bash
# 在容器内
rostopic list
rosnode list
rosnode info /yolo_detector
```

### 4. 监控 GPU
```bash
# 主机上
watch -n 1 nvidia-smi

# 容器内
docker exec perception_yolo nvidia-smi
```

---

## 🆘 常见问题

### Q: 修改代码后没有生效？
**A:** 需要重新构建镜像
```bash
docker-compose build <服务名>
docker-compose restart <服务名>
```

### Q: 容器启动失败？
**A:** 查看日志排查
```bash
docker-compose logs <服务名>
docker logs <容器名>
```

### Q: GPU 不可用？
**A:** 检查 NVIDIA Docker
```bash
docker run --rm --gpus all nvidia/cuda:12.0.1-base-ubuntu20.04 nvidia-smi
```

### Q: ROS Master 连接不上？
**A:** 检查 `.env` 中的 `ROS_MASTER_URI`
```bash
# 测试连接
ping 192.168.56.101
rostopic list
```

### Q: 如何回滚更改？
**A:** 使用 git 恢复
```bash
git checkout -- <文件名>  # 恢复单个文件
git reset --hard HEAD     # 恢复所有未提交更改
```

---

## 📞 获取帮助

- **紧急问题**: 联系 Leader (Suhang Xia)
- **技术讨论**: 在 GitHub Issues 中讨论
- **文档问题**: 提交 PR 改进文档

---

## ✅ 提交检查清单

在提交 PR 之前，请确认:

- [ ] 代码可以正常运行
- [ ] Docker 镜像构建成功
- [ ] 添加了必要的注释和文档
- [ ] 遵循代码规范
- [ ] 测试了主要功能
- [ ] 更新了 README.md（如果需要）
- [ ] Git 提交消息清晰
- [ ] 没有提交大文件（模型、编译产物）

---

感谢您的贡献！🎉
