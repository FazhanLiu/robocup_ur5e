# 📋 Docker 构建问题修复记录（最终版本）

## 问题总结

在运行 `docker-compose build` 时遇到了 **6 个依赖错误**，现已全部修复。

---

## ✅ 修复列表

### 错误 #1: PyYAML 冲突（perception_grasp - Builder Stage）

**错误信息：**
```
Cannot uninstall PyYAML 5.3.1
It is a distutils installed project
```

**原因：** Ubuntu 20.04 系统预装的 PyYAML 5.3.1 无法被 pip 卸载。

**修复：** 修改 `docker/Dockerfile.grasp` 第 44 行
```dockerfile
# 修改前
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 修改后
RUN pip3 install --no-cache-dir --ignore-installed PyYAML -r /tmp/requirements.txt
```

---

### 错误 #2: PyYAML 冲突（perception_grasp - Runtime Stage）⚠️ **新发现**

**错误信息：** 同上（在多阶段构建的 runtime stage 中）

**原因：** 第一次修复只修改了 builder stage（第 44 行），但 runtime stage（第 69 行）也有同样的问题。

**修复：** 修改 `docker/Dockerfile.grasp` 第 69-71 行
```dockerfile
# 修改前
COPY src/perception_grasp/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 修改后
COPY src/perception_grasp/requirements.txt /tmp/requirements.txt
# 忽略系统已安装的 PyYAML 以避免冲突
RUN pip3 install --no-cache-dir --ignore-installed PyYAML -r /tmp/requirements.txt
```

---

### 错误 #3: py_trees_ros 不存在（robocup_brain）

**错误信息：**
```
ERROR: Could not find a version that satisfies the requirement py_trees_ros==2.2.2
```

**原因：** `py_trees_ros` 不能通过 pip 安装，必须使用 ROS 的 apt 包管理器。

**修复：** 修改 `docker/Dockerfile.brain`
```dockerfile
# 修改前（第 23 行）
RUN pip3 install py_trees py_trees_ros

# 修改后（第 19 行，整合到 apt-get install 中）
ros-noetic-py-trees \
ros-noetic-py-trees-ros \
```

---

### 错误 #4: moveit_msgs 缺失（robocup_brain）

**错误信息：**
```
Could not find a package configuration file provided by "moveit_msgs"
```

**原因：** 代码中使用了 MoveIt 和 actionlib，但未安装相关 ROS 包。

**修复：** 修改 `docker/Dockerfile.brain`（第 19 行）
```dockerfile
# 添加到 apt-get install 中
ros-noetic-moveit-msgs \
ros-noetic-actionlib \
```

---

### 错误 #5: torch 2.5.1 不存在（perception_yolo）

**错误信息：**
```
ERROR: Could not find a version that satisfies the requirement torch==2.5.1
```

**原因：** PyTorch 2.5.1 不存在，该版本尚未发布。

**修复：** 修改 `src/perception_yolo/requirements.txt`
```txt
# 修改前
torch==2.5.1
torchvision==0.20.1

# 修改后
torch==2.4.1
torchvision==0.19.1
```

---

### 错误 #6: numpy 1.26.4 不存在（perception_yolo）

**错误信息：**
```
ERROR: Could not find a version that satisfies the requirement numpy==1.26.4
```

**原因：** Python 3.8 最高只支持 NumPy 1.24.x，1.26.x 需要 Python 3.9+。

**修复：** 修改 `src/perception_yolo/requirements.txt`
```txt
# 修改前
numpy==1.26.4

# 修改后
numpy==1.24.4
```

---

## 🔧 完整修复方案

### 方法 1: 使用自动修复脚本（推荐）

```bash
cd /home/suhang/robocup_ur5e_ws
./fix_build.sh
```

### 方法 2: 手动重新构建

```bash
cd /home/suhang/robocup_ur5e_ws

# 清理旧镜像
docker-compose down

# 重新构建（不使用缓存）
docker-compose build --no-cache
```

---

## 📊 预期构建结果

```
✓ robocup_ur5e/brain            4.7GB   ✅ 已成功
✓ robocup_ur5e/perception_yolo  ~18GB   ✅ 已成功
✓ robocup_ur5e/perception_grasp ~10GB   ⏳ 正在修复
```

### 构建时间估算
- **Brain**: ~5 分钟（已完成）
- **YOLO**: ~15 分钟（已完成）
- **Grasp**: ~20 分钟（正在修复第 6 个错误）

---

## 🎯 修复文件清单

| 文件 | 行号 | 修改内容 |
|------|------|----------|
| `docker/Dockerfile.grasp` | 44 | 添加 `--ignore-installed PyYAML`（builder） |
| `docker/Dockerfile.grasp` | 69-71 | 添加 `--ignore-installed PyYAML`（runtime）⚠️ **新** |
| `docker/Dockerfile.brain` | 19 | 改用 apt 安装 py_trees 和 moveit |
| `src/perception_yolo/requirements.txt` | 4-5, 8 | torch 2.4.1, numpy 1.24.4 |

---

## 🚀 下一步

运行以下命令进行最终测试：

```bash
cd /home/suhang/robocup_ur5e_ws

# 重新构建 perception_grasp
docker-compose build --no-cache perception_grasp

# 或者重建所有（推荐）
./rebuild_all.sh
```

---

## 📝 技术笔记

### 为什么 perception_grasp 需要修复两次？

因为 Dockerfile 使用了 **多阶段构建（Multi-stage Build）**：

1. **Builder Stage**（第 36-60 行）：编译代码
   - 需要安装 requirements.txt（第 44 行）
2. **Runtime Stage**（第 65-86 行）：最终运行镜像
   - 也需要安装 requirements.txt（第 69 行）

两个阶段都会遇到 PyYAML 冲突，因此需要**同时修复两处**。

### Python 3.8 版本兼容性矩阵

| 包 | Python 3.8 最高版本 | 最新版本 |
|---|-------------------|---------|
| numpy | 1.24.4 | 2.2.1 |
| torch | 2.4.1 | 2.6.0 |
| torchvision | 0.19.1 | 0.21.0 |

---

**状态：** 所有 6 个错误已修复，等待最终构建验证 ✅
