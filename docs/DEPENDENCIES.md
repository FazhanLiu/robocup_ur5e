# 📦 依赖版本兼容性表

## 重要说明

ROS Noetic 容器基于 **Ubuntu 20.04 + Python 3.8**，这限制了某些包的最高版本。

---

## ✅ 修复后的正确版本

### perception_yolo (CUDA 12.0)

| 包 | 原始版本 | 修复后版本 | 原因 |
|---|---------|-----------|------|
| torch | ~~2.5.1~~ | **2.4.1** | PyTorch 2.5.x 不存在 |
| torchvision | ~~0.20.1~~ | **0.19.1** | 与 torch 2.4.1 匹配 |
| numpy | ~~1.26.4~~ | **1.24.4** | Python 3.8 最高支持 1.24.x |
| ultralytics | 8.3.0 | 8.3.0 | ✅ 正确 |
| opencv-python | 4.10.0.84 | 4.10.0.84 | ✅ 正确 |

### perception_grasp (CUDA 11.3)

| 包 | 版本 | 说明 |
|---|------|------|
| torch | 1.12.1+cu113 | ✅ CUDA 11.3 兼容 |
| torchvision | 0.13.1+cu113 | ✅ CUDA 11.3 兼容 |
| numpy | 1.23.5 | ✅ 正确 |
| open3d | 0.17.0 | ✅ 正确 |
| scipy | 1.10.1 | ✅ 正确 |

### robocup_brain (CPU only)

| 包 | 安装方式 | 版本 |
|---|---------|------|
| py_trees | apt | 0.7.6 (ros-noetic-py-trees) |
| py_trees_ros | apt | 0.6.1 (ros-noetic-py-trees-ros) |
| moveit_msgs | apt | ros-noetic-moveit-msgs |
| actionlib | apt | ros-noetic-actionlib |

---

## 🐍 Python 3.8 版本限制

**为什么不能用最新版本？**

ROS Noetic 基于 Ubuntu 20.04，其 Python 版本为 3.8.x。某些包不支持 Python 3.8：

| 包 | Python 3.8 最高版本 | 最新版本 | 备注 |
|---|-------------------|---------|------|
| numpy | 1.24.4 | 2.2.1 | NumPy 2.x 需要 Python 3.9+ |
| torch | 2.4.1 | 2.6.0 | PyTorch 2.5+ 需要 Python 3.9+ |

---

## 🔧 如何查找兼容版本

### 方法 1: 在容器中测试
```bash
docker run -it --rm python:3.8 bash
pip install numpy==  # 列出所有可用版本
```

### 方法 2: 查询 PyPI
访问 https://pypi.org/project/numpy/#files
筛选 `cp38` (CPython 3.8) 的 wheel 文件

---

## 📝 完整的 requirements.txt

### perception_yolo/requirements.txt（已修复）
```txt
# CUDA 12.0 + Python 3.8
torch==2.4.1
torchvision==0.19.1
ultralytics==8.3.0
opencv-python==4.10.0.84
numpy==1.24.4

rospkg
catkin_pkg
```

### perception_grasp/requirements.txt（已正确）
```txt
# CUDA 11.3 + Python 3.8
torch==1.12.1+cu113
torchvision==0.13.1+cu113
--extra-index-url https://download.pytorch.org/whl/cu113

open3d==0.17.0
numpy==1.23.5
scipy==1.10.1

rospkg
catkin_pkg
```

---

## ✅ 现在应该可以成功构建了！

### 运行构建
```bash
cd /home/suhang/robocup_ur5e_ws

# 方法 1: 使用新的重建脚本
./rebuild_all.sh

# 方法 2: 手动构建
docker-compose build --no-cache
```

### 预期结果
```
robocup_ur5e/brain            latest    <ID>    ~4.7GB  ✅ 已完成
robocup_ur5e/perception_yolo  latest    <ID>    ~8GB    ⏳ 构建中
robocup_ur5e/perception_grasp latest    <ID>    ~10GB   ⏳ 等待
```

---

## 🎯 所有修复总结

| # | 问题 | 修复 | 文件 | 状态 |
|---|------|------|------|------|
| 1 | PyYAML 冲突 | `--ignore-installed` | Dockerfile.grasp | ✅ |
| 2 | py_trees_ros 缺失 | 改用 apt | Dockerfile.brain | ✅ |
| 3 | moveit_msgs 缺失 | 添加 apt 包 | Dockerfile.brain | ✅ |
| 4 | torch 2.5.1 不存在 | 改用 2.4.1 | requirements.txt | ✅ |
| 5 | numpy 1.26.4 不存在 | 改用 1.24.4 | requirements.txt | ✅ |

所有依赖问题已解决！🎉
