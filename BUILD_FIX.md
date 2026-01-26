# 🔧 构建错误修复说明

## 问题总结

在首次构建时遇到了两个依赖问题，现已修复。

---

## 错误 1: perception_grasp 构建失败

### 错误信息
```
× Cannot uninstall PyYAML 5.3.1
╰─> It is a distutils installed project and thus we cannot accurately 
    determine which files belong to it which would lead to only a partial uninstall.
```

### 原因
- ROS Noetic 容器预装了 `PyYAML 5.3.1`（通过系统包管理器安装）
- pip 无法卸载 distutils 安装的包
- `requirements.txt` 要求安装 `PyYAML 6.0.3`，导致冲突

### 修复方法
修改 `docker/Dockerfile.grasp`，使用 `--ignore-installed PyYAML` 跳过已安装版本：

```dockerfile
# 修复前
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 修复后
RUN pip3 install --no-cache-dir --ignore-installed PyYAML -r /tmp/requirements.txt
```

---

## 错误 2: robocup_brain 构建失败

### 错误信息
```
ERROR: Could not find a version that satisfies the requirement py_trees_ros==2.2.2 
ERROR: No matching distribution found for py_trees_ros==2.2.2
```

### 原因
- PyPI 上没有 `py_trees_ros` 包
- `py_trees_ros` 是 ROS 生态系统的包，需要通过 `apt` 安装

### 修复方法
修改 `docker/Dockerfile.brain`，使用 ROS 仓库安装：

```dockerfile
# 修复前
RUN pip3 install \
    py_trees==2.2.3 \
    py_trees_ros==2.2.2

# 修复后
RUN apt-get update && apt-get install -y \
    ros-noetic-py-trees \
    ros-noetic-py-trees-ros \
    && rm -rf /var/lib/apt/lists/*
```

---

## 如何重新构建

### 方法 1: 使用修复脚本（推荐）

```bash
cd /home/suhang/robocup_ur5e_ws
./fix_build.sh
```

这个脚本会：
1. 清理旧的构建缓存
2. 重新构建所有镜像
3. 显示构建结果

### 方法 2: 手动重新构建

```bash
# 清理旧容器和缓存
docker-compose down
docker system prune -f

# 重新构建（不使用缓存）
docker-compose build --no-cache

# 或单独构建每个服务
docker-compose build --no-cache robocup_brain
docker-compose build --no-cache perception_yolo
docker-compose build --no-cache perception_grasp
```

### 方法 3: 使用验证脚本

```bash
./verify_system.sh
```

验证脚本已更新，能正确检测构建错误。

---

## 预期结果

构建成功后，您应该看到：

```bash
$ docker images | grep robocup_ur5e
robocup_ur5e/brain            latest    <ID>    <时间>    ~2GB
robocup_ur5e/perception_yolo  latest    <ID>    <时间>    ~8GB
robocup_ur5e/perception_grasp latest    <ID>    <时间>    ~10GB
```

---

## 常见问题

### Q: 构建时间太长？
**A:** 正常现象。首次构建需要：
- Brain: ~5 分钟
- YOLO: ~10-15 分钟（CUDA 12.0 + PyTorch）
- Grasp: ~15-20 分钟（CUDA 11.3 + Open3D + PyTorch）

### Q: 如何查看详细构建日志？
**A:** 
```bash
# 查看某个服务的构建日志
docker-compose build robocup_brain 2>&1 | tee build.log

# 查看验证脚本生成的日志
cat /tmp/build_brain.log
cat /tmp/build_yolo.log
cat /tmp/build_grasp.log
```

### Q: 磁盘空间不足？
**A:** 
```bash
# 检查空间
df -h

# 清理旧镜像（谨慎使用）
docker system prune -a
```

### Q: 网络下载慢？
**A:** 构建过程需要下载：
- PyTorch (CUDA 11.3): ~1.8 GB
- PyTorch (CUDA 12.0): ~2.0 GB
- Open3D: ~420 MB
- 其他依赖: ~2 GB

如果网络慢，可以考虑使用国内镜像源。

---

## 技术细节

### PyYAML 冲突的根本原因
ROS Noetic 使用 `distutils` 安装 Python 包，这是一个老旧的安装方式：
- 不记录安装的文件列表
- pip 无法安全卸载
- 解决方案：使用 `--ignore-installed` 强制覆盖

### py_trees_ros 包管理
ROS 包的分发策略：
- **Python 包（pip）**: 通用 Python 库
- **ROS 包（apt）**: 与 ROS 紧密集成的包（如 py_trees_ros）
- py_trees_ros 依赖 ROS 的消息系统，必须通过 apt 安装

---

## 验证修复

构建成功后，运行测试：

```bash
# 启动容器
docker-compose up -d

# 测试 Brain
docker exec robocup_brain python3 -c "import py_trees, py_trees_ros; print('OK')"

# 测试 YOLO
docker exec perception_yolo python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"

# 测试 Grasp
docker exec perception_grasp python3 -c "import open3d, torch; print('OK')"
```

所有测试都应该输出 "OK" 或 "CUDA: True"。

---

## 已修复文件

- ✅ `docker/Dockerfile.brain` (修复 py_trees_ros)
- ✅ `docker/Dockerfile.grasp` (修复 PyYAML 冲突)
- ✅ `verify_system.sh` (改进错误检测)
- ✅ `fix_build.sh` (新增修复脚本)

---

现在可以重新运行构建了！🚀
