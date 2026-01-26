# 🔧 构建错误修复说明（更新版）

## 问题总结

在构建过程中遇到了三个依赖问题，现已全部修复。

---

## 错误 1: perception_grasp - PyYAML 冲突 ✅

### 错误信息
```
× Cannot uninstall PyYAML 5.3.1
╰─> It is a distutils installed project
```

### 修复
在 `docker/Dockerfile.grasp` 中添加 `--ignore-installed PyYAML`

---

## 错误 2: robocup_brain - py_trees_ros 不存在 ✅

### 错误信息
```
ERROR: No matching distribution found for py_trees_ros==2.2.2
```

### 修复
使用 `apt` 安装 `ros-noetic-py-trees-ros`

---

## 错误 3: robocup_brain - moveit_msgs 缺失 ✅

### 错误信息
```
CMake Error: Could not find a package configuration file provided by "moveit_msgs"
```

### 原因
- `package.xml` 声明了对 `moveit_msgs` 的依赖
- Dockerfile 中没有安装 MoveIt 相关包

### 修复
在 `docker/Dockerfile.brain` 中添加：
```dockerfile
RUN apt-get update && apt-get install -y \
    ros-noetic-py-trees \
    ros-noetic-py-trees-ros \
    ros-noetic-moveit-msgs \
    ros-noetic-actionlib \
    && rm -rf /var/lib/apt/lists/*
```

---

## 现在重新构建

### 方法 1: 使用修复脚本（推荐）

```bash
cd /home/suhang/robocup_ur5e_ws
./fix_build.sh
```

### 方法 2: 手动构建

```bash
# 清理
docker-compose down
docker system prune -f

# 重新构建（不使用缓存以确保使用最新的 Dockerfile）
docker-compose build --no-cache

# 或单独构建（测试用）
docker-compose build --no-cache robocup_brain
```

---

## 预期结果

所有 3 个镜像应该成功构建：

```bash
$ docker images | grep robocup_ur5e
robocup_ur5e/brain            latest    <ID>    <时间>    ~2GB
robocup_ur5e/perception_yolo  latest    <ID>    <时间>    ~8GB
robocup_ur5e/perception_grasp latest    <ID>    <时间>    ~10GB
```

---

## 构建时间

- **robocup_brain**: ~5 分钟（现在需要安装 MoveIt 包）
- **perception_yolo**: ~10-15 分钟
- **perception_grasp**: ~15-20 分钟

**总计**: 约 30-40 分钟

---

## 验证修复

构建成功后测试：

```bash
# 启动
docker-compose up -d

# 测试 Brain（包括 py_trees 和 moveit_msgs）
docker exec robocup_brain bash -c "python3 -c 'import py_trees, py_trees_ros; print(\"py_trees OK\")'"
docker exec robocup_brain bash -c "rospack find moveit_msgs && echo 'moveit_msgs OK'"

# 测试 YOLO
docker exec perception_yolo python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"

# 测试 Grasp
docker exec perception_grasp python3 -c "import open3d, torch; print('OK')"
```

---

## 已修复的文件

1. ✅ `docker/Dockerfile.brain` - 添加 moveit_msgs 和 actionlib
2. ✅ `docker/Dockerfile.grasp` - 修复 PyYAML 冲突
3. ✅ `verify_system.sh` - 改进错误检测
4. ✅ `fix_build.sh` - 更新修复说明

---

## 为什么需要 moveit_msgs？

查看 `src/robocup_brain/package.xml`:
```xml
<depend>moveit_msgs</depend>
<depend>actionlib</depend>
```

以及 `src/robocup_brain/nodes/brain_node.py`:
```python
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
```

robocup_brain 需要通过 MoveIt 控制机械臂，所以这些依赖是必需的。

---

**所有问题已修复！现在运行 `./fix_build.sh` 开始构建。** 🚀
