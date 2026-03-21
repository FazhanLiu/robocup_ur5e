# perception_grasp/nodes 说明

本文档说明 `perception_grasp/nodes` 目录下在线抓取相关节点的用途、输入输出、联调方式和常见注意事项。

## 1. 目录内容

当前目录下主要有两个核心文件：

- `grasp_estimator_node.py`
  - 在线抓取节点
  - 负责订阅语义点云、接收 FSM 抓取请求、按目标 `label/id` 过滤点云、运行 GraspNet、发布抓取结果
- `grasp_inference_core.py`
  - GraspNet 推理核心
  - 负责模型加载、点云预处理、Grasp 候选生成，以及抓取方向过滤等通用逻辑

## 2. 在线抓取主流程

在线流程的主链路如下：

1. 上游感知节点发布语义点云  
   - `/perception/yolo_bbox_instance_cloud`

2. `grasp_estimator_node.py` 持续缓存最新点云  
   点云中至少需要包含：
   - `x`
   - `y`
   - `z`
   - `label`

3. FSM 通过 `/brain/task_decision` 发送抓取请求  
   其中：
   - `task_type = GRASP`
   - `target_object_id = 目标 label/id`
   - 可选：`target_pose.header.stamp = 感知快照时间戳`
   - 可选：`target_pose.header.frame_id = 感知快照坐标系`

4. 抓取节点优先按 `source_stamp/source_frame` 选中同一帧点云  
   如果 FSM 没有提供时间戳，则退化为使用最新缓存点云

5. 抓取节点按 `target_object_id == label` 过滤点云  
   只保留目标物体对应的那一团点

6. 运行 GraspNet 推理  
   生成原始抓取候选

7. 做抓取后处理  
   当前已接入的约束包括：
   - 接近方向过滤（例如偏上方抓取）
   - 夹爪宽度范围过滤

8. 默认将抓取位姿从输入点云坐标系转换到 `base_link`

9. 发布抓取结果和可视化

## 3. 输入输出说明

### 3.1 输入

#### 点云输入

由参数 `pointcloud_topic` 控制，当前为了贴合项目联调主流程，默认就是：

- `/perception/yolo_bbox_instance_cloud`

如果后面需要兼容旧链路，也可以手动改回：

- `/perception/yolo26_seg_cloud`

点云消息类型：

- `sensor_msgs/PointCloud2`

点云字段要求：

- `x: float32`
- `y: float32`
- `z: float32`
- `label: uint32`

#### FSM 输入

默认 topic：

- `/brain/task_decision`

消息类型：

- `common_msgs/TaskDecision`

当前抓取节点使用其中两个必填字段：

- `task_type`
- `target_object_id`

为了解决“FSM 选中的目标”和“grasp 实际使用的点云帧”错位的问题，当前还支持两个可选快照字段：

- `target_pose.header.stamp`
- `target_pose.header.frame_id`

约定如下：

- `target_pose.header.stamp`
  - 表示 FSM 做决策时所依据的感知快照时间戳
- `target_pose.header.frame_id`
  - 表示该感知快照所在坐标系

如果这两个字段不填，抓取节点会保持旧行为：

- 直接使用最新缓存的点云

### 3.2 输出

#### 抓取候选

- `/perception/grasp_candidates`
- 类型：`common_msgs/GraspCandidate`
- 当前输出字段包括：
  - `pose`
  - `quality`
  - `width`

#### 抓取失败原因

- `/perception/grasp_failure_reason`
- 类型：`std_msgs/String`

#### 抓取可视化

- `/perception/grasp_markers`
- 类型：`visualization_msgs/MarkerArray`

## 4. 当前已实现的在线能力

### 4.1 目标点云过滤

抓取节点会读取点云中的 `label` 字段，并执行：

```text
labels == target_object_id
```

这意味着：

- 如果 FSM 要抓 `label=1`
- 节点就只保留 `label=1` 的点
- 其他点全部丢弃

### 4.2 原始候选与最终输出分离

当前逻辑已支持：

```text
先取很多 raw_top_k
-> 后处理过滤
-> 最终只输出 num_grasp_candidates
```

这样可以避免：

- 原始只取很少候选
- 过滤后变成 0

### 4.3 接近方向过滤

当前支持基于 grasp approach direction 做过滤，用来减少桌面场景下不合理的底部抓取姿态。

相关参数：

- `approach_filter`
- `min_down_dot`
- `max_up_dot`

### 4.4 夹爪宽度过滤

当前支持按真实夹爪的开合范围过滤 grasp：

- 最小：`0.0178 m`
- 最大：`0.1006 m`

相关参数：

- `min_gripper_width`
- `max_gripper_width`

### 4.5 TF 坐标系转换

如果输入点云在相机坐标系下，例如：

- `camera_depth_link`

当前默认会在发布前将抓取位姿转换到：

- `base_link`

相关参数：

- `output_frame`
- `tf_lookup_timeout`
- `publish_transformed_markers`

当前默认策略：

- 推理仍然在输入点云原始坐标系内完成
- 只对最终要发布的少量抓取候选做 TF 转换

### 4.6 时间戳对齐抓取

为减少异步 topic 带来的“不是同一帧数据”问题，抓取节点现在支持：

```text
FSM 发 target_object_id + source_stamp(+source_frame)
-> grasp 在最近若干帧点云缓存中按时间戳选帧
-> 再按 label 过滤目标点云
-> 运行 grasp
```

相关参数：

- `cloud_history_size`
- `source_stamp_tolerance`

默认策略：

- `cloud_history_size = 10`
- `source_stamp_tolerance = 0.15 s`

说明：

- 如果 FSM 提供了 `target_pose.header.stamp`
  - 抓取节点会在最近缓存的点云中寻找时间最接近的一帧
  - 只有在容差范围内才会使用
- 如果 FSM 没提供时间戳
  - 抓取节点退化为使用最新缓存点云，保持向后兼容

### 4.7 可选保存过滤后的目标点云

为了联调和留档，当前节点支持把“按 `target_object_id` 过滤后”的目标点云保存为 `.pcd`。

相关参数：

- `save_filtered_target_cloud`
- `filtered_cloud_save_dir`

适用场景：

- 记录某次抓取请求对应的目标点云
- 离线复现某个在线抓取问题
- 保存证据用于汇报/联调

注意：

- 这是调试增强功能
- 不是在线主流程必须步骤

## 5. 推荐联调方式

### 5.0 环境说明

重建最新 `perception_grasp` Docker 镜像后，工作空间标准环境为：

- `/workspace/devel`

因此，下面所有会用到新版 `GraspCandidate.width` 和新版 grasp 节点的命令，都应先执行：
 
```bash
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash
```

### 5.1 与 YOLO 实例点云联调

推荐上游使用：

- `test_3dcloud_copy.py`

它会发布：

- `/perception/yolo_bbox_instance_cloud`

该点云已经验证为可直接被当前抓取节点读取，且包含：

- `x`
- `y`
- `z`
- `label`

其中 `label` 为实例 ID。

### 5.2 启动 grasp 节点示例

```bash
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

roslaunch perception_grasp grasp_estimator.launch \
  pointcloud_topic:=/perception/yolo_bbox_instance_cloud \
  label_field_name:=label \
  output_frame:=base_link \
  cloud_history_size:=10 \
  source_stamp_tolerance:=0.15 \
  raw_top_k:=1000 \
  num_grasp_candidates:=1 \
  approach_filter:=top_side \
  min_down_dot:=-1.0 \
  max_up_dot:=1.0 \
  min_gripper_width:=0.0178 \
  max_gripper_width:=0.1006 \
  save_filtered_target_cloud:=true \
  filtered_cloud_save_dir:=/workspace/weights/graspnet/debug_clouds/yolo_main
```

这条命令的含义是：

- 输入实例点云
- 按 `label` 过滤
- 输出 `base_link` 下的抓取位姿
- 原始多取候选，再只发布 top-1
- 同时保存这次被筛出来的目标点云

### 5.3 FSM 触发方式

理论上，FSM 只需要发：

- `task_type = GRASP`
- `target_object_id = 目标实例 label`

抓取节点就会自动：

- 过滤目标点云
- 跑 GraspNet
- 发布 grasp

如果手工测试，推荐用 Python 发送消息：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

python3 - <<'PY'
import rospy
from common_msgs.msg import TaskDecision

pub = rospy.Publisher('/brain/task_decision', TaskDecision, queue_size=1)
rospy.init_node('task_decision_test_pub', anonymous=True)
rospy.sleep(1.0)

msg = TaskDecision()
msg.task_type = TaskDecision.GRASP
msg.target_object_id = 1
pub.publish(msg)
rospy.sleep(0.5)
PY
```

## 6. 当前联调已验证的结论

目前已经验证过：

1. `test_3dcloud_copy.py` 发布的 `/perception/yolo_bbox_instance_cloud` 可以被当前抓取节点正确接收
2. 抓取节点可以按 `target_object_id == label` 正确过滤点云
3. 过滤出的目标点云可以保存为 `.pcd`
4. 抓取结果可以发布在 `base_link` 坐标系下
5. 抓取结果能够通过 `/perception/grasp_candidates` 和 `/perception/grasp_markers` 输出

## 7. 需要注意的已知问题

### 7.1 TaskDecision.GRASP 的字符串定义问题

当前 `common_msgs/TaskDecision.msg` 里：

```text
string GRASP = "grasp"
```

会导致 Python 侧常量值变成带引号的字符串：

```python
TaskDecision.GRASP == '"grasp"'
```

因此在手工测试时：

- 最稳的做法是用 Python 发布 `TaskDecision`
- 不建议直接手写 `rostopic pub` 去猜这个字符串

这个问题属于公共消息定义问题，不属于 `perception_grasp` 内部逻辑。

### 7.2 YOLO 实例 ID 是单帧语义

`test_3dcloud_copy.py` 输出的 `label` 是当前帧内实例 ID。  
这意味着：

- 当前帧 `label=1`
- 下一帧不一定还是同一个物体

所以系统级联调时，需要确认：

- FSM 发的 `target_object_id`
- 是否与当前帧实例 ID 语义对齐

### 7.3 方向过滤可能导致 0 candidate

如果 `approach_filter` 过严，可能出现：

- 成功过滤出目标点云
- 但所有 grasp 候选都被后处理筛掉

此时建议：

- 先放宽阈值验证链路
- 再逐步收紧约束

## 8. 总结

当前 `perception_grasp/nodes` 已经具备以下在线能力：

- 接收语义点云
- 按 FSM 目标 ID 过滤目标点云
- 运行 GraspNet
- 做抓取方向与夹爪宽度约束
- 将结果变换到 `base_link`
- 发布 grasp 结果与 RViz marker
- 可选保存过滤后的目标点云用于离线复现

如果联调目标是“证明 grasp 部分已经打通”，推荐优先走这条主链：

```text
test_3dcloud_copy.py
-> /perception/yolo_bbox_instance_cloud
-> grasp_estimator_node.py
-> /perception/grasp_candidates
```
