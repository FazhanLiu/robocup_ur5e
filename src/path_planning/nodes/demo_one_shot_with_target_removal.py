#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
挖掉指定点云部分后的 One-Shot 路径规划 Demo（方式 B：基于 YOLO 分割点云）。

流程：
  1. 订阅原始点云 /camera/depth/points 和分割点云 /perception/yolo26_seg_cloud
  2. 将两者变换到 base_link，用 pointcloud_target_removal 挖空“目标类别”区域
  3. 剩余点云体素化为障碍物，调用 ACO+RRT* 做一次性规划
  4. 发布轨迹到 /motion/command，可选发布 Marker、保存 3D 图

参数：
  ~target_class_id: int，要在点云中挖掉的 YOLO 类别 id（如 0=can）
  ~pointcloud_topic: 原始点云话题，默认 /camera/depth/points
  ~seg_cloud_topic: 带 label 的分割点云，默认 /perception/yolo26_seg_cloud
  ~frame_id: 规划坐标系，默认 base_link
  ~goal_pose_4x4: 可选。若传入 4x4 齐次变换（base 系下目标末端位姿），则使用 plan_one_shot_from_goal_pose，
                  忽略 virtual_grasp_point/goal_joints；默认不设则仍用 virtual_grasp_point + IK。
  其余与 demo_one_shot_planning 一致（~goal_joints_default, ~virtual_grasp_point 等）

运行前请确保：
  - roscore、motion_control 已运行
  - /camera/depth/points 与 /perception/yolo26_seg_cloud 有数据（YOLO 分割节点已跑）
  - TF 中有 base_link -> 点云 frame（如 camera_depth_optical_frame）

运行：
  rosrun path_planning demo_one_shot_with_target_removal.py
  rosrun path_planning demo_one_shot_with_target_removal.py _target_class_id:=0

方式 B 补全（默认开）：
  ~mode_b_merge_raw_obstacles:=true 且同步/订阅得到对齐 raw 时，在实例云体素障碍之外，
  再对 raw 按「目标实例 label」挖空后体素化并**合并**，以补全实例分割中常缺失的**桌面**等。

同步缓冲模式（可选）：
  ~synced_capture_mode:=true 时，启动后不立即规划；先 rosservice call <node>/run_planning_sync
  再在 ~sync_buffer_seconds（默认 10）秒内临时订阅：
    - ~pointcloud_topic（原始点云）
    - ~bbox_instance_cloud_topic（test_3dcloud_copy 的 XYZL 实例点云）
    - ~detections_json_topic（yolo26_seg_json_node 的 JSON）
    - 可选 ~sync_buffer_include_seg 为 true 时再订阅 ~seg_cloud_topic（与 raw 时间戳对齐）
  缓冲结束后按 header.stamp / JSON 内 gazebo_stamp 做最近邻对齐（容差 ~sync_tolerance_sec），再执行 _run_planning_pipeline。
"""

import sys
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import json

import rospy
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import Point as PointMsg, TransformStamped
from common_msgs.msg import MotionCommand
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2

from one_shot_planner import (
    plan_one_shot,
    plan_one_shot_from_goal_pose,
    get_default_virtual_grasp_point,
    build_motion_command_execute_trajectory,
    plot_planning_result_3d,
    build_obstacles_from_yolo_instance_cloud,
)
from aco_rrtstar_planner_node import (
    pointcloud_to_obstacles,
    GAZEBO_DEFAULT_OBSTACLES,
    KinematicsClient,
)
from pointcloud_target_removal import (
    remove_target_region_from_pointcloud,
    remove_target_region_from_pointcloud_by_instance_label,
)


def _param_bool(name, default=False):
    """launch 中 true/false 常为字符串，需安全解析。"""
    v = rospy.get_param(name, default)
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    return bool(v)


def _clear_all_plan_markers(frame_id="base_link"):
    """启动时清除本 demo 使用的所有 Marker（ee_path、status），以及历史上用 target_marker 发布过的目标点球残留。目标位姿已改为仅用 TF 显示，不再发布目标点。"""
    try:
        from visualization_msgs.msg import Marker
        now = rospy.Time.now()
        pubs = [
            ("~ee_path_marker", "one_shot_ee_path", 2),
            ("~target_marker", "one_shot_target", 1),  # 仅 DELETE 旧球体残留，目标现用 TF one_shot_target_pose
            ("~status_marker", "one_shot_status", 3),
        ]
        for topic, ns, mid in pubs:
            pub = rospy.Publisher(topic, Marker, queue_size=1, latch=True)
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame_id
            m.ns = ns
            m.id = mid
            m.action = Marker.DELETE
            pub.publish(m)
        rospy.sleep(0.15)
        rospy.loginfo("[DemoTargetRemoval] Cleared previous plan markers (ee_path, target_marker残留, status)")
    except Exception as e:
        rospy.logdebug("[DemoTargetRemoval] Clear markers: %s", e)


def _transform_pointcloud_to_frame(cloud_msg, target_frame, tf_buffer, timeout=0.5):
    """
    将 PointCloud2 变换到 target_frame（仅 x,y,z）。失败返回 None。
    """
    if cloud_msg.header.frame_id == target_frame:
        return cloud_msg
    try:
        trans = tf_buffer.lookup_transform(
            target_frame,
            cloud_msg.header.frame_id,
            cloud_msg.header.stamp if cloud_msg.header.stamp.to_sec() > 0 else rospy.Time(0),
            rospy.Duration(timeout),
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("[DemoTargetRemoval] TF %s -> %s failed: %s",
                      cloud_msg.header.frame_id, target_frame, str(e))
        return None
    points = []
    for p in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
        pt = PointMsg()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = float(p[2])
        pt_base = tf2_geometry_msgs.do_transform_point(pt, trans)
        points.append((pt_base.point.x, pt_base.point.y, pt_base.point.z))
    if not points:
        return None
    from std_msgs.msg import Header
    header = Header(stamp=trans.header.stamp, frame_id=target_frame)
    return pc2.create_cloud_xyz32(header, points)


def _transform_seg_cloud_to_frame(seg_cloud_msg, target_frame, tf_buffer, timeout=0.5):
    """
    将带 label 的分割点云变换到 target_frame，保留 label 字段。失败返回 None。
    """
    if seg_cloud_msg.header.frame_id == target_frame:
        return seg_cloud_msg
    has_label = any(f.name in ("label", "l") for f in seg_cloud_msg.fields)
    label_name = "label" if any(f.name == "label" for f in seg_cloud_msg.fields) else "l"
    field_names = ["x", "y", "z"]
    if has_label:
        field_names.append(label_name)
    try:
        trans = tf_buffer.lookup_transform(
            target_frame,
            seg_cloud_msg.header.frame_id,
            seg_cloud_msg.header.stamp if seg_cloud_msg.header.stamp.to_sec() > 0 else rospy.Time(0),
            rospy.Duration(timeout),
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("[DemoTargetRemoval] TF seg %s -> %s failed: %s",
                      seg_cloud_msg.header.frame_id, target_frame, str(e))
        return None
    points_out = []
    for p in pc2.read_points(seg_cloud_msg, skip_nans=True, field_names=field_names):
        pt = PointMsg()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = float(p[2])
        pt_base = tf2_geometry_msgs.do_transform_point(pt, trans)
        if has_label and len(p) >= 4:
            points_out.append((pt_base.point.x, pt_base.point.y, pt_base.point.z, int(p[3])))
        else:
            points_out.append((pt_base.point.x, pt_base.point.y, pt_base.point.z, 0))
    if not points_out:
        return None
    from std_msgs.msg import Header
    from sensor_msgs.msg import PointField
    header = Header(stamp=trans.header.stamp, frame_id=target_frame)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="label", offset=12, datatype=PointField.UINT32, count=1),
    ]
    return pc2.create_cloud(header, fields, points_out)


def _parse_float_list(param_val, default, min_len=3):
    if param_val is None:
        return default
    if isinstance(param_val, (list, tuple)) and len(param_val) >= min_len:
        return [float(param_val[i]) for i in range(min_len)]
    if isinstance(param_val, str):
        import ast
        try:
            v = ast.literal_eval(param_val)
            if isinstance(v, (list, tuple)) and len(v) >= min_len:
                return [float(v[i]) for i in range(min_len)]
        except Exception:
            pass
    return default


def _parse_goal_joints(param_val):
    if param_val is None:
        return None
    if isinstance(param_val, (list, tuple)) and len(param_val) >= 6:
        return [float(param_val[i]) for i in range(6)]
    if isinstance(param_val, str):
        import ast
        try:
            v = ast.literal_eval(param_val)
            if isinstance(v, (list, tuple)) and len(v) >= 6:
                return [float(v[i]) for i in range(6)]
        except Exception:
            pass
    return None


def _parse_goal_pose_4x4(param_val):
    """解析 ~goal_pose_4x4：4x4 齐次变换。支持 [[row0],[row1],[row2],[row3]] 或 16 个数行优先。未设置或空字符串返回 None。"""
    if param_val is None or (isinstance(param_val, str) and param_val.strip() == ""):
        return None
    import ast
    try:
        if isinstance(param_val, str):
            param_val = ast.literal_eval(param_val)
        if isinstance(param_val, (list, tuple)):
            if len(param_val) == 4 and all(isinstance(r, (list, tuple)) and len(r) == 4 for r in param_val):
                return [[float(param_val[i][j]) for j in range(4)] for i in range(4)]
            if len(param_val) == 16:
                return [[float(param_val[i * 4 + j]) for j in range(4)] for i in range(4)]
    except Exception:
        pass
    return None


def _gazebo_stamp_to_ros_time(gs):
    """JSON 内 gazebo_stamp: {\"secs\": int, \"nsecs\": int} -> rospy.Time"""
    if not isinstance(gs, dict):
        return None
    try:
        return rospy.Time(int(gs["secs"]), int(gs["nsecs"]))
    except (KeyError, TypeError, ValueError):
        return None


def _stamp_from_detections_json_string(data_str):
    """
    yolo26_seg_json_node 发布的 JSON 数组，取第一条的 gazebo_stamp 作为整帧逻辑时间戳。
    若为空数组则返回 None。
    """
    if not data_str or not isinstance(data_str, str):
        return None
    try:
        arr = json.loads(data_str)
        if not arr or not isinstance(arr, list):
            return None
        first = arr[0]
        if isinstance(first, dict) and "gazebo_stamp" in first:
            return _gazebo_stamp_to_ros_time(first["gazebo_stamp"])
    except (json.JSONDecodeError, TypeError, KeyError):
        pass
    return None


def _align_messages_by_stamp(raw_buf, inst_buf, json_buf, seg_buf, tolerance_sec):
    """
    时间戳对齐方案（三/四路）：
    - raw / instance PointCloud2：使用 msg.header.stamp（传感器时间）。
    - JSON String：无 header，使用解析后首条 detection 的 gazebo_stamp（与 RGB 同步推理时刻一致）。
    - seg（可选）：PointCloud2.header.stamp。

    策略：以 raw 缓冲中「最新一帧」的 stamp 为参考 T_ref，在容差内各找一条
    instance / json / seg 使 |T - T_ref| 最小；若任一路在容差内无候选则失败。

    返回: (raw_msg, inst_msg, json_parsed_list, seg_msg_or_none) 或 None
    """
    if not raw_buf or not inst_buf or not json_buf:
        return None

    def nearest_stamp(target_t, buf):
        best = None
        best_dt = None
        for t, payload in buf:
            dt = abs((t - target_t).to_sec())
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = (t, payload)
        if best is None or best_dt > tolerance_sec:
            return None
        return best[1]

    raw_sorted = sorted(raw_buf, key=lambda x: x[0], reverse=True)
    for t_ref, raw_msg in raw_sorted:
        inst_msg = nearest_stamp(t_ref, inst_buf)
        json_payload = nearest_stamp(t_ref, json_buf)
        if inst_msg is None or json_payload is None:
            continue
        seg_msg = None
        if seg_buf:
            seg_msg = nearest_stamp(t_ref, seg_buf)
            if seg_msg is None:
                continue
        parsed = None
        if isinstance(json_payload, str):
            try:
                parsed = json.loads(json_payload)
            except json.JSONDecodeError:
                continue
        elif isinstance(json_payload, list):
            parsed = json_payload
        else:
            continue
        rospy.loginfo(
            "[DemoTargetRemoval] Aligned triple: ref_stamp=%.6f tol=%.3fs inst_ok json_ok seg=%s",
            t_ref.to_sec(),
            tolerance_sec,
            seg_msg is not None,
        )
        return (raw_msg, inst_msg, parsed, seg_msg)
    return None


def _run_planning_pipeline(
    tf_buffer,
    aligned_raw_msg=None,
    aligned_instance_msg=None,
    aligned_seg_msg=None,
    aligned_json_list=None,
    spin=True,
):
    """
    执行一次完整规划与发布。若传入对齐后的点云/JSON，则优先使用，不再临时订阅等待单帧。
    aligned_json_list: yolo26_seg_detections 解析后的 list[dict]，仅用于日志或后续扩展。
    """
    frame_id = rospy.get_param("~frame_id", "base_link")
    _clear_all_plan_markers(frame_id)
    target_class_id = int(rospy.get_param("~target_class_id", 0))
    pointcloud_topic = rospy.get_param("~pointcloud_topic", "/camera/depth/points")
    seg_cloud_topic = rospy.get_param("~seg_cloud_topic", "/perception/yolo26_seg_cloud")
    instance_cloud_topic = rospy.get_param("~instance_cloud_topic", None)
    removal_padding = float(rospy.get_param("~removal_padding", 0.02))
    voxel_res = float(rospy.get_param("~voxel_resolution", 0.05))
    bounds = (
        tuple(_parse_float_list(rospy.get_param("~workspace_x", None), [-0.5, 1.0], min_len=2)),
        tuple(_parse_float_list(rospy.get_param("~workspace_y", None), [-0.5, 0.5], min_len=2)),
        tuple(_parse_float_list(rospy.get_param("~workspace_z", None), [0.0, 0.8], min_len=2)),
    )

    rospy.loginfo("[DemoTargetRemoval] target_class_id=%s, removal_padding=%.3f, frame=%s",
                  target_class_id, removal_padding, frame_id)
    if aligned_json_list:
        rospy.loginfo("[DemoTargetRemoval] Synced JSON detections count: %d", len(aligned_json_list))

    # 起点
    start_xyz, _ = _get_current_pose_from_tf(tf_buffer)
    if start_xyz is None:
        start_xyz = _parse_float_list(
            rospy.get_param("~start_xyz", None),
            [0.2, 0.0, 0.5],
        )
        rospy.logwarn("[DemoTargetRemoval] Using ~start_xyz: %s", start_xyz)
    rospy.loginfo("[DemoTargetRemoval] Start from TF: %s", [round(x, 3) for x in start_xyz])

    start_joints = _get_start_joints_from_topic(timeout=5.0)
    if start_joints is None:
        start_joints = _parse_goal_joints(rospy.get_param("~home_joints", None))
    if start_joints is None:
        start_joints = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
    rospy.loginfo("[DemoTargetRemoval] Start joints: %s", [round(x, 3) for x in start_joints])

    # 目标
    goal_pose_4x4 = _parse_goal_pose_4x4(rospy.get_param("~goal_pose_4x4", None))
    use_goal_pose_4x4 = goal_pose_4x4 is not None

    if not use_goal_pose_4x4:
        goal_xyz = _parse_float_list(
            rospy.get_param("~virtual_grasp_point", None),
            get_default_virtual_grasp_point(),
        )
        goal_joints = _parse_goal_joints(rospy.get_param("~goal_joints", None))
        if goal_joints is None:
            kc = KinematicsClient(use_motion_control=True)
            ik_joints = kc.ik(goal_xyz)
            if ik_joints is not None:
                goal_joints = list(ik_joints)
                rospy.loginfo("[DemoTargetRemoval] 由 goal_xyz 经 IK 得到 goal_joints: %s", [round(x, 3) for x in goal_joints])
            else:
                goal_joints = _parse_goal_joints(rospy.get_param("~goal_joints_default", None))
                if goal_joints is None:
                    goal_joints = [0.0, -2.0, 1.2, -1.5708, -1.5708, 0.0]
                rospy.logwarn("[DemoTargetRemoval] IK 失败，使用 goal_joints_default: %s", [round(x, 3) for x in goal_joints])
        rospy.loginfo("[DemoTargetRemoval] Goal xyz: %s, goal_joints: %s",
                      [round(x, 3) for x in goal_xyz], [round(x, 3) for x in goal_joints])
    else:
        rospy.loginfo("[DemoTargetRemoval] 使用 goal_pose_4x4 笛卡尔目标位姿")

    obstacles = None

    # 方式 B：实例点云 + target_center（优先使用对齐后的 instance；同步模式可不设 instance_cloud_topic）
    target_center = rospy.get_param("~target_center", None)
    if target_center is not None and (aligned_instance_msg is not None or instance_cloud_topic):
        try:
            if isinstance(target_center, (list, tuple)) and len(target_center) >= 3:
                target_center_xyz = [float(target_center[0]), float(target_center[1]), float(target_center[2])]
            else:
                import ast
                parsed = ast.literal_eval(str(target_center))
                target_center_xyz = [float(parsed[0]), float(parsed[1]), float(parsed[2])]
        except Exception:
            target_center_xyz = None

        if target_center_xyz is not None:
            seg_instance_cloud_msg = aligned_instance_msg
            if seg_instance_cloud_msg is None:
                seg_instance_cloud = [None]

                def _on_instance_cloud(msg):
                    if seg_instance_cloud[0] is None:
                        seg_instance_cloud[0] = msg

                sub_inst = rospy.Subscriber(instance_cloud_topic, PointCloud2, _on_instance_cloud, queue_size=1)
                rate = rospy.Rate(10)
                t0 = rospy.Time.now()
                while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 8.0:
                    if seg_instance_cloud[0] is not None:
                        break
                    rate.sleep()
                sub_inst.unregister()
                seg_instance_cloud_msg = seg_instance_cloud[0]

            if seg_instance_cloud_msg is not None:
                seg_instance_in_frame = _transform_pointcloud_to_frame(
                    seg_instance_cloud_msg, frame_id, tf_buffer, timeout=1.0
                )
                if seg_instance_in_frame is not None:
                    target_and_env = build_obstacles_from_yolo_instance_cloud(
                        seg_instance_in_frame,
                        target_center_xyz,
                        items_list_path=None,
                        voxel_res=voxel_res,
                        bounds=bounds,
                        include_target_obstacle=False,
                        current_joints=start_joints,
                    )
                    if target_and_env is not None:
                        obstacles = target_and_env.get("obstacles") or []
                        cls_id = target_and_env.get("class_id")
                        center = target_and_env.get("center")
                        matched_label = target_and_env.get("matched_label")
                        rospy.loginfo(
                            "[DemoTargetRemoval] YOLO 目标 class_id=%s, center=%s, 环境障碍物数量=%d",
                            str(cls_id),
                            ["{:.3f}".format(c) for c in center] if center is not None else "None",
                            len(obstacles),
                        )
                        # raw 按目标实例挖空后体素化并合并：补全桌面等（实例云常不含桌面）
                        if (
                            _param_bool("~mode_b_merge_raw_obstacles", True)
                            and aligned_raw_msg is not None
                            and matched_label is not None
                        ):
                            raw_for_merge = _transform_pointcloud_to_frame(
                                aligned_raw_msg, frame_id, tf_buffer, timeout=1.0
                            )
                            if raw_for_merge is not None:
                                from aco_rrtstar_planner_node import filter_pointcloud_robot_arm

                                filtered_raw = remove_target_region_from_pointcloud_by_instance_label(
                                    raw_for_merge,
                                    seg_instance_in_frame,
                                    int(matched_label),
                                    padding=removal_padding,
                                    output_frame_id=frame_id,
                                )
                                cloud_no_robot = filter_pointcloud_robot_arm(
                                    filtered_raw, start_joints, frame_id=frame_id
                                )
                                raw_obs = pointcloud_to_obstacles(
                                    cloud_no_robot,
                                    voxel_res=voxel_res,
                                    frame_id=frame_id,
                                    bounds=bounds,
                                )
                                if raw_obs:
                                    obstacles = list(obstacles) + raw_obs
                                    rospy.loginfo(
                                        "[DemoTargetRemoval] Mode B 合并 raw 体素障碍: +%d（含桌面等场景）",
                                        len(raw_obs),
                                    )
                        if not obstacles:
                            obstacles = None

    # 方式 A：分割挖空
    if obstacles is None:
        raw_in_frame = None
        seg_in_frame = None
        if aligned_raw_msg is not None and aligned_seg_msg is not None:
            raw_in_frame = _transform_pointcloud_to_frame(aligned_raw_msg, frame_id, tf_buffer, timeout=1.0)
            seg_in_frame = _transform_seg_cloud_to_frame(aligned_seg_msg, frame_id, tf_buffer, timeout=1.0)
        elif aligned_raw_msg is not None and aligned_seg_msg is None:
            rospy.logwarn("[DemoTargetRemoval] 有对齐 raw 但无 seg，回退为等待 seg 话题")
            raw_in_frame, seg_in_frame = _get_raw_and_seg_clouds(
                pointcloud_topic, seg_cloud_topic, tf_buffer, frame_id, timeout=8.0
            )
        else:
            raw_in_frame, seg_in_frame = _get_raw_and_seg_clouds(
                pointcloud_topic, seg_cloud_topic, tf_buffer, frame_id, timeout=8.0
            )
        if raw_in_frame is None or seg_in_frame is None:
            rospy.logerr("[DemoTargetRemoval] Cannot get point clouds, using default obstacles")
            obstacles = GAZEBO_DEFAULT_OBSTACLES
        else:
            from aco_rrtstar_planner_node import filter_pointcloud_robot_arm
            filtered_cloud = remove_target_region_from_pointcloud(
                raw_in_frame,
                seg_in_frame,
                target_class_id,
                padding=removal_padding,
                output_frame_id=frame_id,
            )
            cloud_no_robot = filter_pointcloud_robot_arm(
                filtered_cloud, start_joints, frame_id=frame_id
            )
            obstacles = pointcloud_to_obstacles(
                cloud_no_robot,
                voxel_res=voxel_res,
                frame_id=frame_id,
                bounds=bounds,
            )
            if not obstacles:
                obstacles = GAZEBO_DEFAULT_OBSTACLES
                rospy.logwarn("[DemoTargetRemoval] No obstacles after removal, using default")
            else:
                rospy.loginfo("[DemoTargetRemoval] Obstacles after target removal: %d", len(obstacles))

    # 一次性规划
    if use_goal_pose_4x4:
        result = plan_one_shot_from_goal_pose(
            goal_pose_4x4=goal_pose_4x4,
            start_xyz=start_xyz,
            start_joints=start_joints,
            obstacles=obstacles,
            bounds=bounds,
            frame_id=frame_id,
            return_vis_data=True,
            seed_joints_for_ik=start_joints,
        )
    else:
        result = plan_one_shot(
            start_xyz=start_xyz,
            start_joints=start_joints,
            goal_xyz=goal_xyz,
            goal_joints=goal_joints,
            obstacles=obstacles,
            return_vis_data=True,
        )
    if result is None or len(result) < 2:
        rospy.logerr("[DemoTargetRemoval] Planning failed")
        return False
    path_joints, trajectory = result[0], result[1]
    vis_data = result[2] if len(result) > 2 else None

    if path_joints is None or trajectory is None:
        rospy.logerr("[DemoTargetRemoval] Planning failed")
        return False

    target_pose_frame_id = rospy.get_param("~target_pose_frame_id", "one_shot_target_pose")
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    target_pose_xyz = list(vis_data["goal_xyz"]) if vis_data and "goal_xyz" in vis_data else [0.0, 0.0, 0.0]

    def _publish_target_pose_tf(_event=None):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = target_pose_frame_id
        t.transform.translation.x = float(target_pose_xyz[0])
        t.transform.translation.y = float(target_pose_xyz[1])
        t.transform.translation.z = float(target_pose_xyz[2])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(t)

    _publish_target_pose_tf()
    rospy.Timer(rospy.Duration(0.1), _publish_target_pose_tf)

    try:
        from one_shot_trajectory_display import publish_plan_markers
        from visualization_msgs.msg import Marker
        if rospy.get_param("~publish_plan_markers", True):
            path_marker_pub = rospy.Publisher("~ee_path_marker", Marker, queue_size=1, latch=True)
            status_marker_pub = rospy.Publisher("~status_marker", Marker, queue_size=1, latch=True)
            rospy.sleep(0.2)
            publish_plan_markers(
                path_marker_pub,
                None,
                status_marker_pub,
                vis_data,
                success=True,
                frame_id=frame_id,
            )
            rospy.loginfo("[DemoTargetRemoval] Published trajectory markers and TF %s -> %s (target pose only in TF)", frame_id, target_pose_frame_id)
    except Exception as e:
        rospy.logdebug("[DemoTargetRemoval] Markers: %s", e)

    if vis_data:
        viz_path = rospy.get_param("~planning_viz_3d_output", "/tmp/planning_result_3d_target_removal.png")
        plot_planning_result_3d(vis_data, output_path=viz_path)

    cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
    rospy.sleep(0.5)
    cmd = build_motion_command_execute_trajectory(trajectory)
    cmd_pub.publish(cmd)
    rospy.loginfo("[DemoTargetRemoval] Published EXECUTE_TRAJECTORY, %d points", len(trajectory.points))

    if spin:
        rospy.spin()
    return True


def _get_current_pose_from_tf(tf_buffer, tcp_link="gripper_tip_link", timeout=0.5):
    try:
        trans = tf_buffer.lookup_transform(
            "base_link", tcp_link, rospy.Time(0), rospy.Duration(timeout)
        )
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        return [x, y, z], trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None, None


def _get_start_joints_from_topic(timeout=5.0):
    joints = [None]

    def cb(msg):
        if joints[0] is None and msg.name and msg.position:
            order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            names = list(msg.name)
            pos = list(msg.position)
            try:
                joints[0] = [pos[names.index(n)] for n in order]
            except (ValueError, IndexError):
                if len(pos) >= 6:
                    joints[0] = list(pos)[:6]

    sub = rospy.Subscriber('/joint_states', JointState, cb, queue_size=1)
    rate = rospy.Rate(20)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < timeout:
        if joints[0] is not None:
            break
        rate.sleep()
    sub.unregister()
    return joints[0]


def _get_raw_and_seg_clouds(pointcloud_topic, seg_cloud_topic, tf_buffer, frame_id, timeout=5.0):
    """等待一帧原始点云和一帧分割点云，并都变换到 frame_id。返回 (raw_in_frame, seg_in_frame) 或 (None, None)。"""
    raw = [None]
    seg = [None]

    def on_raw(msg):
        if raw[0] is None:
            raw[0] = msg

    def on_seg(msg):
        if seg[0] is None:
            seg[0] = msg

    sub_raw = rospy.Subscriber(pointcloud_topic, PointCloud2, on_raw, queue_size=1)
    sub_seg = rospy.Subscriber(seg_cloud_topic, PointCloud2, on_seg, queue_size=1)
    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < timeout:
        if raw[0] is not None and seg[0] is not None:
            break
        rate.sleep()
    sub_raw.unregister()
    sub_seg.unregister()

    if raw[0] is None:
        rospy.logerr("[DemoTargetRemoval] No raw point cloud from %s", pointcloud_topic)
        return None, None
    if seg[0] is None:
        rospy.logerr("[DemoTargetRemoval] No seg point cloud from %s", seg_cloud_topic)
        return None, None

    raw_in_frame = _transform_pointcloud_to_frame(raw[0], frame_id, tf_buffer, timeout=1.0)
    seg_in_frame = _transform_seg_cloud_to_frame(seg[0], frame_id, tf_buffer, timeout=1.0)
    if raw_in_frame is None:
        rospy.logerr("[DemoTargetRemoval] Failed to transform raw cloud to %s", frame_id)
        return None, None
    if seg_in_frame is None:
        rospy.logerr("[DemoTargetRemoval] Failed to transform seg cloud to %s", frame_id)
        return None, None
    return raw_in_frame, seg_in_frame


def main():
    rospy.init_node("demo_one_shot_with_target_removal", anonymous=False)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.5)

    if not _param_bool("~synced_capture_mode", False):
        _run_planning_pipeline(tf_buffer, spin=True)
        return

    # ---- 同步缓冲模式：收到服务调用后再订阅 raw / JSON / instance（可选 seg），缓冲 N 秒后按时间戳对齐 ----
    buffer_sec = float(rospy.get_param("~sync_buffer_seconds", 10.0))
    tol = float(rospy.get_param("~sync_tolerance_sec", 0.05))
    include_seg = _param_bool("~sync_buffer_include_seg", True)
    pointcloud_topic = rospy.get_param("~pointcloud_topic", "/camera/depth/points")
    seg_cloud_topic = rospy.get_param("~seg_cloud_topic", "/perception/yolo26_seg_cloud")
    json_topic = rospy.get_param("~detections_json_topic", "/perception/yolo26_seg_detections")
    instance_topic = rospy.get_param("~bbox_instance_cloud_topic", "/perception/yolo_bbox_instance_cloud")

    raw_buf = []
    inst_buf = []
    json_buf = []
    seg_buf = []

    def on_raw(msg):
        raw_buf.append((msg.header.stamp, msg))

    def on_inst(msg):
        inst_buf.append((msg.header.stamp, msg))

    def on_json(msg):
        st = _stamp_from_detections_json_string(msg.data)
        if st is not None:
            json_buf.append((st, msg.data))

    def on_seg(msg):
        seg_buf.append((msg.header.stamp, msg))

    def handle_run_planning_sync(_req):
        raw_buf.clear()
        inst_buf.clear()
        json_buf.clear()
        seg_buf.clear()
        subs = [
            rospy.Subscriber(pointcloud_topic, PointCloud2, on_raw, queue_size=100),
            rospy.Subscriber(instance_topic, PointCloud2, on_inst, queue_size=100),
            rospy.Subscriber(json_topic, String, on_json, queue_size=100),
        ]
        if include_seg:
            subs.append(rospy.Subscriber(seg_cloud_topic, PointCloud2, on_seg, queue_size=100))
        rospy.loginfo(
            "[DemoTargetRemoval] Sync capture: subscribing %ds to raw=%s inst=%s json=%s%s",
            int(buffer_sec),
            pointcloud_topic,
            instance_topic,
            json_topic,
            " seg=%s" % seg_cloud_topic if include_seg else "",
        )
        rospy.sleep(buffer_sec)
        for s in subs:
            s.unregister()
        seg_for_align = seg_buf if include_seg else None
        triple = _align_messages_by_stamp(raw_buf, inst_buf, json_buf, seg_for_align, tol)
        if triple is None:
            rospy.logerr(
                "[DemoTargetRemoval] 时间戳对齐失败（需 raw+inst+json 在 %.3fs 内；seg=%s）。缓冲条数 raw=%d inst=%d json=%d seg=%d",
                tol,
                include_seg,
                len(raw_buf),
                len(inst_buf),
                len(json_buf),
                len(seg_buf),
            )
            return TriggerResponse(success=False, message="timestamp alignment failed")
        raw_msg, inst_msg, parsed, seg_msg = triple
        try:
            ok = _run_planning_pipeline(
                tf_buffer,
                aligned_raw_msg=raw_msg,
                aligned_instance_msg=inst_msg,
                aligned_seg_msg=seg_msg,
                aligned_json_list=parsed,
                spin=False,
            )
        except Exception as exc:
            rospy.logerr("[DemoTargetRemoval] Planning exception: %s", exc)
            return TriggerResponse(success=False, message=str(exc))
        return TriggerResponse(
            success=bool(ok),
            message="planning executed" if ok else "planning failed",
        )

    rospy.Service("~run_planning_sync", Trigger, handle_run_planning_sync)
    try:
        spinner = rospy.AsyncSpinner()
        spinner.start()
    except Exception:
        pass
    rospy.loginfo(
        "[DemoTargetRemoval] synced_capture_mode=ON: 调用 rosservice call %s/run_planning_sync \"{}\" 开始缓冲并对齐",
        rospy.get_name(),
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
