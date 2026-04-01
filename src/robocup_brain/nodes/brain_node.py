#!/usr/bin/env python3
"""
RoboCup Brain Node - behavior-tree based task orchestration.

Current pick and place flow:
  1. Move to the overview joint pose once.
  2. Open gripper once.
  3. Pick the best YOLO target that is not blacklisted.
  4. Build a direct grasp pose from YOLO 3D position.
  5. Try path_planning first to approach the grasp pose.
  6. If path_planning fails, fall back to direct motion_control MOVE_TO_POSE.
  7. Close the gripper without blocking on holding verification.
  8. Return to the overview joint pose.
  9. Move to the selected trash bin joint pose and release.
"""

import copy
import json
import math
import re

import actionlib
import numpy as np
import actionlib_msgs.msg as action_msgs
import py_trees
import py_trees_ros
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, PoseArray, PoseStamped, Quaternion, TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from py_trees.common import Status
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from common_msgs.msg import (
    GraspCandidate,
    GraspResult,
    MotionCommand,
    PlanExecutePoseAction,
    PlanExecutePoseFeedback,
    PlanExecutePoseGoal,
    PlanExecutePoseResult,
    TaskDecision,
)


FAILED_TARGET_KEYS_BB = "failed_target_keys"
LAST_SUCCESSFUL_PICK_CATEGORY_BB = "last_successful_pick_category"
CUBE_ON_SCALE_BB = "cube_on_scale"
CUBE_ON_SCALE_LABEL_BB = "cube_on_scale_label"
CUBE_ON_SCALE_BIN_COLOR_BB = "cube_on_scale_bin_color"
SCENE_RANK_TARGETS_BB = "scene_rank_targets"
SCENE_RANKED_RESULTS_BB = "scene_ranked_results"
SCENE_RANK_FAILED_TARGETS_BB = "scene_rank_failed_targets"
SCENE_PREOBSERVE_TARGET_BB = "scene_preobserve_target"
LATEST_YOLO_FRAME_ID_BB = "latest_yolo_frame_id"
LATEST_YOLO_STAMP_BB = "latest_yolo_stamp"
LATEST_YOLO_RECV_TIME_BB = "latest_yolo_recv_time"
TARGET_BB_KEYS = (
    "target_object",
    "target_point_base_link",
    "target_grasp_pose",
    "target_grasp_mode",
    "target_grasp_width_m",
    "target_key",
    "target_score",
    "target_confidence",
)
SCENE_RANK_BB_KEYS = (
    SCENE_RANK_TARGETS_BB,
    SCENE_RANKED_RESULTS_BB,
    SCENE_RANK_FAILED_TARGETS_BB,
    SCENE_PREOBSERVE_TARGET_BB,
)

OVERVIEW_JOINTS = [-0.0278, -0.0011, 0.0000, -0.3441, 0.0140, -0.0034]


def is_test_mode():
    return rospy.get_param("~test_mode", False)


def get_blackboard():
    return py_trees.blackboard.Blackboard()


def blackboard_get(key, default=None):
    try:
        value = get_blackboard().get(key)
    except Exception:
        return default
    return default if value is None else value


def log_stage(stage_name):
    rospy.loginfo("[Brain] Stage: %s", stage_name)


def clear_target_selection():
    blackboard = get_blackboard()
    for key in TARGET_BB_KEYS:
        blackboard.set(key, None)


def clear_scene_ranking_state():
    blackboard = get_blackboard()
    blackboard.set(SCENE_RANK_TARGETS_BB, [])
    blackboard.set(SCENE_RANKED_RESULTS_BB, [])
    blackboard.set(SCENE_RANK_FAILED_TARGETS_BB, [])
    blackboard.set(SCENE_PREOBSERVE_TARGET_BB, None)


def clear_terminal_failure():
    blackboard = get_blackboard()
    blackboard.set("task_terminal_failure", False)
    blackboard.set("task_terminal_failure_reason", "")
    blackboard.set("task_complete_no_targets", False)
    blackboard.set("task_complete_no_targets_reason", "")


def set_terminal_failure(reason):
    blackboard = get_blackboard()
    blackboard.set("task_terminal_failure", True)
    blackboard.set("task_terminal_failure_reason", reason)


def set_no_targets_complete(reason):
    blackboard = get_blackboard()
    blackboard.set("task_complete_no_targets", True)
    blackboard.set("task_complete_no_targets_reason", reason)
    blackboard.set("task_terminal_failure", False)
    blackboard.set("task_terminal_failure_reason", "")


def get_failed_target_keys():
    values = blackboard_get(FAILED_TARGET_KEYS_BB, [])
    return set(str(item) for item in values)


def set_failed_target_keys(keys):
    get_blackboard().set(FAILED_TARGET_KEYS_BB, sorted(str(item) for item in keys))


def score_object_label(obj_label):
    score_map = {
        "cube": 1000,
        "blue_cube": 100,
        "green_cube": 100,
        "purple_cube": 100,
        "red_cube": 100,
        "red_can": 80,
        "red_bottle": 80,
        "yellow_can": 60,
        "yellow_bottle": 60,
        "green_can": 40,
        "blue_can": 40,
        "blue_bottle": 40,
        "spam": 60,
    }
    return score_map.get(normalize_label(obj_label), 1)


def pick_best_candidate_by_score(candidates):
    best = None
    for candidate in candidates:
        if best is None:
            best = candidate
            continue
        if candidate["score"] > best["score"] or (
            candidate["score"] == best["score"]
            and candidate["confidence"] > best["confidence"]
        ):
            best = candidate
    return best


def build_target_key(label, point_base):
    return "{label}:{x:.2f}:{y:.2f}:{z:.2f}".format(
        label=str(label).lower(),
        x=float(point_base.point.x),
        y=float(point_base.point.y),
        z=float(point_base.point.z),
    )


def is_point_in_polygon_2d(x, y, polygon_xy):
    """Return True if a 2D point is inside or on-edge of a polygon."""
    if not polygon_xy or len(polygon_xy) < 3:
        return False

    inside = False
    n = len(polygon_xy)
    j = n - 1
    eps = 1e-9
    for i in range(n):
        xi, yi = polygon_xy[i]
        xj, yj = polygon_xy[j]

        # On-edge check (tolerant)
        min_x = min(xi, xj) - eps
        max_x = max(xi, xj) + eps
        min_y = min(yi, yj) - eps
        max_y = max(yi, yj) + eps
        dx = xj - xi
        dy = yj - yi
        cross = (x - xi) * dy - (y - yi) * dx
        if abs(cross) <= eps and min_x <= x <= max_x and min_y <= y <= max_y:
            return True

        intersects = ((yi > y) != (yj > y)) and (x < (dx * (y - yi) / (dy + eps) + xi))
        if intersects:
            inside = not inside
        j = i

    return inside


def is_cube_in_scale_zone(
    pt_base,
    trans_base_to_world,
    top_world_z,
    volume_down_m,
    polygon_xy,
):
    if trans_base_to_world is None:
        return False
    if not polygon_xy or len(polygon_xy) < 3:
        return False

    top_z = float(top_world_z)
    down_m = max(0.0, float(volume_down_m))
    bottom_z = top_z - down_m

    pt_world = tf2_geometry_msgs.do_transform_point(pt_base, trans_base_to_world)
    z = float(pt_world.point.z)
    if z < bottom_z or z > top_z:
        return False

    return is_point_in_polygon_2d(
        float(pt_world.point.x),
        float(pt_world.point.y),
        polygon_xy,
    )


def normalize_label(label):
    return str(label).strip().lower()


DEFAULT_OBJECT_LABEL_ALIASES = {
    "class_0": "blue_bottle",
    "class_1": "blue_cube",
    "class_2": "blue_can",
    "class_3": "green_cube",
    "class_4": "green_can",
    "class_5": "purple_cube",
    "class_6": "red_bottle",
    "class_7": "red_cube",
    "class_8": "red_can",
    "class_9": "yellow_bottle",
    "class_10": "yellow_can",
}


def get_object_label_aliases():
    raw_aliases = rospy.get_param("~object_label_aliases", DEFAULT_OBJECT_LABEL_ALIASES)
    if not isinstance(raw_aliases, dict):
        raw_aliases = DEFAULT_OBJECT_LABEL_ALIASES
    return {
        normalize_label(key): normalize_label(value)
        for key, value in raw_aliases.items()
    }


def semantic_object_label(label):
    normalized = normalize_label(label)
    return get_object_label_aliases().get(normalized, normalized)


def get_cube_class_labels():
    return {
        normalize_label(item)
        for item in rospy.get_param(
            "~cube_class_labels",
            ["class_1", "class_3", "class_5", "class_7"],
        )
    }


def canonical_object_label(label):
    normalized = normalize_label(label)
    semantic = semantic_object_label(normalized)
    if normalized in get_cube_class_labels() or semantic == "cube" or semantic.endswith("_cube"):
        return "cube"
    return semantic


def normalize_pick_category(value):
    normalized = normalize_label(value).replace("-", "_")
    if normalized == "cube":
        return "cube"
    if normalized in ("non_cube", "noncube", "other", "others"):
        return "non_cube"
    return ""


def pick_category_for_label(label):
    return "cube" if canonical_object_label(label) == "cube" else "non_cube"


def opposite_pick_category(category):
    normalized = normalize_pick_category(category)
    if normalized == "cube":
        return "non_cube"
    if normalized == "non_cube":
        return "cube"
    return ""


CUBE_BIN_COLOR_MAP = {
    "class_1": "blue",
    "class_7": "blue",
    "blue_cube": "blue",
    "red_cube": "blue",
    "class_3": "green",
    "class_5": "green",
    "green_cube": "green",
    "purple_cube": "green",
}


def cube_bin_color_for_label(label):
    """Return the trash bin color for a cube label after weighing on the scale."""
    normalized = normalize_label(label)
    semantic = semantic_object_label(normalized)
    color = CUBE_BIN_COLOR_MAP.get(normalized) or CUBE_BIN_COLOR_MAP.get(semantic)
    if color:
        return color
    if "blue" in semantic or "red" in semantic:
        return "blue"
    if "green" in semantic or "purple" in semantic:
        return "green"
    return "green"


def rotate_vector_by_quaternion(quat, vector):
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm < 1e-9:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    vx = vector[0]
    vy = vector[1]
    vz = vector[2]

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return rx, ry, rz


def build_direct_yolo_grasp_pose(target_point, target_label, stamp=None, override_yaw=None):
    grasp_min_z = float(rospy.get_param("~direct_grasp_min_z", 0.05))
    grasp_max_z = float(rospy.get_param("~direct_grasp_max_z", 1.20))
    grasp_roll = float(rospy.get_param("~direct_grasp_roll", 3.141592653589793))
    grasp_pitch = float(rospy.get_param("~direct_grasp_pitch", 1.5707963267948966))
    grasp_yaw = float(rospy.get_param("~direct_grasp_yaw", 0.0))
    if override_yaw is not None and math.isfinite(float(override_yaw)):
        grasp_yaw = float(override_yaw)

    qx, qy, qz, qw = quaternion_from_euler(grasp_roll, grasp_pitch, grasp_yaw)
    grasp_orientation = Quaternion(qx, qy, qz, qw)

    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = target_point.header.frame_id or "base_link"
    grasp_pose.header.stamp = stamp if stamp is not None else rospy.Time.now()
    grasp_pose.pose.orientation = grasp_orientation

    grasp_offset_x = float(rospy.get_param("~direct_grasp_offset_x", 0.0))
    grasp_offset_y = float(rospy.get_param("~direct_grasp_offset_y", 0.0))
    grasp_offset_z = float(rospy.get_param("~direct_grasp_offset_z", 0.0))
    grasp_local_x_offset = float(rospy.get_param("~direct_grasp_local_x_offset", 0.15))
    local_dx, local_dy, local_dz = rotate_vector_by_quaternion(
        grasp_orientation,
        (grasp_local_x_offset, 0.0, 0.0),
    )

    canonical_label = canonical_object_label(target_label)
    cube_offset_mode = normalize_label(
        rospy.get_param("~cube_direct_grasp_offset_mode", "local_x")
    ) or "local_x"
    if canonical_label == "cube" and cube_offset_mode == "base":
        base_offset_x = float(rospy.get_param("~cube_direct_grasp_base_x_offset", 0.15))
        base_offset_y = float(rospy.get_param("~cube_direct_grasp_base_y_offset", 0.0))
        base_offset_z = float(rospy.get_param("~cube_direct_grasp_base_z_offset", 0.0))

        raw_z = float(target_point.point.z) + base_offset_z
        clamped_z = max(grasp_min_z, min(grasp_max_z, raw_z))
        grasp_pose.pose.position.x = float(target_point.point.x) + base_offset_x
        grasp_pose.pose.position.y = float(target_point.point.y) + base_offset_y
        grasp_pose.pose.position.z = clamped_z
        metadata = {
            "offset_mode": "cube_base",
            "offset_x": base_offset_x,
            "offset_y": base_offset_y,
            "offset_z": base_offset_z,
            "local_x_offset": 0.0,
        }
        return grasp_pose, metadata, raw_z, clamped_z

    raw_z = float(target_point.point.z) + grasp_offset_z
    clamped_z = max(grasp_min_z, min(grasp_max_z, raw_z))
    grasp_pose.pose.position.x = float(target_point.point.x) + grasp_offset_x + local_dx
    grasp_pose.pose.position.y = float(target_point.point.y) + grasp_offset_y + local_dy
    grasp_pose.pose.position.z = clamped_z + local_dz
    metadata = {
        "offset_mode": "local_x",
        "offset_x": grasp_offset_x,
        "offset_y": grasp_offset_y,
        "offset_z": grasp_offset_z,
        "local_x_offset": grasp_local_x_offset,
    }
    return grasp_pose, metadata, raw_z, clamped_z


def blacklist_current_target(reason):
    blackboard = get_blackboard()
    target_key = blackboard_get("target_key")
    target_object = blackboard_get("target_object", {}) or {}
    target_label = (
        target_object.get("name", "unknown")
        if isinstance(target_object, dict)
        else str(target_object)
    )

    if target_key:
        failed = get_failed_target_keys()
        failed.add(str(target_key))
        set_failed_target_keys(failed)

    rospy.logwarn(
        "[Brain] Target failed, switching to another candidate | target=%s reason=%s",
        target_label,
        reason,
    )
    blackboard.set("last_target_failure_reason", reason)
    clear_target_selection()


def prepare_retry_from_overview(reason, blacklist=False):
    blackboard = get_blackboard()
    if blacklist:
        blacklist_current_target(reason)
    else:
        rospy.logwarn("[Brain] Recovery requested | reason=%s", reason)
        blackboard.set("last_target_failure_reason", reason)
        clear_target_selection()

    blackboard.set("overview_done", False)
    blackboard.set("holding_object", False)
    blackboard.set("executed_trajectory", None)
    blackboard.set("task_retry_requested", True)
    blackboard.set("task_retry_reason", reason)
    clear_scene_ranking_state()
    clear_terminal_failure()


def prepare_retry_without_overview(reason, blacklist=False):
    blackboard = get_blackboard()
    if blacklist:
        blacklist_current_target(reason)
    else:
        rospy.logwarn("[Brain] Recovery requested without overview | reason=%s", reason)
        blackboard.set("last_target_failure_reason", reason)
        clear_target_selection()

    blackboard.set("overview_done", True)
    blackboard.set("task_retry_requested", True)
    blackboard.set("task_retry_reason", reason)
    clear_scene_ranking_state()
    clear_terminal_failure()


def prepare_next_pick_cycle():
    blackboard = get_blackboard()
    blackboard.set("overview_done", False)
    blackboard.set("holding_object", False)
    blackboard.set("executed_trajectory", None)
    blackboard.set("placed_bin_color", "")
    blackboard.set("last_target_failure_reason", "")
    blackboard.set("task_retry_requested", False)
    blackboard.set("task_retry_reason", "")
    set_failed_target_keys([])
    clear_target_selection()
    clear_scene_ranking_state()
    clear_terminal_failure()


def prepare_next_scene_ranking_cycle():
    blackboard = get_blackboard()
    blackboard.set("overview_done", True)
    blackboard.set("holding_object", False)
    blackboard.set("executed_trajectory", None)
    blackboard.set("placed_bin_color", "")
    blackboard.set("last_target_failure_reason", "")
    blackboard.set("task_retry_requested", False)
    blackboard.set("task_retry_reason", "")
    set_failed_target_keys([])
    clear_target_selection()
    clear_scene_ranking_state()
    clear_terminal_failure()


def is_scene_wide_non_cube_ranking_enabled():
    return bool(rospy.get_param("~scene_wide_non_cube_ranking", False))


def is_debug_non_cube_only_enabled():
    return bool(rospy.get_param("~debug_non_cube_only", False))


def should_execute_after_scene_ranking():
    return bool(rospy.get_param("~execute_after_scene_ranking", False))


def pointcloud2_to_xyzl(msg, label_field_name="label"):
    field_names = [field.name for field in msg.fields]
    label_field = None
    if label_field_name in field_names:
        label_field = label_field_name
    elif "l" in field_names:
        label_field = "l"

    points = []
    labels = []
    if label_field is None:
        for x, y, z in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            points.append((float(x), float(y), float(z)))
        return np.asarray(points, dtype=np.float32), None

    for x, y, z, label in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", label_field)):
        points.append((float(x), float(y), float(z)))
        labels.append(int(label))
    return np.asarray(points, dtype=np.float32), np.asarray(labels, dtype=np.int32)


def estimate_simple_topdown_grasp(points, target_label, frame_id="base_link"):
    if points is None or len(points) < int(rospy.get_param("~simple_grasp_min_points", 25)):
        return None, "Not enough target points for simple grasp estimation"

    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        return None, "Invalid point shape for simple grasp estimation"

    p05 = np.percentile(pts, 5, axis=0)
    p50 = np.percentile(pts, 50, axis=0)
    p95 = np.percentile(pts, 95, axis=0)
    extents = p95 - p05
    height = float(max(extents[2], 1e-4))
    width_x = float(max(extents[0], 1e-4))
    width_y = float(max(extents[1], 1e-4))
    planar_major = max(width_x, width_y)
    planar_minor = min(width_x, width_y)

    center_x = float(p50[0])
    center_y = float(p50[1])
    z_min = float(p05[2])
    z_max = float(p95[2])

    grasp_height_ratio = float(rospy.get_param("~simple_grasp_height_ratio", 0.60))
    grasp_top_margin = float(rospy.get_param("~simple_grasp_top_margin", 0.03))
    raw_z = min(z_max - grasp_top_margin, z_min + grasp_height_ratio * height)

    grasp_min_z = float(rospy.get_param("~direct_grasp_min_z", 0.05))
    grasp_max_z = float(rospy.get_param("~direct_grasp_max_z", 1.20))
    grasp_z = max(grasp_min_z, min(grasp_max_z, raw_z))

    xy_centered = pts[:, :2] - np.array([[center_x, center_y]], dtype=np.float32)
    yaw = 0.0
    if len(xy_centered) >= 3:
        cov = np.cov(xy_centered.T)
        try:
            eigvals, eigvecs = np.linalg.eigh(cov)
            major_vec = eigvecs[:, int(np.argmax(eigvals))]
            yaw = float(math.atan2(float(major_vec[1]), float(major_vec[0])))
        except Exception:
            yaw = 0.0

    canonical_label = canonical_object_label(target_label)
    if canonical_label.endswith("bottle") or canonical_label.endswith("can"):
        yaw += float(rospy.get_param("~simple_grasp_yaw_offset", 0.0))

    roll = float(rospy.get_param("~direct_grasp_roll", 3.141592653589793))
    pitch = float(rospy.get_param("~direct_grasp_pitch", 1.5707963267948966))
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

    pose = PoseStamped()
    pose.header.frame_id = frame_id or "base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = Point(x=center_x, y=center_y, z=grasp_z)
    pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    grasp_width = float(planar_minor + 0.02)
    min_width = float(rospy.get_param("~simple_grasp_min_width", 0.02))
    max_width = float(rospy.get_param("~simple_grasp_max_width", 0.10))
    grasp_width = max(min_width, min(max_width, grasp_width))

    quality = float(min(1.0, 0.4 + 0.2 * min(len(pts) / 200.0, 1.0) + 0.2 * min(height / 0.12, 1.0)))

    metadata = {
        "point_count": int(len(pts)),
        "height": float(height),
        "width_x": float(width_x),
        "width_y": float(width_y),
        "planar_major": float(planar_major),
        "planar_minor": float(planar_minor),
        "raw_z": float(raw_z),
        "grasp_z": float(grasp_z),
        "yaw": float(yaw),
        "estimated_width": float(grasp_width),
        "quality": float(quality),
    }
    return {"pose": pose, "quality": quality, "width": grasp_width, "metadata": metadata}, ""


def compute_grasp_geometry_scores(pose_stamped, quality):
    approach_axis = rotate_vector_by_quaternion(pose_stamped.pose.orientation, (1.0, 0.0, 0.0))
    top_score = -float(approach_axis[2])

    object_x = float(pose_stamped.pose.position.x)
    object_y = float(pose_stamped.pose.position.y)
    object_z = float(pose_stamped.pose.position.z)
    approach_xy_norm = math.hypot(float(approach_axis[0]), float(approach_axis[1]))
    object_xy_norm = math.hypot(object_x, object_y)

    if approach_xy_norm > 1e-6 and object_xy_norm > 1e-6:
        robot_side_score = (
            (float(approach_axis[0]) / approach_xy_norm) * (object_x / object_xy_norm)
            + (float(approach_axis[1]) / approach_xy_norm) * (object_y / object_xy_norm)
        )
    else:
        robot_side_score = 0.0

    planar_distance = object_xy_norm
    return {
        "top_score": top_score,
        "robot_side_score": robot_side_score,
        "quality": float(quality),
        "planar_distance": planar_distance,
        "pose_z": object_z,
    }


def grasp_geometry_rank_key(metrics):
    return (
        -float(metrics["top_score"]),
        -float(metrics["robot_side_score"]),
        -float(metrics["quality"]),
        float(metrics["planar_distance"]),
    )


class MoveToOverviewBehavior(py_trees.behaviour.Behaviour):
    """Move to the fixed overview joint pose once, then open the gripper once."""

    def __init__(self, name="MoveToOverview"):
        super().__init__(name)
        self.motion_cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
        self.motion_result_sub = rospy.Subscriber(
            "/motion/result", GraspResult, self._motion_result_callback
        )
        self.gripper_cmd_pub = rospy.Publisher("/gripper/command", String, queue_size=1)
        self.yolo_enable_topic = rospy.get_param(
            "~yolo_enable_topic",
            "/perception/yolo26_seg_enabled",
        )
        self.yolo_enable_pub = rospy.Publisher(
            self.yolo_enable_topic,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.gripper_release_srv = rospy.ServiceProxy("/gripper/release", Trigger)
        self.clear_target_pose_srv = rospy.ServiceProxy("/path_planning/clear_target_pose", Trigger)
        self.command_sent = False
        self.last_result = None
        self.release_done = False

    def _release_response_usable(self, message):
        text = str(message or "")
        lowered = text.lower()
        if "partial-open" in lowered or "accepted as partial-open" in lowered:
            return True

        min_closed_error = float(rospy.get_param("~release_min_closed_error", 0.25))
        match = re.search(r"goal error\s+([-\d\.eE]+)", text)
        if match is None:
            return False
        try:
            goal_error = float(match.group(1))
        except ValueError:
            return False
        return abs(goal_error) >= min_closed_error

    def _motion_result_callback(self, msg):
        self.last_result = msg

    def initialise(self):
        if blackboard_get("overview_done", False):
            return
        self.command_sent = False
        self.last_result = None
        self.release_done = False
        log_stage("MoveToOverview")
        self.yolo_enable_pub.publish(Bool(data=False))
        rospy.loginfo("[Brain] Stage action: disable YOLO perception")
        self._clear_path_planning_target_pose()

    def _clear_path_planning_target_pose(self):
        try:
            rospy.wait_for_service("/path_planning/clear_target_pose", timeout=0.5)
            response = self.clear_target_pose_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: cleared stale prm_target_pose")
            else:
                rospy.logwarn(
                    "[Brain] Failed to clear prm_target_pose before detection: %s",
                    response.message,
                )
        except (rospy.ROSException, rospy.ServiceException):
            rospy.logwarn_throttle(
                5.0,
                "[Brain] /path_planning/clear_target_pose unavailable; continuing without clearing target pose",
            )

    def _open_gripper(self):
        try:
            rospy.wait_for_service("/gripper/release", timeout=0.5)
            response = self.gripper_release_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: open gripper at poseToTakePics")
                return True
            if self._release_response_usable(response.message):
                rospy.logwarn(
                    "[Brain] Open gripper accepted as usable even though release returned failure: %s",
                    response.message,
                )
                return True
            rospy.logerr("[Brain] Failed to open gripper at poseToTakePics: %s", response.message)
            return False
        except (rospy.ROSException, rospy.ServiceException):
            if self.gripper_cmd_pub.get_num_connections() > 0:
                self.gripper_cmd_pub.publish(String(data="release"))
                rospy.logwarn(
                    "[Brain] /gripper/release service unavailable, published release command instead"
                )
                return True
            rospy.logerr("[Brain] Gripper release interface unavailable")
            return False

    def update(self):
        blackboard = get_blackboard()
        if blackboard_get("overview_done", False):
            return Status.SUCCESS

        if self.motion_cmd_pub.get_num_connections() == 0:
            return Status.RUNNING

        if not self.command_sent:
            clear_terminal_failure()
            cmd = MotionCommand()
            cmd.command_type = MotionCommand.MOVE_TO_JOINT
            cmd.joint_positions = OVERVIEW_JOINTS
            cmd.max_velocity = 1.0
            cmd.max_acceleration = 1.0
            cmd.collision_check = True
            self.motion_cmd_pub.publish(cmd)
            rospy.loginfo(
                "[Brain] Stage action: move to overview joints [%s]",
                ", ".join(f"{joint:.4f}" for joint in OVERVIEW_JOINTS),
            )
            self.command_sent = True
            return Status.RUNNING

        if self.last_result is None:
            return Status.RUNNING

        if self.last_result.status != GraspResult.SUCCESS:
            reason = self.last_result.message or "poseToTakePics motion failed"
            rospy.logerr("[Brain] Overview move failed: %s", reason)
            set_terminal_failure(reason)
            return Status.FAILURE

        if not self.release_done:
            if not self._open_gripper():
                reason = "Failed to open gripper at poseToTakePics"
                set_terminal_failure(reason)
                return Status.FAILURE
            self.release_done = True

        blackboard.set("overview_done", True)
        rospy.loginfo("[Brain] Stage complete: MoveToOverview")
        return Status.SUCCESS


class EvaluateTargetsBehavior(py_trees.behaviour.Behaviour):
    """Receive YOLO detections, transform them, and select direct or scene-ranked targets."""

    def __init__(self, name="EvaluateTargets"):
        super().__init__(name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.latest_detections = []
        self.latest_frame_id = ""
        self.latest_stamp = rospy.Time(0)
        self.sub = None
        self.mock_done = False
        self.yolo_enable_topic = rospy.get_param(
            "~yolo_enable_topic",
            "/perception/yolo26_seg_enabled",
        )
        self.yolo_enable_pub = rospy.Publisher(
            self.yolo_enable_topic,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.wait_start_time = None
        self.exclude_cubes_on_scale = bool(rospy.get_param("~exclude_cubes_on_scale", True))
        self.scale_filter_world_frame = str(
            rospy.get_param("~scale_filter_world_frame", "world")
        )
        self.scale_filter_corner_output_frame = str(
            rospy.get_param("~scale_filter_corner_output_frame", "base_link")
        )
        self.scale_filter_min_world_z = float(
            rospy.get_param("~scale_filter_min_world_z", 0.54)
        )
        self.scale_filter_volume_down_m = float(
            rospy.get_param("~scale_filter_volume_down_m", 0.80)
        )
        polygon_default = [
            [0.486, 0.304],
            [0.627, 0.185],
            [0.834, 0.432],
            [0.693, 0.550],
        ]
        polygon_raw = rospy.get_param("~scale_filter_polygon_world_xy", polygon_default)
        self.scale_filter_polygon_world_xy = []
        if isinstance(polygon_raw, list):
            for item in polygon_raw:
                if isinstance(item, (list, tuple)) and len(item) >= 2:
                    self.scale_filter_polygon_world_xy.append((float(item[0]), float(item[1])))

        self.publish_scale_filter_corners = bool(
            rospy.get_param("~publish_scale_filter_corners", True)
        )
        self.scale_filter_corner_world_z = float(
            rospy.get_param("~scale_filter_corner_world_z", 0.275)
        )
        self.scale_filter_corners_topic = str(
            rospy.get_param("~scale_filter_corners_topic", "/brain/scale_filter_corners")
        )
        self.scale_filter_corners_marker_topic = str(
            rospy.get_param(
                "~scale_filter_corners_marker_topic",
                "/brain/scale_filter_corners_markers",
            )
        )
        self.scale_filter_corners_pub = rospy.Publisher(
            self.scale_filter_corners_topic,
            PoseArray,
            queue_size=1,
            latch=True,
        )
        self.scale_filter_corners_marker_pub = rospy.Publisher(
            self.scale_filter_corners_marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self._published_scale_filter_corners = False

    def setup(self, timeout=None):
        if is_test_mode():
            return True
        self.sub = rospy.Subscriber(
            "/perception/yolo26_seg_detections",
            String,
            self._yolo_callback,
        )
        self._publish_scale_filter_corners_visualization(force=True)
        return True

    def _yolo_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except (TypeError, ValueError):
            return

        if not isinstance(detections, list):
            return

        self.latest_detections = detections
        blackboard = get_blackboard()
        blackboard.set("detected_objects", list(detections))

        if not detections:
            return

        first = detections[0]
        self.latest_frame_id = str(first.get("frame_id", "")) or rospy.get_param(
            "~perception_frame_id", "camera_rgb_optical_frame"
        )
        blackboard.set(LATEST_YOLO_FRAME_ID_BB, self.latest_frame_id)
        stamp_dict = first.get("gazebo_stamp", {})
        secs = int(stamp_dict.get("secs", 0))
        nsecs = int(stamp_dict.get("nsecs", 0))
        self.latest_stamp = (
            rospy.Time(secs=secs, nsecs=nsecs) if (secs or nsecs) else rospy.Time(0)
        )
        blackboard.set(LATEST_YOLO_STAMP_BB, self.latest_stamp)
        blackboard.set(LATEST_YOLO_RECV_TIME_BB, rospy.Time.now())

    def _score_object(self, obj_label):
        return score_object_label(obj_label)

    @staticmethod
    def _pick_best_candidate(candidates):
        return pick_best_candidate_by_score(candidates)

    def _store_selected_target(self, best_entry):
        clear_terminal_failure()
        clear_scene_ranking_state()
        blackboard = get_blackboard()
        blackboard.set("target_object", best_entry["target_object"])
        blackboard.set("target_point_base_link", best_entry["target_point_base"])
        blackboard.set("target_key", best_entry["target_key"])
        blackboard.set("target_score", best_entry["score"])
        blackboard.set("target_confidence", best_entry["confidence"])

    def _store_scene_rank_targets(self, scene_targets):
        clear_terminal_failure()
        clear_target_selection()
        clear_scene_ranking_state()
        blackboard = get_blackboard()
        blackboard.set(SCENE_RANK_TARGETS_BB, list(scene_targets))
        blackboard.set(SCENE_RANKED_RESULTS_BB, [])
        blackboard.set(SCENE_RANK_FAILED_TARGETS_BB, [])
        blackboard.set(SCENE_PREOBSERVE_TARGET_BB, pick_best_candidate_by_score(scene_targets))

    def _publish_scale_filter_corners_visualization(self, force=False):
        if not self.publish_scale_filter_corners:
            return
        if self._published_scale_filter_corners and not force:
            return
        if len(self.scale_filter_polygon_world_xy) < 3:
            return

        publish_frame = self.scale_filter_corner_output_frame or self.scale_filter_world_frame
        top_world_points = [
            Point(
                x=float(x),
                y=float(y),
                z=float(self.scale_filter_corner_world_z),
            )
            for x, y in self.scale_filter_polygon_world_xy
        ]
        bottom_world_points = [
            Point(
                x=float(point.x),
                y=float(point.y),
                z=float(point.z) - float(self.scale_filter_volume_down_m),
            )
            for point in top_world_points
        ]
        world_points = top_world_points + bottom_world_points

        transformed_points = []
        if publish_frame == self.scale_filter_world_frame:
            transformed_points = [
                Point(x=float(point.x), y=float(point.y), z=float(point.z))
                for point in world_points
            ]
        else:
            try:
                world_to_publish = self.tf_buffer.lookup_transform(
                    publish_frame,
                    self.scale_filter_world_frame,
                    rospy.Time(0),
                    rospy.Duration(0.3),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "[Brain] Cannot publish scale corners in %s (from %s): %s",
                    publish_frame,
                    self.scale_filter_world_frame,
                    exc,
                )
                return

            for point in world_points:
                pt_world = PointStamped()
                pt_world.header.frame_id = self.scale_filter_world_frame
                pt_world.header.stamp = rospy.Time(0)
                pt_world.point.x = float(point.x)
                pt_world.point.y = float(point.y)
                pt_world.point.z = float(point.z)
                pt_publish = tf2_geometry_msgs.do_transform_point(pt_world, world_to_publish)
                transformed_points.append(
                    Point(
                        x=float(pt_publish.point.x),
                        y=float(pt_publish.point.y),
                        z=float(pt_publish.point.z),
                    )
                )

        n_top = len(top_world_points)
        if len(transformed_points) != 2 * n_top:
            return

        top_points = transformed_points[:n_top]
        bottom_points = transformed_points[n_top:]

        poses = PoseArray()
        poses.header.frame_id = publish_frame
        poses.header.stamp = rospy.Time.now()
        for point in transformed_points:
            pose = PoseStamped().pose
            pose.position.x = float(point.x)
            pose.position.y = float(point.y)
            pose.position.z = float(point.z)
            pose.orientation.w = 1.0
            poses.poses.append(pose)
        self.scale_filter_corners_pub.publish(poses)

        markers = MarkerArray()

        corners = Marker()
        corners.header.frame_id = publish_frame
        corners.header.stamp = poses.header.stamp
        corners.ns = "scale_filter"
        corners.id = 0
        corners.type = Marker.SPHERE_LIST
        corners.action = Marker.ADD
        corners.scale.x = 0.02
        corners.scale.y = 0.02
        corners.scale.z = 0.02
        corners.color = ColorRGBA(1.0, 0.2, 0.2, 1.0)
        corners.points = list(transformed_points)
        markers.markers.append(corners)

        top_outline = Marker()
        top_outline.header.frame_id = publish_frame
        top_outline.header.stamp = poses.header.stamp
        top_outline.ns = "scale_filter"
        top_outline.id = 1
        top_outline.type = Marker.LINE_STRIP
        top_outline.action = Marker.ADD
        top_outline.scale.x = 0.008
        top_outline.color = ColorRGBA(1.0, 0.8, 0.0, 1.0)
        top_outline.pose.orientation.w = 1.0
        for point in top_points:
            top_outline.points.append(
                Point(x=float(point.x), y=float(point.y), z=float(point.z))
            )
        top_outline.points.append(
            Point(
                x=float(top_points[0].x),
                y=float(top_points[0].y),
                z=float(top_points[0].z),
            )
        )
        markers.markers.append(top_outline)

        bottom_outline = Marker()
        bottom_outline.header.frame_id = publish_frame
        bottom_outline.header.stamp = poses.header.stamp
        bottom_outline.ns = "scale_filter"
        bottom_outline.id = 2
        bottom_outline.type = Marker.LINE_STRIP
        bottom_outline.action = Marker.ADD
        bottom_outline.scale.x = 0.008
        bottom_outline.color = ColorRGBA(0.2, 0.8, 1.0, 1.0)
        bottom_outline.pose.orientation.w = 1.0
        for point in bottom_points:
            bottom_outline.points.append(
                Point(x=float(point.x), y=float(point.y), z=float(point.z))
            )
        bottom_outline.points.append(
            Point(
                x=float(bottom_points[0].x),
                y=float(bottom_points[0].y),
                z=float(bottom_points[0].z),
            )
        )
        markers.markers.append(bottom_outline)

        vertical_edges = Marker()
        vertical_edges.header.frame_id = publish_frame
        vertical_edges.header.stamp = poses.header.stamp
        vertical_edges.ns = "scale_filter"
        vertical_edges.id = 3
        vertical_edges.type = Marker.LINE_LIST
        vertical_edges.action = Marker.ADD
        vertical_edges.scale.x = 0.006
        vertical_edges.color = ColorRGBA(0.7, 1.0, 0.2, 1.0)
        vertical_edges.pose.orientation.w = 1.0
        for top_point, bottom_point in zip(top_points, bottom_points):
            vertical_edges.points.append(
                Point(x=float(top_point.x), y=float(top_point.y), z=float(top_point.z))
            )
            vertical_edges.points.append(
                Point(
                    x=float(bottom_point.x),
                    y=float(bottom_point.y),
                    z=float(bottom_point.z),
                )
            )
        markers.markers.append(vertical_edges)

        self.scale_filter_corners_marker_pub.publish(markers)
        self._published_scale_filter_corners = True

    def initialise(self):
        self._publish_scale_filter_corners_visualization()
        self.mock_done = False
        self.latest_detections = []
        self.latest_frame_id = ""
        self.latest_stamp = rospy.Time(0)
        self.wait_start_time = rospy.Time.now()
        clear_target_selection()
        clear_scene_ranking_state()
        log_stage("EvaluateTargets")
        self.yolo_enable_pub.publish(Bool(data=True))
        rospy.loginfo("[Brain] Stage action: enable YOLO perception")

    def _lookup_transform(self, frame_id, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                stamp,
                rospy.Duration(0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                rospy.Time(0),
                rospy.Duration(0.5),
            )

    def update(self):
        if is_test_mode():
            if not self.mock_done:
                mock_point = PointStamped()
                mock_point.header.frame_id = "base_link"
                mock_point.point.x = 0.35
                mock_point.point.y = -0.10
                mock_point.point.z = 0.30
                blackboard = get_blackboard()
                blackboard.set("target_object", {"name": "mock_target", "confidence": 1.0})
                blackboard.set("target_point_base_link", mock_point)
                blackboard.set("target_key", build_target_key("mock_target", mock_point))
                self.mock_done = True
            return Status.SUCCESS

        no_target_timeout = float(rospy.get_param("~no_target_timeout", 3.0))
        elapsed = 0.0
        if self.wait_start_time is not None:
            elapsed = max(0.0, (rospy.Time.now() - self.wait_start_time).to_sec())

        if not self.latest_detections:
            if elapsed >= no_target_timeout:
                self.yolo_enable_pub.publish(Bool(data=False))
                reason = "No YOLO detections observed for %.1fs" % elapsed
                rospy.loginfo("[Brain] %s. Treating table as empty.", reason)
                set_no_targets_complete(reason)
                return Status.FAILURE
            return Status.RUNNING

        failed_target_keys = get_failed_target_keys()
        preferred_label = normalize_label(rospy.get_param("~preferred_target_label", ""))
        defer_cube_until_others = bool(rospy.get_param("~defer_cube_until_others", True))
        alternate_pick_categories = bool(rospy.get_param("~alternate_pick_categories", True))
        scene_wide_non_cube_ranking = is_scene_wide_non_cube_ranking_enabled()
        debug_non_cube_only = is_debug_non_cube_only_enabled()
        frame_id = self.latest_frame_id or rospy.get_param(
            "~perception_frame_id", "camera_rgb_optical_frame"
        )
        latest_labels = [
            str(item.get("name", "unknown"))
            for item in self.latest_detections
            if isinstance(item, dict)
        ]

        try:
            trans = self._lookup_transform(frame_id, self.latest_stamp)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(2.0, "[Brain] Waiting for TF to base_link: %s", exc)
            return Status.RUNNING

        trans_base_to_world = None
        if self.exclude_cubes_on_scale and len(self.scale_filter_polygon_world_xy) >= 3:
            try:
                trans_base_to_world = self.tf_buffer.lookup_transform(
                    self.scale_filter_world_frame,
                    "base_link",
                    rospy.Time(0),
                    rospy.Duration(0.2),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "[Brain] Scale filter disabled this cycle (TF base_link->%s unavailable): %s",
                    self.scale_filter_world_frame,
                    exc,
                )

        selectable_candidates = []

        for obj in self.latest_detections:
            if not isinstance(obj, dict):
                continue

            center = obj.get("center_3d")
            if not isinstance(center, dict):
                continue

            pt_camera = PointStamped()
            pt_camera.header.frame_id = frame_id
            pt_camera.header.stamp = trans.header.stamp
            pt_camera.point.x = float(center.get("x", 0.0))
            pt_camera.point.y = float(center.get("y", 0.0))
            pt_camera.point.z = float(center.get("z", 0.0))

            pt_base = tf2_geometry_msgs.do_transform_point(pt_camera, trans)
            label = str(obj.get("name", "unknown"))
            semantic_label = semantic_object_label(label)
            canonical_label = canonical_object_label(label)
            pick_category = pick_category_for_label(label)
            if debug_non_cube_only and pick_category == "cube":
                continue

            in_scale = pick_category == "cube" and is_cube_in_scale_zone(
                pt_base,
                trans_base_to_world,
                self.scale_filter_corner_world_z,
                self.scale_filter_volume_down_m,
                self.scale_filter_polygon_world_xy,
            )

            if in_scale and not blackboard_get(CUBE_ON_SCALE_BB, False):
                rospy.loginfo_throttle(
                    1.0,
                    "[Brain] Skip cube in scale zone | label=%s world_frame=%s",
                    label,
                    self.scale_filter_world_frame,
                )
                continue

            if preferred_label and canonical_label != preferred_label and semantic_label != preferred_label:
                continue
            target_key = build_target_key(label, pt_base)
            if target_key in failed_target_keys:
                continue

            confidence = float(obj.get("confidence", 0.0))
            score = self._score_object(canonical_label)
            selectable_candidates.append(
                {
                    "target_object": obj,
                    "target_point_base": pt_base,
                    "target_key": target_key,
                    "label": label,
                    "semantic_label": semantic_label,
                    "canonical_label": canonical_label,
                    "pick_category": pick_category,
                    "score": score,
                    "confidence": confidence,
                    "in_scale_zone": in_scale,
                }
            )

        best_entry = None
        scene_rank_targets = []
        cube_on_scale = blackboard_get(CUBE_ON_SCALE_BB, False)
        if cube_on_scale:
            scale_cubes = [
                c for c in selectable_candidates
                if c["pick_category"] == "cube" and c.get("in_scale_zone", False)
            ]
            if scale_cubes:
                best_entry = self._pick_best_candidate(scale_cubes)
                rospy.loginfo(
                    "[Brain] Scale retrieval: found cube on scale | label=%s",
                    best_entry.get("label", "unknown"),
                )
            else:
                blackboard = get_blackboard()
                blackboard.set(CUBE_ON_SCALE_BB, False)
                blackboard.set(CUBE_ON_SCALE_LABEL_BB, "")
                blackboard.set(CUBE_ON_SCALE_BIN_COLOR_BB, "")
                rospy.logwarn(
                    "[Brain] Scale retrieval: no cube detected on scale, clearing state and resuming normal cube-first pickup"
                )
                normal_candidates = [
                    c for c in selectable_candidates if not c.get("in_scale_zone", False)
                ]
                cube_candidates = [c for c in normal_candidates if c["pick_category"] == "cube"]
                non_cube_candidates = [c for c in normal_candidates if c["pick_category"] == "non_cube"]
                if cube_candidates:
                    best_entry = self._pick_best_candidate(cube_candidates)
                elif non_cube_candidates:
                    if scene_wide_non_cube_ranking:
                        scene_rank_targets = list(non_cube_candidates)
                    else:
                        best_entry = self._pick_best_candidate(non_cube_candidates)
                else:
                    best_entry = self._pick_best_candidate(normal_candidates)
        elif preferred_label:
            if scene_wide_non_cube_ranking:
                non_cube_candidates = [
                    candidate
                    for candidate in selectable_candidates
                    if candidate["pick_category"] == "non_cube"
                ]
                if non_cube_candidates:
                    scene_rank_targets = list(non_cube_candidates)
                else:
                    best_entry = self._pick_best_candidate(selectable_candidates)
            else:
                best_entry = self._pick_best_candidate(selectable_candidates)
        elif selectable_candidates:
            cube_candidates = [
                candidate
                for candidate in selectable_candidates
                if candidate["pick_category"] == "cube"
            ]
            non_cube_candidates = [
                candidate
                for candidate in selectable_candidates
                if candidate["pick_category"] == "non_cube"
            ]

            if debug_non_cube_only:
                if scene_wide_non_cube_ranking and non_cube_candidates:
                    scene_rank_targets = list(non_cube_candidates)
                else:
                    best_entry = self._pick_best_candidate(non_cube_candidates)
            elif cube_candidates:
                best_entry = self._pick_best_candidate(cube_candidates)
                rospy.loginfo(
                    "[Brain] Cube-first strategy: %d cube(s) remaining, picking cube",
                    len(cube_candidates),
                )
            elif non_cube_candidates:
                rospy.loginfo(
                    "[Brain] Cube-first strategy: no cubes left, switching to non_cube (%d remaining)",
                    len(non_cube_candidates),
                )
                if scene_wide_non_cube_ranking:
                    scene_rank_targets = list(non_cube_candidates)
                else:
                    best_entry = self._pick_best_candidate(non_cube_candidates)
            else:
                best_entry = self._pick_best_candidate(selectable_candidates)

        if scene_rank_targets:
            self._store_scene_rank_targets(scene_rank_targets)
            self.yolo_enable_pub.publish(Bool(data=False))
            rospy.loginfo("[Brain] Stage action: disable YOLO perception")
            preview_labels = ", ".join(
                f"{entry['semantic_label']}@{entry['confidence']:.3f}"
                for entry in scene_rank_targets[:5]
            )
            rospy.loginfo(
                "[Brain] Stage complete: EvaluateTargets | scene_non_cube_candidates=%d labels=%s",
                len(scene_rank_targets),
                preview_labels or "(none)",
            )
            return Status.SUCCESS

        if best_entry is None:
            if preferred_label:
                rospy.logwarn_throttle(
                    2.0,
                    "[Brain] Waiting for preferred target '%s' | visible=%s blacklisted=%d",
                    preferred_label,
                    ", ".join(latest_labels),
                    len(failed_target_keys),
                )
            else:
                rospy.logwarn_throttle(
                    2.0,
                    "[Brain] No selectable YOLO target right now | labels=%s blacklisted=%d",
                    ", ".join(latest_labels),
                    len(failed_target_keys),
                )
            if elapsed >= no_target_timeout:
                self.yolo_enable_pub.publish(Bool(data=False))
                if preferred_label:
                    reason = (
                        "No selectable target '%s' observed for %.1fs"
                        % (preferred_label, elapsed)
                    )
                else:
                    reason = "No selectable YOLO target observed for %.1fs" % elapsed
                rospy.loginfo("[Brain] %s. Treating table as empty.", reason)
                set_no_targets_complete(reason)
                return Status.FAILURE
            return Status.RUNNING

        self._store_selected_target(best_entry)
        selected_label = str(best_entry["label"])
        selected_semantic_label = str(best_entry["semantic_label"])
        selected_pick_category = str(best_entry["pick_category"])

        preview_pose, preview_metadata, _, _ = build_direct_yolo_grasp_pose(
            best_entry["target_point_base"],
            selected_label,
            stamp=rospy.Time.now(),
        )

        self.yolo_enable_pub.publish(Bool(data=False))
        rospy.loginfo("[Brain] Stage action: disable YOLO perception")
        rospy.loginfo(
            "[Brain] Stage complete: EvaluateTargets | selected=%s semantic=%s category=%s score=%s confidence=%.3f preview=(%.3f, %.3f, %.3f) offset_mode=%s offset=(%.3f, %.3f, %.3f) local_x_offset=%.3f",
            selected_label,
            selected_semantic_label,
            selected_pick_category,
            best_entry["score"],
            best_entry["confidence"],
            preview_pose.pose.position.x,
            preview_pose.pose.position.y,
            preview_pose.pose.position.z,
            preview_metadata["offset_mode"],
            preview_metadata["offset_x"],
            preview_metadata["offset_y"],
            preview_metadata["offset_z"],
            preview_metadata["local_x_offset"],
        )
        return Status.SUCCESS


class MoveToNonCubePreObserveBehavior(py_trees.behaviour.Behaviour):
    """Move to a YOLO-derived non-cube pre-observation pose before scene-wide GraspNet ranking."""

    def __init__(self, name="MoveToNonCubePreObserve"):
        super().__init__(name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.motion_cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
        self.motion_result_sub = rospy.Subscriber(
            "/motion/result", GraspResult, self._motion_result_callback
        )
        self.yolo_enable_topic = rospy.get_param(
            "~yolo_enable_topic",
            "/perception/yolo26_seg_enabled",
        )
        self.yolo_enable_pub = rospy.Publisher(
            self.yolo_enable_topic,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.motion_result_count = 0
        self.last_motion_result = None
        self.motion_start_count = 0
        self.command_sent = False
        self.refresh_start_time = None
        self.reference_target = None
        self.preobserve_pose = None
        self.enabled = False
        self.refresh_timeout = 1.5

    def _motion_result_callback(self, msg):
        self.last_motion_result = msg
        self.motion_result_count += 1

    def initialise(self):
        self.command_sent = False
        self.last_motion_result = None
        self.motion_start_count = self.motion_result_count
        self.refresh_start_time = None
        self.reference_target = blackboard_get(SCENE_PREOBSERVE_TARGET_BB)
        scene_targets = list(blackboard_get(SCENE_RANK_TARGETS_BB, []) or [])
        if self.reference_target is None:
            self.reference_target = pick_best_candidate_by_score(scene_targets)
        self.enabled = bool(
            rospy.get_param("~scene_preobserve_enabled", False)
            and is_scene_wide_non_cube_ranking_enabled()
            and scene_targets
            and self.reference_target is not None
        )
        self.refresh_timeout = float(rospy.get_param("~scene_preobserve_refresh_timeout", 1.5))
        self.preobserve_pose = None
        if not self.enabled:
            return

        log_stage("MoveToNonCubePreObserve")
        target_label = str(self.reference_target.get("label", "unknown"))
        self.preobserve_pose, raw_z, clamped_z = self._build_preobserve_pose(
            self.reference_target["target_point_base"],
            target_label,
        )
        if abs(clamped_z - raw_z) > 1e-6:
            rospy.logwarn(
                "[Brain] Clamped non-cube pre-observation z from %.3f to %.3f for %s",
                raw_z,
                clamped_z,
                target_label,
            )
        rospy.loginfo(
            "[Brain] Stage action: move to non-cube pre-observation pose | reference=%s semantic=%s pos=(%.3f, %.3f, %.3f) reverse_local_x_offset=%.3f base_offset_yz=(%.3f, %.3f)",
            target_label,
            self.reference_target.get("semantic_label", "unknown"),
            self.preobserve_pose.pose.position.x,
            self.preobserve_pose.pose.position.y,
            self.preobserve_pose.pose.position.z,
            float(rospy.get_param("~scene_preobserve_base_x_offset", 0.40)),
            float(rospy.get_param("~scene_preobserve_base_y_offset", 0.0)),
            float(rospy.get_param("~scene_preobserve_base_z_offset", 0.0)),
        )

    def _lookup_transform(self, frame_id, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                stamp,
                rospy.Duration(0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                rospy.Time(0),
                rospy.Duration(0.5),
            )

    def _build_preobserve_pose(self, target_point, target_label):
        grasp_min_z = float(rospy.get_param("~direct_grasp_min_z", 0.05))
        grasp_max_z = float(rospy.get_param("~direct_grasp_max_z", 1.20))
        grasp_roll = float(rospy.get_param("~direct_grasp_roll", 3.141592653589793))
        grasp_pitch = float(rospy.get_param("~direct_grasp_pitch", 1.5707963267948966))
        grasp_yaw = float(rospy.get_param("~direct_grasp_yaw", 0.0))
        reverse_local_x_offset = float(rospy.get_param("~scene_preobserve_base_x_offset", 0.40))
        base_offset_y = float(rospy.get_param("~scene_preobserve_base_y_offset", 0.0))
        base_offset_z = float(rospy.get_param("~scene_preobserve_base_z_offset", 0.0))

        qx, qy, qz, qw = quaternion_from_euler(grasp_roll, grasp_pitch, grasp_yaw)
        orientation = Quaternion(qx, qy, qz, qw)
        local_dx, local_dy, local_dz = rotate_vector_by_quaternion(
            orientation,
            (-reverse_local_x_offset, 0.0, 0.0),
        )

        pose = PoseStamped()
        pose.header.frame_id = target_point.header.frame_id or "base_link"
        pose.header.stamp = rospy.Time.now()
        pose.pose.orientation = orientation
        raw_z = float(target_point.point.z) + base_offset_z + local_dz
        clamped_z = max(grasp_min_z, min(grasp_max_z, raw_z))
        pose.pose.position.x = float(target_point.point.x) + local_dx
        pose.pose.position.y = float(target_point.point.y) + base_offset_y + local_dy
        pose.pose.position.z = clamped_z
        return pose, raw_z, clamped_z

    def _send_preobserve_motion(self):
        if self.motion_cmd_pub.get_num_connections() == 0 or self.preobserve_pose is None:
            return False

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_POSE
        cmd.target_pose = self.preobserve_pose
        cmd.max_velocity = float(rospy.get_param("~scene_preobserve_max_velocity", 0.6))
        cmd.max_acceleration = float(rospy.get_param("~scene_preobserve_max_acceleration", 0.6))
        cmd.collision_check = False
        self.last_motion_result = None
        self.motion_start_count = self.motion_result_count
        self.motion_cmd_pub.publish(cmd)
        self.command_sent = True
        return True

    def _refresh_scene_targets_from_current_view(self):
        detections = list(blackboard_get("detected_objects", []) or [])
        if not detections:
            return []

        frame_id = blackboard_get(LATEST_YOLO_FRAME_ID_BB, "") or rospy.get_param(
            "~perception_frame_id", "camera_rgb_optical_frame"
        )
        stamp = blackboard_get(LATEST_YOLO_STAMP_BB, rospy.Time(0))
        trans = self._lookup_transform(frame_id, stamp)
        failed_target_keys = get_failed_target_keys()
        refreshed = []

        for obj in detections:
            if not isinstance(obj, dict):
                continue
            center = obj.get("center_3d")
            if not isinstance(center, dict):
                continue

            pt_camera = PointStamped()
            pt_camera.header.frame_id = frame_id
            pt_camera.header.stamp = trans.header.stamp
            pt_camera.point.x = float(center.get("x", 0.0))
            pt_camera.point.y = float(center.get("y", 0.0))
            pt_camera.point.z = float(center.get("z", 0.0))
            pt_base = tf2_geometry_msgs.do_transform_point(pt_camera, trans)

            label = str(obj.get("name", "unknown"))
            semantic_label = semantic_object_label(label)
            canonical_label = canonical_object_label(label)
            pick_category = pick_category_for_label(label)
            if pick_category != "non_cube":
                continue

            target_key = build_target_key(label, pt_base)
            if target_key in failed_target_keys:
                continue

            refreshed.append(
                {
                    "target_object": obj,
                    "target_point_base": pt_base,
                    "target_key": target_key,
                    "label": label,
                    "semantic_label": semantic_label,
                    "canonical_label": canonical_label,
                    "pick_category": pick_category,
                    "score": score_object_label(canonical_label),
                    "confidence": float(obj.get("confidence", 0.0)),
                }
            )

        return refreshed

    def update(self):
        if not self.enabled:
            return Status.SUCCESS

        if not self.command_sent:
            if not self._send_preobserve_motion():
                return Status.RUNNING
            return Status.RUNNING

        if self.refresh_start_time is None:
            if self.motion_result_count <= self.motion_start_count:
                return Status.RUNNING
            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                self.refresh_start_time = rospy.Time.now()
                self.yolo_enable_pub.publish(Bool(data=True))
                rospy.loginfo("[Brain] Stage action: refresh YOLO detections from non-cube pre-observation pose")
                return Status.RUNNING

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "non-cube pre-observation motion failed"
            )
            self.yolo_enable_pub.publish(Bool(data=False))
            rospy.logwarn(
                "[Brain] Non-cube pre-observation move failed, continuing from current overview state: %s",
                failure_message,
            )
            return Status.SUCCESS

        now = rospy.Time.now()
        latest_recv_time = blackboard_get(LATEST_YOLO_RECV_TIME_BB, rospy.Time(0))
        if isinstance(latest_recv_time, rospy.Time) and latest_recv_time > self.refresh_start_time:
            try:
                refreshed_targets = self._refresh_scene_targets_from_current_view()
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(2.0, "[Brain] Waiting for TF during non-cube pre-observation refresh: %s", exc)
                return Status.RUNNING

            if refreshed_targets:
                blackboard = get_blackboard()
                clear_target_selection()
                blackboard.set(SCENE_RANK_TARGETS_BB, list(refreshed_targets))
                blackboard.set(SCENE_RANKED_RESULTS_BB, [])
                blackboard.set(SCENE_RANK_FAILED_TARGETS_BB, [])
                blackboard.set(SCENE_PREOBSERVE_TARGET_BB, pick_best_candidate_by_score(refreshed_targets))
                self.yolo_enable_pub.publish(Bool(data=False))
                rospy.loginfo(
                    "[Brain] Stage complete: MoveToNonCubePreObserve | refreshed_non_cube_targets=%d reference=%s preobserve=(%.3f, %.3f, %.3f)",
                    len(refreshed_targets),
                    str(self.reference_target.get("label", "unknown")),
                    self.preobserve_pose.pose.position.x,
                    self.preobserve_pose.pose.position.y,
                    self.preobserve_pose.pose.position.z,
                )
                return Status.SUCCESS

        if (now - self.refresh_start_time).to_sec() >= self.refresh_timeout:
            self.yolo_enable_pub.publish(Bool(data=False))
            rospy.logwarn(
                "[Brain] Non-cube pre-observation refresh timed out after %.2fs; proceeding with previous scene targets",
                (now - self.refresh_start_time).to_sec(),
            )
            return Status.SUCCESS

        return Status.RUNNING


class RequestGraspPoseBehavior(py_trees.behaviour.Behaviour):
    """Build a grasp pose from direct YOLO geometry, one-shot GraspNet, or scene-wide GraspNet ranking."""

    def __init__(self, name="RequestGraspPose"):
        super().__init__(name)
        self.request_sent = False
        self.grasp_pose_source = "yolo"
        self.graspnet_request_start = None
        self.graspnet_request_stamp = None
        self.graspnet_candidate_msg = None
        self.graspnet_failure_message = ""
        self.graspnet_candidate_topic = rospy.get_param(
            "~graspnet_candidate_topic",
            "/perception/grasp_candidates",
        )
        self.graspnet_failure_topic = rospy.get_param(
            "~graspnet_failure_topic",
            "/perception/grasp_failure_reason",
        )
        self.graspnet_task_decision_topic = rospy.get_param(
            "~graspnet_task_decision_topic",
            "/brain/task_decision",
        )
        self.target_grasp_pose_topic = rospy.get_param(
            "~target_grasp_pose_topic",
            "/brain/target_grasp_pose",
        )
        self.target_grasp_tf_frame = rospy.get_param(
            "~target_grasp_tf_frame",
            "brain_target_grasp_pose",
        )
        self.publish_target_grasp_tf = bool(rospy.get_param("~publish_target_grasp_tf", True))
        self.non_cube_ranked_grasp_pose_topic = rospy.get_param(
            "~non_cube_ranked_grasp_pose_topic",
            "/brain/non_cube_ranked_grasp_poses",
        )
        self.non_cube_ranked_grasp_marker_topic = rospy.get_param(
            "~non_cube_ranked_grasp_marker_topic",
            "/brain/non_cube_ranked_grasp_markers",
        )
        self.graspnet_request_pub = rospy.Publisher(
            self.graspnet_task_decision_topic,
            TaskDecision,
            queue_size=1,
        )
        self.target_grasp_pose_pub = rospy.Publisher(
            self.target_grasp_pose_topic,
            PoseStamped,
            queue_size=1,
            latch=True,
        )
        self.target_pose_preview_pub = rospy.Publisher(
            "/path_planning/target_pose_preview",
            PoseStamped,
            queue_size=1,
            latch=True,
        )
        self.non_cube_ranked_grasp_pose_pub = rospy.Publisher(
            self.non_cube_ranked_grasp_pose_topic,
            PoseArray,
            queue_size=1,
            latch=True,
        )
        self.non_cube_ranked_grasp_marker_pub = rospy.Publisher(
            self.non_cube_ranked_grasp_marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.graspnet_candidate_sub = rospy.Subscriber(
            self.graspnet_candidate_topic,
            GraspCandidate,
            self._graspnet_candidate_callback,
            queue_size=10,
        )
        self.graspnet_failure_sub = rospy.Subscriber(
            self.graspnet_failure_topic,
            String,
            self._graspnet_failure_callback,
            queue_size=10,
        )
        self.target_pose_tf_broadcaster = (
            tf2_ros.TransformBroadcaster() if self.publish_target_grasp_tf else None
        )
        # This behaviour computes non-cube grasp yaw in base_link from camera-frame bbox axes.
        # Keep a dedicated tf buffer/listener so lookup_transform is always available here.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.scene_targets = []
        self.scene_target_index = 0
        self.scene_current_target = None
        self.scene_current_candidates = []
        self.scene_current_failure_message = ""
        self.scene_current_last_candidate_time = None
        self.scene_ranked_results = []
        self.scene_failed_targets = []
        self.scene_request_gap_until = None
        self.scene_wide_non_cube_ranking = False
        self.debug_non_cube_only = False
        self.execute_after_scene_ranking = False
        self.scene_grasp_per_target_timeout = 1.0
        self.scene_grasp_candidate_settle_time = 0.15
        self.simple_grasp_cloud_topic = rospy.get_param("~simple_grasp_cloud_topic", "/perception/yolo_bbox_instance_cloud")
        self.simple_grasp_label_field_name = rospy.get_param("~simple_grasp_label_field_name", "label")
        self.latest_simple_cloud_points = np.empty((0, 3), dtype=np.float32)
        self.latest_simple_cloud_labels = None
        self.latest_simple_cloud_frame_id = "base_link"
        self.latest_simple_cloud_stamp = rospy.Time(0)
        self.simple_cloud_sub = rospy.Subscriber(
            self.simple_grasp_cloud_topic,
            PointCloud2,
            self._simple_cloud_callback,
            queue_size=1,
        )

    @staticmethod
    def _extract_numeric_suffix(value):
        match = re.search(r"(\d+)$", str(value or ""))
        if match is None:
            return None
        return int(match.group(1))

    def _reset_scene_request_state(self):
        self.scene_current_target = None
        self.scene_current_candidates = []
        self.scene_current_failure_message = ""
        self.scene_current_last_candidate_time = None
        self.graspnet_failure_message = ""
        self.graspnet_candidate_msg = None
        self.graspnet_request_start = None
        self.graspnet_request_stamp = None
        self.request_sent = False

    def _candidate_matches_request(self, msg):
        tolerance = float(rospy.get_param("~graspnet_response_stamp_tolerance", 0.3))
        stamp = msg.pose.header.stamp
        if (
            self.graspnet_request_stamp is not None
            and stamp is not None
            and (stamp.secs != 0 or stamp.nsecs != 0)
        ):
            stamp_delta = abs((stamp - self.graspnet_request_stamp).to_sec())
            if stamp_delta > tolerance:
                return False
        return True

    def _candidate_from_msg(self, msg):
        return {
            "pose": copy.deepcopy(msg.pose),
            "quality": float(msg.quality),
            "width": float(msg.width),
        }

    def _graspnet_candidate_callback(self, msg):
        if not self.request_sent:
            return
        if not self._candidate_matches_request(msg):
            return

        if self.grasp_pose_source == "scene_graspnet":
            if self.scene_current_target is None:
                return
            self.scene_current_candidates.append(self._candidate_from_msg(msg))
            self.scene_current_last_candidate_time = rospy.Time.now()
            return

        if self.grasp_pose_source != "graspnet":
            return

        if (
            self.graspnet_candidate_msg is None
            or float(msg.quality) > float(self.graspnet_candidate_msg.quality)
        ):
            self.graspnet_candidate_msg = msg

    def _graspnet_failure_callback(self, msg):
        if not self.request_sent:
            return

        failure_message = str(msg.data or "").strip()
        if self.grasp_pose_source == "scene_graspnet":
            if self.scene_current_target is None or self.scene_current_candidates:
                return
            self.scene_current_failure_message = failure_message
            return

        if self.grasp_pose_source != "graspnet":
            return
        self.graspnet_failure_message = failure_message

    def _simple_cloud_callback(self, msg):
        try:
            points, labels = pointcloud2_to_xyzl(msg, self.simple_grasp_label_field_name)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[Brain] Failed to decode simple grasp cloud: %s", exc)
            return
        self.latest_simple_cloud_points = points
        self.latest_simple_cloud_labels = labels
        self.latest_simple_cloud_frame_id = msg.header.frame_id or "base_link"
        self.latest_simple_cloud_stamp = msg.header.stamp

    def initialise(self):
        self.request_sent = False
        self.graspnet_request_start = None
        self.graspnet_request_stamp = None
        self.graspnet_candidate_msg = None
        self.graspnet_failure_message = ""
        self.scene_targets = list(blackboard_get(SCENE_RANK_TARGETS_BB, []) or [])
        self.scene_target_index = 0
        self.scene_current_target = None
        self.scene_current_candidates = []
        self.scene_current_failure_message = ""
        self.scene_current_last_candidate_time = None
        self.scene_ranked_results = []
        self.scene_failed_targets = []
        self.scene_request_gap_until = None
        self.scene_wide_non_cube_ranking = is_scene_wide_non_cube_ranking_enabled()
        self.debug_non_cube_only = is_debug_non_cube_only_enabled()
        self.execute_after_scene_ranking = should_execute_after_scene_ranking()
        self.scene_grasp_per_target_timeout = float(rospy.get_param("~scene_grasp_per_target_timeout", 1.0))
        self.scene_grasp_candidate_settle_time = float(rospy.get_param("~scene_grasp_candidate_settle_time", 0.15))

        requested_source = normalize_label(rospy.get_param("~grasp_pose_source", "hybrid"))
        target_object = blackboard_get("target_object", {}) or {}
        target_label = (
            target_object.get("name", "unknown")
            if isinstance(target_object, dict)
            else str(target_object)
        )
        semantic_label = semantic_object_label(target_label)
        canonical_label = canonical_object_label(target_label)

        if self.scene_wide_non_cube_ranking and self.scene_targets:
            self.grasp_pose_source = "scene_simple"
            log_stage("RankSceneWideNonCubeSimpleGrasps")
            rospy.loginfo(
                "[Brain] Stage action: scene-wide non-cube simple point-cloud ranking | targets=%d debug_non_cube_only=%s execute_after_scene_ranking=%s",
                len(self.scene_targets),
                self.debug_non_cube_only,
                self.execute_after_scene_ranking,
            )
            return

        if requested_source in ("graspnet", "grasp_net"):
            self.grasp_pose_source = "graspnet"
        elif requested_source in ("hybrid", "mixed", "auto"):
            self.grasp_pose_source = "yolo" if canonical_label == "cube" else "graspnet"
        else:
            self.grasp_pose_source = "yolo"

        if self.grasp_pose_source == "graspnet":
            log_stage("RequestGraspPoseFromGraspNet")
        else:
            log_stage("BuildDirectGraspPose")

        rospy.loginfo(
            "[Brain] Stage action: grasp pose source=%s for target=%s semantic=%s category=%s configured_mode=%s",
            self.grasp_pose_source,
            target_label,
            semantic_label,
            pick_category_for_label(target_label),
            requested_source or "yolo",
        )

    def _resolve_graspnet_target_id(self, target_object):
        if not isinstance(target_object, dict):
            return None, "missing_target_object"

        instance_id = target_object.get("instance_id")
        if instance_id is not None:
            try:
                resolved = int(instance_id)
            except (TypeError, ValueError):
                resolved = None
            if resolved is not None and resolved >= 0:
                return resolved, "instance_id"

        class_id = target_object.get("class_id")
        if class_id is not None:
            try:
                resolved = int(class_id)
            except (TypeError, ValueError):
                resolved = None
            if resolved is not None and resolved >= 0:
                return resolved, "class_id"

        label = target_object.get("name", "")
        numeric_suffix = self._extract_numeric_suffix(label)
        if numeric_suffix is not None:
            return numeric_suffix, "name_suffix"

        return None, "unresolved"

    @staticmethod
    def _stamp_from_target_object(target_object):
        if not isinstance(target_object, dict):
            return None
        stamp_dict = target_object.get("gazebo_stamp", {})
        secs = int(stamp_dict.get("secs", 0))
        nsecs = int(stamp_dict.get("nsecs", 0))
        if secs == 0 and nsecs == 0:
            return None
        return rospy.Time(secs=secs, nsecs=nsecs)

    def _publish_resolved_target_pose(self, pose):
        resolved_pose = PoseStamped()
        resolved_pose.header.frame_id = pose.header.frame_id or "base_link"
        resolved_pose.header.stamp = pose.header.stamp
        if resolved_pose.header.stamp == rospy.Time(0):
            resolved_pose.header.stamp = rospy.Time.now()
        resolved_pose.pose = pose.pose

        self.target_grasp_pose_pub.publish(resolved_pose)
        self.target_pose_preview_pub.publish(resolved_pose)

        if self.publish_target_grasp_tf and self.target_pose_tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = resolved_pose.header.stamp
            transform.header.frame_id = resolved_pose.header.frame_id
            transform.child_frame_id = self.target_grasp_tf_frame
            transform.transform.translation.x = resolved_pose.pose.position.x
            transform.transform.translation.y = resolved_pose.pose.position.y
            transform.transform.translation.z = resolved_pose.pose.position.z
            transform.transform.rotation = resolved_pose.pose.orientation
            self.target_pose_tf_broadcaster.sendTransform(transform)

        return resolved_pose

    def _compute_non_cube_bbox_grasp_hints(self, target_object):
        if not isinstance(target_object, dict):
            return {}

        bbox = target_object.get("bbox")
        width_px = 0.0
        height_px = 0.0
        if isinstance(bbox, dict):
            width_px = float(bbox.get("width", bbox.get("w", 0.0)) or 0.0)
            height_px = float(bbox.get("height", bbox.get("h", 0.0)) or 0.0)
        elif isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            width_px = max(0.0, float(bbox[2]) - float(bbox[0]))
            height_px = max(0.0, float(bbox[3]) - float(bbox[1]))

        center = target_object.get("center_3d") if isinstance(target_object.get("center_3d"), dict) else {}
        depth_z = float(center.get("z", 0.0) or 0.0)
        fx = float(target_object.get("camera_fx", 0.0) or 0.0)
        fy = float(target_object.get("camera_fy", 0.0) or 0.0)

        width_x_m = None
        width_y_m = None

        width_min = float(rospy.get_param("~non_cube_bbox_width_min_m", 0.01))
        width_max = float(rospy.get_param("~non_cube_bbox_width_max_m", 0.10))
        cube_nominal_width_m = float(rospy.get_param("~cube_nominal_grasp_width_m", 0.0278))
        non_cube_fixed_width_multiplier = float(rospy.get_param("~non_cube_fixed_width_multiplier", 2.5))
        target_width_m = max(
            width_min,
            min(width_max, cube_nominal_width_m * non_cube_fixed_width_multiplier),
        )

        long_axis_cam = (1.0, 0.0, 0.0)
        if height_px > width_px:
            long_axis_cam = (0.0, 1.0, 0.0)

        if width_px > 0.0 and height_px > 0.0 and depth_z > 1e-6 and fx > 1e-6 and fy > 1e-6:
            width_x_m = (width_px * depth_z) / fx
            width_y_m = (height_px * depth_z) / fy
            if width_y_m > width_x_m:
                long_axis_cam = (0.0, 1.0, 0.0)

        long_axis_yaw = None
        frame_id = str(target_object.get("frame_id", "") or "").strip()
        if frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.2),
                )
                axis_base = rotate_vector_by_quaternion(transform.transform.rotation, long_axis_cam)
                if math.hypot(float(axis_base[0]), float(axis_base[1])) > 1e-6:
                    long_axis_yaw = math.atan2(float(axis_base[1]), float(axis_base[0]))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                long_axis_yaw = None

        return {
            "bbox_width_px": float(width_px),
            "bbox_height_px": float(height_px),
            "bbox_width_x_m": None if width_x_m is None else float(width_x_m),
            "bbox_height_y_m": None if width_y_m is None else float(width_y_m),
            "target_width_m": None if target_width_m is None else float(target_width_m),
            "long_axis_yaw": None if long_axis_yaw is None else float(long_axis_yaw),
        }

    def _build_direct_yolo_grasp_pose(self, blackboard, target_point, target_label, target_object):
        category = pick_category_for_label(target_label)
        bbox_hints = {}
        override_yaw = None
        target_grasp_width_m = None

        if category == "non_cube":
            bbox_hints = self._compute_non_cube_bbox_grasp_hints(target_object)
            override_yaw = bbox_hints.get("long_axis_yaw")
            target_grasp_width_m = bbox_hints.get("target_width_m")

        grasp_pose, metadata, raw_z, clamped_z = build_direct_yolo_grasp_pose(
            target_point,
            target_label,
            stamp=rospy.Time.now(),
            override_yaw=override_yaw,
        )

        if blackboard_get(CUBE_ON_SCALE_BB, False) and category == "cube":
            retreat = float(rospy.get_param("~scale_retrieval_local_x_retreat", 0.061))
            dx, dy, dz = rotate_vector_by_quaternion(
                grasp_pose.pose.orientation, (-retreat, 0.0, 0.0),
            )
            grasp_pose.pose.position.x += dx
            grasp_pose.pose.position.y += dy
            grasp_pose.pose.position.z += dz
            rospy.loginfo(
                "[Brain] Scale retrieval: retreated %.3f m along -local_x (delta=%.4f, %.4f, %.4f) -> pos=(%.3f, %.3f, %.3f)",
                retreat, dx, dy, dz,
                grasp_pose.pose.position.x,
                grasp_pose.pose.position.y,
                grasp_pose.pose.position.z,
            )

        resolved_pose = self._publish_resolved_target_pose(grasp_pose)

        blackboard.set("target_grasp_pose", resolved_pose)
        blackboard.set("target_grasp_mode", "direct_yolo_point")
        blackboard.set("target_grasp_width_m", target_grasp_width_m)

        if abs(clamped_z - raw_z) > 1e-6:
            rospy.logwarn(
                "[Brain] Clamped target z from %.3f to %.3f for %s",
                raw_z,
                clamped_z,
                target_label,
            )

        grasp_yaw = float(override_yaw) if override_yaw is not None else float(rospy.get_param("~direct_grasp_yaw", 0.0))
        rospy.loginfo(
            "[Brain] Stage complete: BuildDirectGraspPose | target=%s category=%s pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) offset_mode=%s offset=(%.3f, %.3f, %.3f) local_x_offset=%.3f target_width_m=%s bbox_px=(%.1f, %.1f)",
            target_label,
            category,
            resolved_pose.pose.position.x,
            resolved_pose.pose.position.y,
            resolved_pose.pose.position.z,
            float(rospy.get_param("~direct_grasp_roll", 3.141592653589793)),
            float(rospy.get_param("~direct_grasp_pitch", 1.5707963267948966)),
            grasp_yaw,
            metadata["offset_mode"],
            metadata["offset_x"],
            metadata["offset_y"],
            metadata["offset_z"],
            metadata["local_x_offset"],
            "None" if target_grasp_width_m is None else f"{target_grasp_width_m:.4f}",
            float(bbox_hints.get("bbox_width_px", 0.0) or 0.0),
            float(bbox_hints.get("bbox_height_px", 0.0) or 0.0),
        )
        self.request_sent = True
        return Status.SUCCESS

    def _send_graspnet_request(self, target_object, include_source_context=True):
        target_id, id_source = self._resolve_graspnet_target_id(target_object)
        if target_id is None:
            return False, f"Cannot resolve GraspNet target_object_id ({id_source})"

        request = TaskDecision()
        request.task_type = TaskDecision.GRASP
        request.target_object_id = int(target_id)

        source_stamp = self._stamp_from_target_object(target_object) if include_source_context else None
        source_frame = (
            str(target_object.get("frame_id", "")).strip()
            if include_source_context and isinstance(target_object, dict)
            else ""
        )
        self.graspnet_request_stamp = None
        if source_stamp is not None:
            request.target_pose.header.stamp = source_stamp
            self.graspnet_request_stamp = source_stamp
        if source_frame:
            request.target_pose.header.frame_id = source_frame

        self.graspnet_candidate_msg = None
        self.graspnet_failure_message = ""
        self.graspnet_request_start = rospy.Time.now()
        self.graspnet_request_pub.publish(request)
        self.request_sent = True
        rospy.loginfo(
            "[Brain] Stage action: request grasp pose from GraspNet | target_id=%d id_source=%s source_frame=%s source_stamp=%s",
            int(target_id),
            id_source,
            source_frame or "(latest)",
            "None" if source_stamp is None else f"{int(source_stamp.secs)}.{int(source_stamp.nsecs):09d}",
        )
        return True, id_source

    def _update_graspnet_pose(self, blackboard, target_object):
        target_label = (
            target_object.get("name", "unknown")
            if isinstance(target_object, dict)
            else str(target_object)
        )

        if not self.request_sent:
            success, message = self._send_graspnet_request(target_object)
            if not success:
                prepare_retry_without_overview(
                    f"graspnet request setup failed for {target_label}: {message}",
                    blacklist=True,
                )
                return Status.FAILURE
            return Status.RUNNING

        if self.graspnet_candidate_msg is not None:
            resolved_pose = self._publish_resolved_target_pose(self.graspnet_candidate_msg.pose)
            blackboard.set("target_grasp_pose", resolved_pose)
            blackboard.set("target_grasp_mode", "graspnet")
            rospy.loginfo(
                "[Brain] Stage complete: RequestGraspPoseFromGraspNet | target=%s quality=%.3f width=%.3f frame=%s",
                target_label,
                float(self.graspnet_candidate_msg.quality),
                float(self.graspnet_candidate_msg.width),
                resolved_pose.header.frame_id or "unknown",
            )
            return Status.SUCCESS

        if self.graspnet_failure_message:
            prepare_retry_without_overview(
                f"graspnet failed for {target_label}: {self.graspnet_failure_message}",
                blacklist=True,
            )
            return Status.FAILURE

        timeout_sec = float(rospy.get_param("~graspnet_request_timeout", 5.0))
        if self.graspnet_request_start is not None:
            elapsed = (rospy.Time.now() - self.graspnet_request_start).to_sec()
            if elapsed >= timeout_sec:
                prepare_retry_without_overview(
                    f"graspnet timed out for {target_label} after {elapsed:.1f}s",
                    blacklist=True,
                )
                return Status.FAILURE

        return Status.RUNNING

    @staticmethod
    def _rank_candidate_entry(entry):
        return grasp_geometry_rank_key(entry)

    def _rank_scene_candidates(self, candidates):
        ranked = []
        min_pose_z = float(rospy.get_param("~scene_rank_min_pose_z", 0.10))
        min_top_score = float(rospy.get_param("~scene_rank_min_top_score", 0.0))
        for candidate in candidates:
            scored = dict(candidate)
            scored.update(compute_grasp_geometry_scores(candidate["pose"], candidate["quality"]))
            if float(scored["pose_z"]) < min_pose_z:
                continue
            if float(scored["top_score"]) <= min_top_score:
                continue
            ranked.append(scored)
        ranked.sort(key=self._rank_candidate_entry)
        return ranked

    def _make_scene_rank_gripper_marker(self, marker_id, frame_id, stamp, pose_stamped, width, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "brain_scene_rank_gripper"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.004
        marker.color = color

        half_gap = max(float(width), 0.005) * 0.5
        local_segments = [
            ((-0.02, half_gap, 0.0), (0.04, half_gap, 0.0)),
            ((-0.02, -half_gap, 0.0), (0.04, -half_gap, 0.0)),
            ((-0.02, half_gap, 0.0), (-0.02, -half_gap, 0.0)),
        ]
        for start_local, end_local in local_segments:
            start_offset = rotate_vector_by_quaternion(pose_stamped.pose.orientation, start_local)
            end_offset = rotate_vector_by_quaternion(pose_stamped.pose.orientation, end_local)
            marker.points.append(
                Point(
                    x=float(pose_stamped.pose.position.x + start_offset[0]),
                    y=float(pose_stamped.pose.position.y + start_offset[1]),
                    z=float(pose_stamped.pose.position.z + start_offset[2]),
                )
            )
            marker.points.append(
                Point(
                    x=float(pose_stamped.pose.position.x + end_offset[0]),
                    y=float(pose_stamped.pose.position.y + end_offset[1]),
                    z=float(pose_stamped.pose.position.z + end_offset[2]),
                )
            )
        return marker

    def _make_scene_rank_text_marker(self, marker_id, frame_id, stamp, pose_stamped, scene_entry, rank_index, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "brain_scene_rank_text"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(
            x=float(pose_stamped.pose.position.x),
            y=float(pose_stamped.pose.position.y),
            z=float(pose_stamped.pose.position.z + 0.06),
        )
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.03
        marker.color = color
        marker.text = (
            f"#{rank_index + 1} {scene_entry['semantic_label']} "
            f"z={scene_entry['best_candidate']['pose_z']:.2f} "
            f"top={scene_entry['best_candidate']['top_score']:.2f} "
            f"side={scene_entry['best_candidate']['robot_side_score']:.2f} "
            f"q={scene_entry['best_candidate']['quality']:.2f}"
        )
        return marker

    def _publish_scene_ranked_outputs(self, ranked_results):
        stamp = rospy.Time.now()
        frame_id = "base_link"

        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = stamp

        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = frame_id
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        total = max(len(ranked_results), 1)
        for index, scene_entry in enumerate(ranked_results):
            best_candidate = scene_entry["best_candidate"]
            pose_array.poses.append(copy.deepcopy(best_candidate["pose"].pose))
            color_ratio = 1.0 - (float(index) / float(total))
            color = ColorRGBA(
                r=float(1.0 - color_ratio),
                g=float(color_ratio),
                b=0.2,
                a=0.95,
            )
            marker_array.markers.append(
                self._make_scene_rank_gripper_marker(
                    marker_id=index * 2,
                    frame_id=frame_id,
                    stamp=stamp,
                    pose_stamped=best_candidate["pose"],
                    width=best_candidate["width"],
                    color=color,
                )
            )
            marker_array.markers.append(
                self._make_scene_rank_text_marker(
                    marker_id=index * 2 + 1,
                    frame_id=frame_id,
                    stamp=stamp,
                    pose_stamped=best_candidate["pose"],
                    scene_entry=scene_entry,
                    rank_index=index,
                    color=color,
                )
            )

        self.non_cube_ranked_grasp_pose_pub.publish(pose_array)
        self.non_cube_ranked_grasp_marker_pub.publish(marker_array)

    def _advance_scene_target(self):
        self.scene_target_index += 1
        self.scene_request_gap_until = rospy.Time.now() + rospy.Duration(
            max(self.scene_grasp_candidate_settle_time, 0.05)
        )
        self._reset_scene_request_state()

    def _record_scene_target_failure(self, reason):
        current = self.scene_current_target or {}
        self.scene_failed_targets.append(
            {
                "target_key": current.get("target_key"),
                "label": current.get("label", "unknown"),
                "semantic_label": current.get("semantic_label", "unknown"),
                "reason": reason,
            }
        )
        rospy.logwarn(
            "[Brain] Scene ranking target failed | target=%s semantic=%s reason=%s",
            current.get("label", "unknown"),
            current.get("semantic_label", "unknown"),
            reason,
        )
        self._advance_scene_target()

    def _record_scene_target_success(self):
        current = self.scene_current_target or {}
        ranked_candidates = self._rank_scene_candidates(self.scene_current_candidates)
        if not ranked_candidates:
            min_pose_z = float(rospy.get_param("~scene_rank_min_pose_z", 0.10))
            min_top_score = float(rospy.get_param("~scene_rank_min_top_score", 0.0))
            self._record_scene_target_failure(
                "No candidates remained after scene-ranking pose filtering"
                f" (min_pose_z={min_pose_z:.3f}, min_top_score>{min_top_score:.3f})"
            )
            return
        best_candidate = ranked_candidates[0]
        self.scene_ranked_results.append(
            {
                "target_object": current.get("target_object", {}),
                "target_point_base": current.get("target_point_base"),
                "target_key": current.get("target_key"),
                "label": current.get("label", "unknown"),
                "semantic_label": current.get("semantic_label", "unknown"),
                "pick_category": current.get("pick_category", "non_cube"),
                "score": current.get("score", 0.0),
                "confidence": current.get("confidence", 0.0),
                "best_candidate": best_candidate,
                "candidate_count": len(ranked_candidates),
            }
        )
        rospy.loginfo(
            "[Brain] Scene ranking target complete | target=%s semantic=%s candidates=%d top=%.3f side=%.3f quality=%.3f z=%.3f",
            current.get("label", "unknown"),
            current.get("semantic_label", "unknown"),
            len(ranked_candidates),
            best_candidate["top_score"],
            best_candidate["robot_side_score"],
            best_candidate["quality"],
            best_candidate["pose_z"],
        )
        self._advance_scene_target()

    def _persist_scene_failures(self):
        if self.debug_non_cube_only:
            return
        failed_target_keys = get_failed_target_keys()
        for failure in self.scene_failed_targets:
            target_key = failure.get("target_key")
            if target_key:
                failed_target_keys.add(str(target_key))
        set_failed_target_keys(failed_target_keys)

    def _finish_scene_ranking(self, blackboard):
        ranked_results = sorted(
            self.scene_ranked_results,
            key=lambda entry: self._rank_candidate_entry(entry["best_candidate"]),
        )
        blackboard.set(SCENE_RANKED_RESULTS_BB, ranked_results)
        blackboard.set(SCENE_RANK_FAILED_TARGETS_BB, list(self.scene_failed_targets))
        self._publish_scene_ranked_outputs(ranked_results)
        self._persist_scene_failures()

        if not ranked_results:
            clear_target_selection()
            blackboard.set("target_grasp_mode", "scene_graspnet_empty")
            rospy.logwarn("[Brain] Scene-wide non-cube simple ranking found no valid grasps in this scan")
            if self.execute_after_scene_ranking:
                prepare_retry_without_overview(
                    "scene-wide simple point-cloud ranking found no candidates for current non-cube targets",
                    blacklist=False,
                )
                return Status.FAILURE
            return Status.SUCCESS

        best_entry = ranked_results[0]
        resolved_pose = self._publish_resolved_target_pose(best_entry["best_candidate"]["pose"])
        blackboard.set("target_object", best_entry["target_object"])
        blackboard.set("target_point_base_link", best_entry["target_point_base"])
        blackboard.set("target_key", best_entry["target_key"])
        blackboard.set("target_score", best_entry["score"])
        blackboard.set("target_confidence", best_entry["confidence"])
        blackboard.set("target_grasp_pose", resolved_pose)
        blackboard.set("target_grasp_mode", "scene_graspnet")
        rospy.loginfo(
            "[Brain] Stage complete: RankSceneWideNonCubeSimpleGrasps | ranked=%d best=%s semantic=%s top=%.3f side=%.3f quality=%.3f z=%.3f execute_after_scene_ranking=%s",
            len(ranked_results),
            best_entry["label"],
            best_entry["semantic_label"],
            best_entry["best_candidate"]["top_score"],
            best_entry["best_candidate"]["robot_side_score"],
            best_entry["best_candidate"]["quality"],
            best_entry["best_candidate"]["pose_z"],
            self.execute_after_scene_ranking,
        )
        return Status.SUCCESS

    def _update_scene_wide_non_cube_ranking(self, blackboard):
        if not self.scene_targets:
            if self.execute_after_scene_ranking:
                prepare_retry_without_overview(
                    "scene-wide non-cube simple ranking started without any targets",
                    blacklist=False,
                )
                return Status.FAILURE
            rospy.logwarn("[Brain] Scene-wide non-cube simple ranking has no targets for this scan")
            return Status.SUCCESS

        self.scene_ranked_results = []
        self.scene_failed_targets = []
        min_pose_z = float(rospy.get_param("~scene_rank_min_pose_z", 0.10))
        min_top_score = float(rospy.get_param("~scene_rank_min_top_score", 0.0))

        for current in self.scene_targets:
            target_object = current.get("target_object", {})
            target_id, id_source = self._resolve_graspnet_target_id(target_object)
            if target_id is None:
                self.scene_failed_targets.append({
                    "target_key": current.get("target_key"),
                    "label": current.get("label", "unknown"),
                    "semantic_label": current.get("semantic_label", "unknown"),
                    "reason": "Cannot resolve target id for simple point-cloud grasp",
                })
                rospy.logwarn(
                    "[Brain] Scene ranking target failed | target=%s semantic=%s reason=%s",
                    current.get("label", "unknown"),
                    current.get("semantic_label", "unknown"),
                    "Cannot resolve target id for simple point-cloud grasp",
                )
                continue

            rospy.loginfo(
                "[Brain] Stage action: estimate simple grasp from point cloud | target_id=%d id_source=%s label=%s semantic=%s",
                int(target_id),
                id_source,
                current.get("label", "unknown"),
                current.get("semantic_label", "unknown"),
            )

            labels = self.latest_simple_cloud_labels
            points = self.latest_simple_cloud_points
            if labels is None or points is None or len(points) == 0:
                reason = "Simple grasp cloud is empty or unlabeled"
                self.scene_failed_targets.append({
                    "target_key": current.get("target_key"),
                    "label": current.get("label", "unknown"),
                    "semantic_label": current.get("semantic_label", "unknown"),
                    "reason": reason,
                })
                rospy.logwarn(
                    "[Brain] Scene ranking target failed | target=%s semantic=%s reason=%s",
                    current.get("label", "unknown"),
                    current.get("semantic_label", "unknown"),
                    reason,
                )
                continue

            mask = labels == int(target_id)
            target_points = points[mask]
            grasp_candidate, error = estimate_simple_topdown_grasp(
                target_points,
                current.get("label", "unknown"),
                frame_id=self.latest_simple_cloud_frame_id or "base_link",
            )
            if grasp_candidate is None:
                self.scene_failed_targets.append({
                    "target_key": current.get("target_key"),
                    "label": current.get("label", "unknown"),
                    "semantic_label": current.get("semantic_label", "unknown"),
                    "reason": error,
                })
                rospy.logwarn(
                    "[Brain] Scene ranking target failed | target=%s semantic=%s reason=%s",
                    current.get("label", "unknown"),
                    current.get("semantic_label", "unknown"),
                    error,
                )
                continue

            scored = dict(grasp_candidate)
            scored.update(compute_grasp_geometry_scores(grasp_candidate["pose"], grasp_candidate["quality"]))
            if float(scored["pose_z"]) < min_pose_z or float(scored["top_score"]) <= min_top_score:
                reason = (
                    "No candidates remained after scene-ranking pose filtering "
                    f"(min_pose_z={min_pose_z:.3f}, min_top_score>{min_top_score:.3f})"
                )
                self.scene_failed_targets.append({
                    "target_key": current.get("target_key"),
                    "label": current.get("label", "unknown"),
                    "semantic_label": current.get("semantic_label", "unknown"),
                    "reason": reason,
                })
                rospy.logwarn(
                    "[Brain] Scene ranking target failed | target=%s semantic=%s reason=%s",
                    current.get("label", "unknown"),
                    current.get("semantic_label", "unknown"),
                    reason,
                )
                continue

            self.scene_ranked_results.append({
                "target_object": current.get("target_object", {}),
                "target_point_base": current.get("target_point_base"),
                "target_key": current.get("target_key"),
                "label": current.get("label", "unknown"),
                "semantic_label": current.get("semantic_label", "unknown"),
                "pick_category": current.get("pick_category", "non_cube"),
                "score": current.get("score", 0.0),
                "confidence": current.get("confidence", 0.0),
                "best_candidate": scored,
                "candidate_count": 1,
            })
            rospy.loginfo(
                "[Brain] Scene ranking target complete | target=%s semantic=%s points=%d top=%.3f side=%.3f quality=%.3f z=%.3f width=%.3f",
                current.get("label", "unknown"),
                current.get("semantic_label", "unknown"),
                int(grasp_candidate["metadata"]["point_count"]),
                scored["top_score"],
                scored["robot_side_score"],
                scored["quality"],
                scored["pose_z"],
                scored["width"],
            )

        return self._finish_scene_ranking(blackboard)

    def update(self):
        if is_test_mode():
            if not self.request_sent:
                mock_pose = PoseStamped()
                mock_pose.header.frame_id = "base_link"
                mock_pose.pose.position.x = 0.35
                mock_pose.pose.position.y = -0.10
                mock_pose.pose.position.z = 0.30
                mock_pose.pose.orientation.w = 1.0
                get_blackboard().set("target_grasp_pose", mock_pose)
                self.request_sent = True
            return Status.SUCCESS

        blackboard = get_blackboard()
        if self.grasp_pose_source == "scene_simple":
            return self._update_scene_wide_non_cube_ranking(blackboard)

        target_object = blackboard_get("target_object", {}) or {}
        target_label = (
            target_object.get("name", "unknown")
            if isinstance(target_object, dict)
            else str(target_object)
        )

        if self.grasp_pose_source == "graspnet":
            return self._update_graspnet_pose(blackboard, target_object)

        target_point = blackboard_get("target_point_base_link")
        if target_point is None:
            return Status.RUNNING

        if self.request_sent:
            return Status.SUCCESS

        return self._build_direct_yolo_grasp_pose(blackboard, target_point, target_label, target_object)


class ExecutePickAndPlaceBehavior(py_trees.behaviour.Behaviour):
    """Approach, grasp, return to overview, move to bin, then release."""

    def __init__(self, name="ExecutePickAndPlace"):
        super().__init__(name)
        self.client = None
        self.motion_cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
        self.motion_result_sub = rospy.Subscriber(
            "/motion/result", GraspResult, self._motion_result_callback
        )
        self.gripper_cmd_pub = rospy.Publisher("/gripper/command", String, queue_size=1)
        self.gripper_grasp_srv = rospy.ServiceProxy("/gripper/grasp", Trigger)
        self.gripper_release_srv = rospy.ServiceProxy("/gripper/release", Trigger)
        self.gripper_is_holding_srv = rospy.ServiceProxy("/gripper/is_holding", Trigger)
        self.goal_sent = False
        self.direct_motion_sent = False
        self.return_overview_sent = False
        self.bin_motion_sent = False
        self.mock_done = False
        self.last_feedback_stage = None
        self.last_motion_result = None
        self.motion_result_count = 0
        self.direct_motion_start_count = 0
        self.return_overview_start_count = 0
        self.bin_motion_start_count = 0
        self.phase = "approach"
        self.place_bin_color = "green"
        self.place_bin_joints = []
        self.is_cube = False
        self.cube_label = ""
        self.cube_scale_joints = []
        self.cube_final_bin_color = ""
        self.is_scale_retrieval = False

    def setup(self, timeout=None):
        if is_test_mode():
            return True
        self.client = actionlib.SimpleActionClient(
            "/path_planning/plan_execute_pose", PlanExecutePoseAction
        )
        return True

    def _motion_result_callback(self, msg):
        self.last_motion_result = msg
        self.motion_result_count += 1

    def _feedback_callback(self, feedback):
        stage_name = (
            "Planning"
            if feedback.stage == PlanExecutePoseFeedback.PLANNING
            else "Executing"
        )
        if stage_name == self.last_feedback_stage:
            return
        self.last_feedback_stage = stage_name
        rospy.loginfo("[Brain] Stage: %s", stage_name)

    def initialise(self):
        self.goal_sent = False
        self.direct_motion_sent = False
        self.return_overview_sent = False
        self.bin_motion_sent = False
        self.mock_done = False
        self.last_feedback_stage = None
        self.direct_motion_start_count = self.motion_result_count
        self.return_overview_start_count = self.motion_result_count
        self.bin_motion_start_count = self.motion_result_count
        self.phase = "approach"
        self.place_bin_color = normalize_label(rospy.get_param("~place_bin_color", "green")) or "green"
        default_bin_joints = self._get_bin_joints(self.place_bin_color)
        self.place_bin_joints = list(default_bin_joints)
        self.motion_control_active_grasp_category_param = rospy.get_param(
            "~motion_control_active_grasp_category_param",
            "/motion_control/active_grasp_category",
        )
        self.motion_control_active_grasp_width_param = rospy.get_param(
            "~motion_control_active_grasp_width_param",
            "/motion_control/active_grasp_target_width_m",
        )
        self.is_cube = False
        self.cube_label = ""
        self.cube_scale_joints = []
        self.cube_final_bin_color = ""
        self.is_scale_retrieval = False

        log_stage("ApproachTarget")
        target_object = blackboard_get("target_object", {}) or {}
        target_label = (
            target_object.get("name", "unknown")
            if isinstance(target_object, dict)
            else str(target_object)
        )
        grasp_category = pick_category_for_label(target_label)

        cube_on_scale = blackboard_get(CUBE_ON_SCALE_BB, False)
        if cube_on_scale:
            self.is_scale_retrieval = True
            self.is_cube = True
            self.cube_label = target_label
            bin_color = blackboard_get(CUBE_ON_SCALE_BIN_COLOR_BB, "green")
            self.cube_final_bin_color = bin_color
            self.place_bin_joints = self._get_bin_joints_by_color(bin_color)
            rospy.loginfo(
                "[Brain] Scale retrieval | label=%s | -> %s_bin [%s]",
                target_label,
                bin_color,
                ", ".join(f"{j:.4f}" for j in self.place_bin_joints),
            )
        elif grasp_category == "cube":
            self.is_cube = True
            self.cube_label = target_label
            self.cube_scale_joints = self._get_cube_place_joints(default_bin_joints)
            self.place_bin_joints = list(self.cube_scale_joints)
            self.cube_final_bin_color = cube_bin_color_for_label(target_label)
            rospy.loginfo(
                "[Brain] Cube phase1 | label=%s | -> scale [%s] | later -> %s_bin",
                target_label,
                ", ".join(f"{j:.4f}" for j in self.cube_scale_joints),
                self.cube_final_bin_color,
            )
        else:
            self.place_bin_joints = list(default_bin_joints)
        target_grasp_width_m = blackboard_get("target_grasp_width_m", None)
        active_category = "scale_cube" if self.is_scale_retrieval else grasp_category
        rospy.set_param(self.motion_control_active_grasp_category_param, active_category)
        if grasp_category == "non_cube" and target_grasp_width_m is not None and float(target_grasp_width_m) > 0.0:
            rospy.set_param(self.motion_control_active_grasp_width_param, float(target_grasp_width_m))
        else:
            rospy.set_param(self.motion_control_active_grasp_width_param, -1.0)
        rospy.loginfo(
            "[Brain] Current grasp target label: %s | category=%s | target_width_m=%s | place_bin=%s",
            target_label,
            grasp_category,
            "None" if target_grasp_width_m is None else f"{float(target_grasp_width_m):.4f}",
            self.place_bin_color,
        )

    def _get_bin_joints(self, bin_color):
        defaults = {
            "green": [-0.4696, -1.1530, -1.6126, 1.1743, 1.5815, -0.4750],
            "blue": [1.4744, -1.0557, -1.4196, 0.9190, 1.5769, 1.4693],
        }
        resolved_color = normalize_label(bin_color)
        if resolved_color not in defaults:
            rospy.logwarn(
                "[Brain] Unknown place_bin_color '%s', falling back to green",
                bin_color,
            )
            resolved_color = "green"
        values = rospy.get_param("~%s_bin_joints" % resolved_color, defaults[resolved_color])
        joints = [float(v) for v in values]
        if len(joints) != 6:
            raise ValueError("%s_bin_joints must contain 6 joint values" % resolved_color)
        self.place_bin_color = resolved_color
        return joints

    def _get_bin_joints_by_color(self, bin_color):
        """Return bin joints for the given color without modifying self.place_bin_color."""
        defaults = {
            "green": [-0.4696, -1.1530, -1.6126, 1.1743, 1.5815, -0.4750],
            "blue": [1.4744, -1.0557, -1.4196, 0.9190, 1.5769, 1.4693],
        }
        resolved_color = normalize_label(bin_color)
        if resolved_color not in defaults:
            resolved_color = "green"
        values = rospy.get_param("~%s_bin_joints" % resolved_color, defaults[resolved_color])
        joints = [float(v) for v in values]
        if len(joints) != 6:
            return defaults.get(resolved_color, defaults["green"])
        return joints

    def _get_cube_place_joints(self, fallback_joints):
        defaults = [0.3000, 0.7006, 1.5000, -2.5000, 0.0002, 0.0000]
        values = rospy.get_param("~cube_place_joints", defaults)
        try:
            joints = [float(v) for v in values]
        except (TypeError, ValueError):
            rospy.logwarn("[Brain] cube_place_joints is invalid, using default bin joints")
            return list(fallback_joints)
        if len(joints) != 6:
            rospy.logwarn("[Brain] cube_place_joints must contain 6 joint values, using default bin joints")
            return list(fallback_joints)
        return joints

    @staticmethod
    def _is_interface_failure(message):
        text = str(message).lower()
        return (
            "unavailable" in text
            or "not reachable" in text
            or "timeout" in text and "gripper" in text
            or "no gripper" in text
            or text.startswith("unknown:")
        )

    def _call_gripper_grasp(self):
        try:
            rospy.wait_for_service("/gripper/grasp", timeout=0.5)
            response = self.gripper_grasp_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: command gripper grasp")
                return True, response.message or "grasp_success"
            return False, response.message or "grasp_failed"
        except (rospy.ROSException, rospy.ServiceException):
            if self.gripper_cmd_pub.get_num_connections() > 0:
                self.gripper_cmd_pub.publish(String(data="grasp"))
                rospy.logwarn(
                    "[Brain] /gripper/grasp service unavailable, published grasp command instead"
                )
                return True, "grasp_command_published"
            return False, "gripper grasp interface unavailable"

    def _call_gripper_release(self):
        try:
            rospy.wait_for_service("/gripper/release", timeout=0.5)
            response = self.gripper_release_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: command gripper release")
                return True, response.message or "release_success"
            return False, response.message or "release_failed"
        except (rospy.ROSException, rospy.ServiceException):
            if self.gripper_cmd_pub.get_num_connections() > 0:
                self.gripper_cmd_pub.publish(String(data="release"))
                rospy.logwarn(
                    "[Brain] /gripper/release service unavailable, published release command instead"
                )
                return True, "release_command_published"
            return False, "gripper release interface unavailable"

    def _send_direct_motion_goal(self, target_pose, reason):
        if self.motion_cmd_pub.get_num_connections() == 0:
            return False

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_POSE
        cmd.target_pose = target_pose
        cmd.max_velocity = float(rospy.get_param("~direct_motion_max_velocity", 0.6))
        cmd.max_acceleration = float(
            rospy.get_param("~direct_motion_max_acceleration", 0.6)
        )
        cmd.collision_check = False

        log_stage("DirectMotionFallback")
        rospy.loginfo(
            "[Brain] Stage action: send pose directly to motion_control | reason=%s",
            reason,
        )
        self.last_motion_result = None
        self.direct_motion_start_count = self.motion_result_count
        self.motion_cmd_pub.publish(cmd)
        self.direct_motion_sent = True
        return True

    def _send_overview_joint_goal(self):
        if self.motion_cmd_pub.get_num_connections() == 0:
            return False

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_JOINT
        cmd.joint_positions = list(OVERVIEW_JOINTS)
        cmd.max_velocity = float(rospy.get_param("~return_overview_max_velocity", 1.0))
        cmd.max_acceleration = float(
            rospy.get_param("~return_overview_max_acceleration", 1.0)
        )
        cmd.collision_check = True

        log_stage("ReturnToOverview")
        rospy.loginfo(
            "[Brain] Stage action: return to overview joints [%s]",
            ", ".join(f"{joint:.4f}" for joint in OVERVIEW_JOINTS),
        )
        self.last_motion_result = None
        self.return_overview_start_count = self.motion_result_count
        self.motion_cmd_pub.publish(cmd)
        self.return_overview_sent = True
        return True

    def _send_bin_joint_goal(self):
        if self.motion_cmd_pub.get_num_connections() == 0:
            return False

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_JOINT
        cmd.joint_positions = list(self.place_bin_joints)
        cmd.max_velocity = float(rospy.get_param("~place_motion_max_velocity", 1.0))
        cmd.max_acceleration = float(rospy.get_param("~place_motion_max_acceleration", 1.0))
        cmd.collision_check = True

        log_stage("MoveToPlaceBin")
        rospy.loginfo(
            "[Brain] Stage action: move to %s bin joints [%s]",
            self.place_bin_color,
            ", ".join(f"{joint:.4f}" for joint in self.place_bin_joints),
        )
        self.last_motion_result = None
        self.bin_motion_start_count = self.motion_result_count
        self.motion_cmd_pub.publish(cmd)
        self.bin_motion_sent = True
        return True

    def _recover(self, reason, blacklist=False):
        prepare_retry_from_overview(reason, blacklist=blacklist)
        return Status.FAILURE

    def _after_target_reached(self, completion_label, trajectory=None):
        blackboard = get_blackboard()
        if trajectory is not None:
            blackboard.set("executed_trajectory", trajectory)

        success, message = self._call_gripper_grasp()
        if not success:
            if self._is_interface_failure(message):
                set_terminal_failure(f"Gripper grasp failed: {message}")
                rospy.logerr("[Brain] Gripper grasp failed: %s", message)
                return Status.FAILURE
            return self._recover(f"gripper grasp command failed: {message}", blacklist=False)

        blackboard.set("holding_object", True)
        rospy.loginfo("[Brain] Stage complete: %s", completion_label)
        rospy.sleep(float(rospy.get_param("~post_grasp_settle_time", 0.3)))
        self.phase = "return_overview"
        return Status.RUNNING

    def update(self):
        if is_test_mode():
            if not self.mock_done:
                self.mock_done = True
            return Status.SUCCESS

        blackboard = get_blackboard()
        target_pose = blackboard_get("target_grasp_pose")
        if target_pose is None:
            return Status.RUNNING
        if not isinstance(target_pose, PoseStamped):
            set_terminal_failure("target_grasp_pose is not a PoseStamped")
            rospy.logerr(
                "[Brain] target_grasp_pose is not a PoseStamped: %r", type(target_pose)
            )
            return Status.FAILURE

        if self.phase == "return_overview":
            if not self.return_overview_sent:
                if self._send_overview_joint_goal():
                    return Status.RUNNING
                set_terminal_failure("motion_control is not reachable for return_overview")
                rospy.logerr("[Brain] motion_control is not reachable for return_overview")
                return Status.FAILURE

            if self.motion_result_count <= self.return_overview_start_count:
                return Status.RUNNING

            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                self.return_overview_sent = False
                rospy.loginfo("[Brain] Stage complete: ReturnToOverview")
                self.phase = "move_to_bin"
                return Status.RUNNING

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "return_overview motion failed"
            )
            return self._recover(f"return to overview failed: {failure_message}", blacklist=False)

        if self.phase == "move_to_bin":
            if not self.bin_motion_sent:
                if self._send_bin_joint_goal():
                    return Status.RUNNING
                set_terminal_failure("motion_control is not reachable for bin placement")
                rospy.logerr("[Brain] motion_control is not reachable for bin placement")
                return Status.FAILURE

            if self.motion_result_count <= self.bin_motion_start_count:
                return Status.RUNNING

            self.bin_motion_sent = False
            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                self.phase = "release"
                log_stage("ReleaseToBin")
                return Status.RUNNING

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "bin motion failed"
            )
            return self._recover(
                f"{self.place_bin_color} bin motion failed: {failure_message}",
                blacklist=False,
            )

        if self.phase == "release":
            success, message = self._call_gripper_release()
            if not success:
                if self._is_interface_failure(message):
                    set_terminal_failure(f"Gripper release failed: {message}")
                    rospy.logerr("[Brain] Gripper release failed: %s", message)
                    return Status.FAILURE
                return self._recover(f"release at {self.place_bin_color} bin failed: {message}")

            blackboard.set("holding_object", False)
            rospy.set_param(self.motion_control_active_grasp_category_param, '')
            rospy.set_param(self.motion_control_active_grasp_width_param, -1.0)

            if self.is_cube and not self.is_scale_retrieval:
                blackboard.set(CUBE_ON_SCALE_BB, True)
                blackboard.set(CUBE_ON_SCALE_LABEL_BB, self.cube_label)
                blackboard.set(CUBE_ON_SCALE_BIN_COLOR_BB, self.cube_final_bin_color)
                blackboard.set(LAST_SUCCESSFUL_PICK_CATEGORY_BB, "cube")
                rospy.loginfo(
                    "[Brain] Stage complete: PlaceCubeToScale | label=%s | next cycle will retrieve -> %s_bin",
                    self.cube_label,
                    self.cube_final_bin_color,
                )
                return Status.SUCCESS

            if self.is_scale_retrieval:
                blackboard.set("placed_bin_color", self.cube_final_bin_color)
                blackboard.set(LAST_SUCCESSFUL_PICK_CATEGORY_BB, "cube")
                rospy.loginfo(
                    "[Brain] Stage complete: CubeScaleRetrieval done | label=%s -> %s_bin | keeping cube_on_scale=True for re-check",
                    self.cube_label,
                    self.cube_final_bin_color,
                )
                return Status.SUCCESS

            blackboard.set("placed_bin_color", self.place_bin_color)
            target_object = blackboard_get("target_object", {}) or {}
            target_label = (
                target_object.get("name", "unknown")
                if isinstance(target_object, dict)
                else str(target_object)
            )
            blackboard.set(
                LAST_SUCCESSFUL_PICK_CATEGORY_BB,
                pick_category_for_label(target_label),
            )
            rospy.loginfo("[Brain] Stage complete: PlaceTo%sBin", self.place_bin_color.capitalize())
            return Status.SUCCESS

        if self.direct_motion_sent:
            if self.motion_result_count <= self.direct_motion_start_count:
                return Status.RUNNING
            self.direct_motion_sent = False
            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                return self._after_target_reached("DirectMotionFallback")

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "motion_control direct move failed"
            )
            return self._recover(f"direct motion failed: {failure_message}", blacklist=True)

        if not self.goal_sent:
            if self.client is not None and self.client.wait_for_server(rospy.Duration(0.1)):
                goal = PlanExecutePoseGoal()
                goal.target_pose = target_pose
                goal.position_only = rospy.get_param("~path_planning_position_only", False)

                rospy.loginfo("[Brain] Stage action: send pose to path_planning")
                self.client.send_goal(goal, feedback_cb=self._feedback_callback)
                self.goal_sent = True
                return Status.RUNNING

            if self._send_direct_motion_goal(target_pose, "path_planning server unavailable"):
                return Status.RUNNING

            set_terminal_failure("Neither path_planning nor motion_control is reachable")
            rospy.logerr("[Brain] No path_planning server or direct motion_control connection available")
            return Status.FAILURE

        state = self.client.get_state()
        if state in (action_msgs.GoalStatus.ACTIVE, action_msgs.GoalStatus.PENDING):
            return Status.RUNNING

        result = self.client.get_result()
        self.goal_sent = False

        if state == action_msgs.GoalStatus.SUCCEEDED and result is not None and result.success:
            return self._after_target_reached("PathPlanning", trajectory=result.trajectory)

        if result is not None and result.status == PlanExecutePoseResult.PREEMPTED:
            failure_message = result.message or "path_planning preempted"
        elif result is not None:
            failure_message = result.message or "path_planning failed"
        else:
            failure_message = "path_planning action failed without a result"

        rospy.logwarn(
            "[Brain] Path planning failed, trying direct motion fallback: %s",
            failure_message,
        )
        if self._send_direct_motion_goal(target_pose, failure_message):
            return Status.RUNNING

        set_terminal_failure("Direct motion fallback could not be started")
        rospy.logerr("[Brain] Direct motion fallback could not be started")
        return Status.FAILURE


def create_behavior_tree():
    children = [
        MoveToOverviewBehavior(),
        EvaluateTargetsBehavior(),
        MoveToNonCubePreObserveBehavior(),
        RequestGraspPoseBehavior(),
    ]
    if not (is_scene_wide_non_cube_ranking_enabled() and not should_execute_after_scene_ranking()):
        children.append(ExecutePickAndPlaceBehavior())

    main_sequence = py_trees.composites.Sequence(
        name="MainTaskSequence",
        memory=True,
        children=children,
    )
    root = py_trees.composites.Selector(
        name="TaskOrRecovery",
        children=[main_sequence],
    )
    return root


def initialise_blackboard_state():
    blackboard = get_blackboard()
    blackboard.set("overview_done", False)
    blackboard.set("holding_object", False)
    blackboard.set("placed_bin_color", "")
    blackboard.set("detected_objects", [])
    blackboard.set(LATEST_YOLO_FRAME_ID_BB, "")
    blackboard.set(LATEST_YOLO_STAMP_BB, rospy.Time(0))
    blackboard.set(LATEST_YOLO_RECV_TIME_BB, rospy.Time(0))
    blackboard.set(FAILED_TARGET_KEYS_BB, [])
    blackboard.set(LAST_SUCCESSFUL_PICK_CATEGORY_BB, "")
    blackboard.set(CUBE_ON_SCALE_BB, False)
    blackboard.set(CUBE_ON_SCALE_LABEL_BB, "")
    blackboard.set(CUBE_ON_SCALE_BIN_COLOR_BB, "")
    blackboard.set("last_target_failure_reason", "")
    blackboard.set("task_retry_requested", False)
    blackboard.set("task_retry_reason", "")
    clear_terminal_failure()
    clear_target_selection()
    clear_scene_ranking_state()


def main():
    rospy.init_node("robocup_brain", anonymous=False)
    rospy.loginfo("=" * 50)
    rospy.loginfo("RoboCup Brain Node Starting")
    rospy.loginfo("=" * 50)
    rospy.loginfo("[Brain] test_mode=%s", is_test_mode())

    single_cycle = rospy.get_param("~single_cycle", True)
    loop_until_no_targets = rospy.get_param("~loop_until_no_targets", True)
    scene_wide_non_cube_ranking = is_scene_wide_non_cube_ranking_enabled()
    execute_after_scene_ranking = should_execute_after_scene_ranking()
    initialise_blackboard_state()

    root = create_behavior_tree()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    if hasattr(behaviour_tree, "setup"):
        behaviour_tree.setup(timeout=15)

    rate = rospy.Rate(10)
    task_finished = False

    rospy.loginfo("[Brain] Behavior Tree initialized. Starting main loop...")

    try:
        while not rospy.is_shutdown():
            if task_finished:
                rate.sleep()
                continue

            behaviour_tree.tick()

            if root.status == Status.SUCCESS:
                if scene_wide_non_cube_ranking and not execute_after_scene_ranking:
                    rospy.loginfo(
                        "[Brain] Scene ranking cycle complete. Refreshing non-cube ranking from overview."
                    )
                    prepare_next_scene_ranking_cycle()
                    root.stop(Status.INVALID)
                elif loop_until_no_targets:
                    rospy.loginfo("[Brain] Cycle complete. Returning to overview for next target.")
                    prepare_next_pick_cycle()
                    root.stop(Status.INVALID)
                elif single_cycle:
                    rospy.loginfo("[Brain] Task complete. Holding after single cycle.")
                    task_finished = True
            elif root.status == Status.FAILURE:
                if blackboard_get("task_complete_no_targets", False):
                    reason = blackboard_get("task_complete_no_targets_reason", "No targets remain")
                    rospy.loginfo("[Brain] Task complete. %s", reason)
                    task_finished = True
                elif blackboard_get("task_terminal_failure", False):
                    reason = blackboard_get("task_terminal_failure_reason", "unknown")
                    rospy.logwarn(
                        "[Brain] Terminal failure. Holding after single cycle: %s",
                        reason,
                    )
                    task_finished = True
                else:
                    reason = blackboard_get(
                        "task_retry_reason",
                        blackboard_get("last_target_failure_reason", "unknown"),
                    )
                    if blackboard_get("overview_done", False):
                        rospy.logwarn(
                            "[Brain] Non-terminal failure. Retrying from current overview state: %s",
                            reason,
                        )
                    else:
                        rospy.logwarn(
                            "[Brain] Non-terminal failure. Returning to overview and retrying: %s",
                            reason,
                        )
                    get_blackboard().set("task_retry_requested", False)
                    get_blackboard().set("task_retry_reason", "")
                    root.stop(Status.INVALID)

            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[Brain] Shutting down...")


if __name__ == "__main__":
    main()
