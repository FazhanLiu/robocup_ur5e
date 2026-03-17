#!/usr/bin/env python3
"""
RoboCup Brain Node - behavior-tree based task orchestration.

Current pick flow:
  1. Move to HOME once.
  2. Open gripper once.
  3. Pick the best YOLO target that is not blacklisted.
  4. Build a direct grasp pose from YOLO 3D position.
  5. Try path_planning first.
  6. If path_planning fails, fall back to direct motion_control MOVE_TO_POSE.
  7. If both fail, blacklist the target and try another one.
  8. After reaching the target, close gripper and stop on success.
"""

import json

import actionlib
import actionlib_msgs.msg as action_msgs
import py_trees
import py_trees_ros
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from py_trees.common import Status
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler

from common_msgs.msg import (
    GraspResult,
    MotionCommand,
    PlanExecutePoseAction,
    PlanExecutePoseFeedback,
    PlanExecutePoseGoal,
    PlanExecutePoseResult,
)


FAILED_TARGET_KEYS_BB = "failed_target_keys"
TARGET_BB_KEYS = (
    "target_object",
    "target_point_base_link",
    "target_grasp_pose",
    "target_grasp_mode",
    "target_key",
    "target_score",
    "target_confidence",
)


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


def clear_terminal_failure():
    blackboard = get_blackboard()
    blackboard.set("task_terminal_failure", False)
    blackboard.set("task_terminal_failure_reason", "")


def set_terminal_failure(reason):
    blackboard = get_blackboard()
    blackboard.set("task_terminal_failure", True)
    blackboard.set("task_terminal_failure_reason", reason)


def get_failed_target_keys():
    values = blackboard_get(FAILED_TARGET_KEYS_BB, [])
    return set(str(item) for item in values)


def set_failed_target_keys(keys):
    get_blackboard().set(FAILED_TARGET_KEYS_BB, sorted(str(item) for item in keys))


def build_target_key(label, point_base):
    return "{label}:{x:.2f}:{y:.2f}:{z:.2f}".format(
        label=str(label).lower(),
        x=float(point_base.point.x),
        y=float(point_base.point.y),
        z=float(point_base.point.z),
    )


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


class MoveToOverviewBehavior(py_trees.behaviour.Behaviour):
    """Move to HOME once, then open the gripper once."""

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
        self.command_sent = False
        self.last_result = None
        self.release_done = False

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

    def _open_gripper(self):
        try:
            rospy.wait_for_service("/gripper/release", timeout=0.5)
            response = self.gripper_release_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: open gripper after HOME")
                return True
            rospy.logerr("[Brain] Failed to open gripper after HOME: %s", response.message)
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
            cmd.command_type = MotionCommand.HOME
            cmd.max_velocity = 1.0
            cmd.max_acceleration = 1.0
            cmd.collision_check = True
            self.motion_cmd_pub.publish(cmd)
            self.command_sent = True
            return Status.RUNNING

        if self.last_result is None:
            return Status.RUNNING

        if self.last_result.status != GraspResult.SUCCESS:
            reason = self.last_result.message or "HOME motion failed"
            rospy.logerr("[Brain] Overview move failed: %s", reason)
            set_terminal_failure(reason)
            return Status.FAILURE

        if not self.release_done:
            if not self._open_gripper():
                reason = "Failed to open gripper after HOME"
                set_terminal_failure(reason)
                return Status.FAILURE
            self.release_done = True

        blackboard.set("overview_done", True)
        rospy.loginfo("[Brain] Stage complete: MoveToOverview")
        return Status.SUCCESS


class EvaluateTargetsBehavior(py_trees.behaviour.Behaviour):
    """Receive YOLO detections, transform them, and pick the best non-blacklisted target."""

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

    def setup(self, timeout=None):
        if is_test_mode():
            return True
        self.sub = rospy.Subscriber(
            "/perception/yolo26_seg_detections",
            String,
            self._yolo_callback,
        )
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
        stamp_dict = first.get("gazebo_stamp", {})
        secs = int(stamp_dict.get("secs", 0))
        nsecs = int(stamp_dict.get("nsecs", 0))
        self.latest_stamp = (
            rospy.Time(secs=secs, nsecs=nsecs) if (secs or nsecs) else rospy.Time(0)
        )

    def _score_object(self, obj_label):
        score_map = {
            "green_cube": 100,
            "purple_cube": 100,
            "red_can": 80,
            "red_bottle": 80,
            "yellow_can": 60,
            "spam": 60,
            "green_can": 40,
            "blue_bottle": 40,
        }
        return score_map.get(str(obj_label).lower(), 1)

    def initialise(self):
        self.mock_done = False
        self.latest_detections = []
        self.latest_frame_id = ""
        self.latest_stamp = rospy.Time(0)
        clear_target_selection()
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

        if not self.latest_detections:
            return Status.RUNNING

        failed_target_keys = get_failed_target_keys()
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

        best_target = None
        best_point_base = None
        best_target_key = None
        highest_score = -1.0
        highest_confidence = -1.0

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
            target_key = build_target_key(label, pt_base)
            if target_key in failed_target_keys:
                continue

            confidence = float(obj.get("confidence", 0.0))
            score = self._score_object(label)
            if score > highest_score or (
                score == highest_score and confidence > highest_confidence
            ):
                highest_score = score
                highest_confidence = confidence
                best_target = obj
                best_point_base = pt_base
                best_target_key = target_key

        if best_target is None:
            rospy.logwarn_throttle(
                2.0,
                "[Brain] No selectable YOLO target right now | labels=%s blacklisted=%d",
                ", ".join(latest_labels),
                len(failed_target_keys),
            )
            return Status.RUNNING

        clear_terminal_failure()
        blackboard = get_blackboard()
        blackboard.set("target_object", best_target)
        blackboard.set("target_point_base_link", best_point_base)
        blackboard.set("target_key", best_target_key)
        blackboard.set("target_score", highest_score)
        blackboard.set("target_confidence", highest_confidence)
        self.yolo_enable_pub.publish(Bool(data=False))
        rospy.loginfo("[Brain] Stage action: disable YOLO perception")
        rospy.loginfo(
            "[Brain] Stage complete: EvaluateTargets | selected=%s score=%s confidence=%.3f",
            best_target.get("name", "unknown"),
            highest_score,
            highest_confidence,
        )
        return Status.SUCCESS


class RequestGraspPoseBehavior(py_trees.behaviour.Behaviour):
    """Build a simple grasp pose directly from the selected YOLO 3D point."""

    def __init__(self, name="RequestGraspPose"):
        super().__init__(name)
        self.request_sent = False

    def initialise(self):
        self.request_sent = False
        log_stage("BuildDirectGraspPose")

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
        target_point = blackboard_get("target_point_base_link")
        if target_point is None:
            return Status.RUNNING

        if self.request_sent:
            return Status.SUCCESS

        target_object = blackboard_get("target_object", {}) or {}
        target_label = (
            target_object.get("name", "unknown")
            if isinstance(target_object, dict)
            else str(target_object)
        )

        grasp_offset_x = float(rospy.get_param("~direct_grasp_offset_x", 0.0))
        grasp_offset_y = float(rospy.get_param("~direct_grasp_offset_y", 0.0))
        grasp_offset_z = float(rospy.get_param("~direct_grasp_offset_z", 0.02))
        grasp_min_z = float(rospy.get_param("~direct_grasp_min_z", 0.05))
        grasp_max_z = float(rospy.get_param("~direct_grasp_max_z", 1.20))
        grasp_roll = float(rospy.get_param("~direct_grasp_roll", 3.141592653589793))
        grasp_pitch = float(rospy.get_param("~direct_grasp_pitch", 0.0))
        grasp_yaw = float(rospy.get_param("~direct_grasp_yaw", 0.0))

        raw_z = float(target_point.point.z) + grasp_offset_z
        clamped_z = max(grasp_min_z, min(grasp_max_z, raw_z))
        qx, qy, qz, qw = quaternion_from_euler(grasp_roll, grasp_pitch, grasp_yaw)

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_link"
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.pose.position.x = float(target_point.point.x) + grasp_offset_x
        grasp_pose.pose.position.y = float(target_point.point.y) + grasp_offset_y
        grasp_pose.pose.position.z = clamped_z
        grasp_pose.pose.orientation = Quaternion(qx, qy, qz, qw)

        blackboard.set("target_grasp_pose", grasp_pose)
        blackboard.set("target_grasp_mode", "direct_yolo_point")

        if abs(clamped_z - raw_z) > 1e-6:
            rospy.logwarn(
                "[Brain] Clamped target z from %.3f to %.3f for %s",
                raw_z,
                clamped_z,
                target_label,
            )

        rospy.loginfo(
            "[Brain] Stage complete: BuildDirectGraspPose | target=%s pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
            target_label,
            grasp_pose.pose.position.x,
            grasp_pose.pose.position.y,
            grasp_pose.pose.position.z,
            grasp_roll,
            grasp_pitch,
            grasp_yaw,
        )
        self.request_sent = True
        return Status.SUCCESS


class ExecutePickAndPlaceBehavior(py_trees.behaviour.Behaviour):
    """Try path planning first, then fall back to direct motion, then close gripper."""

    def __init__(self, name="ExecutePickAndPlace"):
        super().__init__(name)
        self.client = None
        self.motion_cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
        self.motion_result_sub = rospy.Subscriber(
            "/motion/result", GraspResult, self._motion_result_callback
        )
        self.gripper_cmd_pub = rospy.Publisher("/gripper/command", String, queue_size=1)
        self.gripper_grasp_srv = rospy.ServiceProxy("/gripper/grasp", Trigger)
        self.goal_sent = False
        self.direct_motion_sent = False
        self.mock_done = False
        self.last_feedback_stage = None
        self.last_motion_result = None
        self.motion_result_count = 0
        self.direct_motion_start_count = 0

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
        self.mock_done = False
        self.last_feedback_stage = None
        self.direct_motion_start_count = self.motion_result_count
        log_stage("PathPlanning")

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

    def _handle_reach_success(self, completion_label, trajectory=None):
        success, message = self._call_gripper_grasp()
        if not success:
            set_terminal_failure(f"Gripper grasp failed: {message}")
            rospy.logerr("[Brain] Gripper grasp failed: %s", message)
            return Status.FAILURE

        blackboard = get_blackboard()
        if trajectory is not None:
            blackboard.set("executed_trajectory", trajectory)
        rospy.loginfo("[Brain] Stage complete: %s", completion_label)
        return Status.SUCCESS

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

        if self.direct_motion_sent:
            if self.motion_result_count <= self.direct_motion_start_count:
                return Status.RUNNING
            self.direct_motion_sent = False
            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                return self._handle_reach_success("DirectMotionFallback")

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "motion_control direct move failed"
            )
            blacklist_current_target(f"direct motion failed: {failure_message}")
            return Status.FAILURE

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
            return self._handle_reach_success("PathPlanning", trajectory=result.trajectory)

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
    main_sequence = py_trees.composites.Sequence(
        name="MainTaskSequence",
        memory=True,
        children=[
            MoveToOverviewBehavior(),
            EvaluateTargetsBehavior(),
            RequestGraspPoseBehavior(),
            ExecutePickAndPlaceBehavior(),
        ],
    )
    root = py_trees.composites.Selector(
        name="TaskOrRecovery",
        children=[main_sequence],
    )
    return root


def initialise_blackboard_state():
    blackboard = get_blackboard()
    blackboard.set("overview_done", False)
    blackboard.set("detected_objects", [])
    blackboard.set(FAILED_TARGET_KEYS_BB, [])
    blackboard.set("last_target_failure_reason", "")
    clear_terminal_failure()
    clear_target_selection()


def main():
    rospy.init_node("robocup_brain", anonymous=False)
    rospy.loginfo("=" * 50)
    rospy.loginfo("RoboCup Brain Node Starting")
    rospy.loginfo("=" * 50)
    rospy.loginfo("[Brain] test_mode=%s", is_test_mode())

    single_cycle = rospy.get_param("~single_cycle", True)
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

            if single_cycle and root.status == Status.SUCCESS:
                rospy.loginfo("[Brain] Task complete. Holding after single cycle.")
                task_finished = True
            elif root.status == Status.FAILURE:
                if blackboard_get("task_terminal_failure", False):
                    reason = blackboard_get("task_terminal_failure_reason", "unknown")
                    rospy.logwarn(
                        "[Brain] Terminal failure. Holding after single cycle: %s",
                        reason,
                    )
                    task_finished = True

            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[Brain] Shutting down...")


if __name__ == "__main__":
    main()
