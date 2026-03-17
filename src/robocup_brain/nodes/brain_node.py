#!/usr/bin/env python3
"""
RoboCup Brain Node - 基于行为树的决策系统
Architecture: py_trees_ros
Strategy: 增量扫描 -> 优先级评估(YOLO+TF) -> 请求抓取(GraspNet) -> 放置(MoveIt)
"""

import json
import time

import rospy
import py_trees
import py_trees_ros
from py_trees.common import Status

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
import actionlib
import actionlib_msgs.msg as action_msgs
from std_msgs.msg import String

from common_msgs.msg import (
    MotionCommand,
    GraspResult,
    PlanExecutePoseAction,
    PlanExecutePoseGoal,
    PlanExecutePoseResult,
    PlanExecutePoseFeedback,
)


def is_test_mode():
    return rospy.get_param("~test_mode", False)


def get_blackboard():
    return py_trees.blackboard.Blackboard()


class MoveToOverviewBehavior(py_trees.behaviour.Behaviour):
    """阶段1：移动到高空俯视点，准备拍照"""
    def __init__(self, name="MoveToOverview"):
        super().__init__(name)
        self.motion_cmd_pub = rospy.Publisher('/motion/command', MotionCommand, queue_size=10)
        self.motion_result_sub = rospy.Subscriber('/motion/result', GraspResult, self._motion_result_callback)
        self.command_sent = False
        self.last_result = None

    def _motion_result_callback(self, msg):
        self.last_result = msg

    def initialise(self):
        self.command_sent = False
        self.last_result = None

    def update(self):
        if self.motion_cmd_pub.get_num_connections() == 0:
            rospy.loginfo_throttle(2.0, "[Brain] Waiting for motion_control node...")
            return Status.RUNNING

        if not self.command_sent:
            rospy.loginfo("[Brain] Sending HOME command to motion_control...")
            cmd = MotionCommand()
            cmd.command_type = MotionCommand.HOME
            cmd.max_velocity = 1.0
            cmd.max_acceleration = 1.0
            cmd.collision_check = True
            self.motion_cmd_pub.publish(cmd)
            self.command_sent = True
            return Status.RUNNING

        if self.last_result is None:
            rospy.loginfo_throttle(2.0, "[Brain] Waiting for HOME result...")
            return Status.RUNNING

        if self.last_result.status == GraspResult.SUCCESS:
            rospy.loginfo("[Brain] Overview pose reached")
            return Status.SUCCESS

        rospy.logerr("[Brain] Overview move failed: %s", self.last_result.message)
        return Status.FAILURE


class EvaluateTargetsBehavior(py_trees.behaviour.Behaviour):
    """阶段2：接收YOLO数据，TF转换，按比赛规则打分，选出最佳目标"""
    def __init__(self, name="EvaluateTargets"):
        super().__init__(name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.latest_detections = []
        self.latest_frame_id = ""
        self.latest_stamp = rospy.Time(0)
        self.sub = None
        self.mock_done = False
        self.yolo_callback_count = 0
        self.last_yolo_wall_time = None
        self.last_yolo_payload_size = 0
        self.last_yolo_labels = []

    def setup(self, timeout=None):
        rospy.loginfo("[Brain] EvaluateTargets: Setup")
        if is_test_mode():
            rospy.loginfo("[Brain] EvaluateTargets running in test mode")
            return True
        self.sub = rospy.Subscriber(
            "/perception/yolo26_seg_detections",
            String,
            self._yolo_callback
        )
        rospy.loginfo("[Brain] EvaluateTargets subscribed to /perception/yolo26_seg_detections")
        return True

    def _yolo_callback(self, msg):
        self.yolo_callback_count += 1
        self.last_yolo_wall_time = time.time()
        self.last_yolo_payload_size = len(msg.data)

        try:
            detections = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            rospy.logwarn_throttle(
                2.0,
                "[Brain] Failed to parse YOLO JSON: %s | payload_bytes=%d | preview=%s",
                str(exc),
                self.last_yolo_payload_size,
                msg.data[:160],
            )
            return

        if not isinstance(detections, list):
            rospy.logwarn_throttle(
                2.0,
                "[Brain] YOLO JSON payload is not a list | type=%s | payload_bytes=%d",
                type(detections).__name__,
                self.last_yolo_payload_size,
            )
            return

        self.latest_detections = detections
        self.last_yolo_labels = [str(item.get("name", "unknown")) for item in detections if isinstance(item, dict)]
        blackboard = get_blackboard()
        blackboard.set("detected_objects", detections)

        if not detections:
            rospy.loginfo_throttle(
                1.0,
                "[Brain] YOLO callback #%d received an empty detection list",
                self.yolo_callback_count,
            )
            return

        first = detections[0]
        self.latest_frame_id = str(first.get("frame_id", "")) or rospy.get_param(
            "~perception_frame_id", "camera_rgb_optical_frame"
        )
        stamp_dict = first.get("gazebo_stamp", {})
        secs = int(stamp_dict.get("secs", 0))
        nsecs = int(stamp_dict.get("nsecs", 0))
        self.latest_stamp = rospy.Time(secs=secs, nsecs=nsecs) if (secs or nsecs) else rospy.Time(0)
        rospy.loginfo_throttle(
            1.0,
            "[Brain] YOLO callback #%d: count=%d frame_id=%s stamp=%d.%09d labels=%s",
            self.yolo_callback_count,
            len(detections),
            self.latest_frame_id,
            secs,
            nsecs,
            ", ".join(self.last_yolo_labels),
        )

    def _score_object(self, obj_label):
        """比赛优先级打分系统"""
        score_map = {
            "green_cube": 100, "purple_cube": 100, # Cube 优先级最高（能称重刷分）
            "red_can": 80, "red_bottle": 80,       # 30分高优
            "yellow_can": 60, "spam": 60,          # 20分
            "green_can": 40, "blue_bottle": 40     # 10分保底
        }
        return score_map.get(obj_label.lower(), 1)

    def initialise(self):
        self.mock_done = False

    def _lookup_transform(self, frame_id, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                stamp,
                rospy.Duration(0.5)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                rospy.Time(0),
                rospy.Duration(0.5)
            )

    def update(self):
        if is_test_mode():
            if not self.mock_done:
                rospy.loginfo("[Brain] [Test] Mock YOLO detection complete")
                blackboard = py_trees.blackboard.Blackboard()
                blackboard.set("target_object", "mock_target")
                blackboard.set("target_point_base_link", PointStamped())
                self.mock_done = True
            return Status.SUCCESS

        if len(self.latest_detections) == 0:
            if self.last_yolo_wall_time is None:
                last_callback_text = "never"
            else:
                last_callback_text = f"{time.time() - self.last_yolo_wall_time:.2f}s ago"
            rospy.loginfo_throttle(
                2.0,
                "[Brain] Waiting for YOLO objects... callbacks=%d, last_callback=%s, last_payload_bytes=%d",
                self.yolo_callback_count,
                last_callback_text,
                self.last_yolo_payload_size,
            )
            return Status.RUNNING

        best_target = None
        highest_score = -1.0
        highest_confidence = -1.0
        best_point_base_link = None
        frame_id = self.latest_frame_id or rospy.get_param("~perception_frame_id", "camera_rgb_optical_frame")

        try:
            trans = self._lookup_transform(frame_id, self.latest_stamp)
            rospy.loginfo_throttle(
                1.0,
                "[Brain] Evaluating YOLO list: count=%d frame_id=%s labels=%s",
                len(self.latest_detections),
                frame_id,
                ", ".join(self.last_yolo_labels),
            )
            
            for obj in self.latest_detections:
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
                confidence = float(obj.get("confidence", 0.0))
                score = self._score_object(label)
                if score > highest_score or (score == highest_score and confidence > highest_confidence):
                    highest_score = score
                    highest_confidence = confidence
                    best_target = obj
                    best_point_base_link = pt_base

            if best_target:
                rospy.loginfo(
                    "[Brain] Selected Target: %s, Score: %s, Confidence: %.3f",
                    best_target.get("name", "unknown"),
                    highest_score,
                    highest_confidence,
                )
                blackboard = get_blackboard()
                blackboard.set("detected_objects", list(self.latest_detections))
                blackboard.set("target_object", best_target)
                blackboard.set("target_point_base_link", best_point_base_link)
                return Status.SUCCESS

            rospy.logwarn_throttle(
                2.0,
                "[Brain] YOLO list received but no valid target survived filtering | labels=%s",
                ", ".join(self.last_yolo_labels),
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[Brain] TF Error: {e}")
            return Status.FAILURE

        return Status.RUNNING


class RequestGraspPoseBehavior(py_trees.behaviour.Behaviour):
    """阶段3：直接用 YOLO 3D 点构造一个简化抓取位姿"""
    def __init__(self, name="RequestGraspPose"):
        super().__init__(name)
        self.request_sent = False

    def initialise(self):
        self.request_sent = False

    def update(self):
        if is_test_mode():
            if not self.request_sent:
                rospy.loginfo("[Brain] [Test] Mock grasp pose request complete")
                mock_pose = PoseStamped()
                mock_pose.header.frame_id = "base_link"
                mock_pose.pose.position.x = 0.35
                mock_pose.pose.position.y = -0.10
                mock_pose.pose.position.z = 0.30
                mock_pose.pose.orientation.w = 1.0
                blackboard = get_blackboard()
                blackboard.set("target_grasp_pose", mock_pose)
                self.request_sent = True
            return Status.SUCCESS

        blackboard = get_blackboard()
        target_point = blackboard.get("target_point_base_link")
        if target_point is None:
            rospy.loginfo_throttle(2.0, "[Brain] Waiting for a selected target point...")
            return Status.RUNNING

        if not self.request_sent:
            target_object = blackboard.get("target_object") or {}
            target_label = target_object.get("name", "unknown") if isinstance(target_object, dict) else str(target_object)

            grasp_offset_x = float(rospy.get_param("~direct_grasp_offset_x", 0.0))
            grasp_offset_y = float(rospy.get_param("~direct_grasp_offset_y", 0.0))
            grasp_offset_z = float(rospy.get_param("~direct_grasp_offset_z", 0.02))

            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = "base_link"
            grasp_pose.header.stamp = rospy.Time.now()
            grasp_pose.pose.position.x = float(target_point.point.x) + grasp_offset_x
            grasp_pose.pose.position.y = float(target_point.point.y) + grasp_offset_y
            grasp_pose.pose.position.z = float(target_point.point.z) + grasp_offset_z
            grasp_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

            blackboard.set("target_grasp_pose", grasp_pose)
            blackboard.set("target_grasp_mode", "direct_yolo_point")
            rospy.loginfo(
                "[Brain] Direct grasp pose from YOLO for %s: pos=(%.3f, %.3f, %.3f) offsets=(%.3f, %.3f, %.3f)",
                target_label,
                grasp_pose.pose.position.x,
                grasp_pose.pose.position.y,
                grasp_pose.pose.position.z,
                grasp_offset_x,
                grasp_offset_y,
                grasp_offset_z,
            )
            self.request_sent = True
            return Status.SUCCESS

        return Status.SUCCESS


class ExecutePickAndPlaceBehavior(py_trees.behaviour.Behaviour):
    """阶段4：非阻塞执行抓取与放置"""
    def __init__(self, name="ExecutePickAndPlace"):
        super().__init__(name)
        self.client = None
        self.gripper_cmd_pub = rospy.Publisher('/gripper/command', String, queue_size=1)
        self.goal_sent = False
        self.mock_done = False
        self.last_feedback_message = None
        
    def setup(self, timeout=None):
        if is_test_mode():
            return True
        self.client = actionlib.SimpleActionClient('/path_planning/plan_execute_pose', PlanExecutePoseAction)
        return True

    def _feedback_callback(self, feedback):
        if feedback.message == self.last_feedback_message:
            return
        self.last_feedback_message = feedback.message
        stage_name = "planning" if feedback.stage == PlanExecutePoseFeedback.PLANNING else "executing"
        rospy.loginfo("[Brain] Path planning feedback (%s): %s", stage_name, feedback.message)

    def initialise(self):
        self.goal_sent = False
        self.mock_done = False
        self.last_feedback_message = None

    def update(self):
        if is_test_mode():
            if not self.mock_done:
                rospy.loginfo("[Brain] [Test] Mock pick-and-place complete")
                self.mock_done = True
            return Status.SUCCESS

        # 初始化检查
        if not self.client.wait_for_server(rospy.Duration(0.1)):
            rospy.loginfo_throttle(2.0, "[Brain] Waiting for path_planning action server...")
            return Status.RUNNING

        # 1. 如果还没发指令，发送指令
        if not self.goal_sent:
            blackboard = py_trees.blackboard.Blackboard()
            target_pose = blackboard.get("target_grasp_pose")

            if target_pose is None:
                rospy.loginfo_throttle(2.0, "[Brain] Waiting for target_grasp_pose...")
                return Status.RUNNING
            if not isinstance(target_pose, PoseStamped):
                rospy.logerr("[Brain] target_grasp_pose is not a PoseStamped: %r", type(target_pose))
                return Status.FAILURE

            goal = PlanExecutePoseGoal()
            goal.target_pose = target_pose
            goal.position_only = rospy.get_param("~path_planning_position_only", True)

            if rospy.get_param("~open_gripper_before_pick", True):
                self.gripper_cmd_pub.publish(String(data="release"))
                rospy.loginfo("[Brain] Sent gripper release command before approach")

            rospy.loginfo(
                "[Brain] Sending target pose to path_planning action... pos=(%.3f, %.3f, %.3f) position_only=%s",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                str(goal.position_only),
            )
            self.client.send_goal(goal, feedback_cb=self._feedback_callback)
            self.goal_sent = True
            return Status.RUNNING
            
        # 2. 如果已经发了指令，检查状态（这解决了你之前代码的阻塞问题！）
        state = self.client.get_state()
        
        if state in [action_msgs.GoalStatus.ACTIVE, action_msgs.GoalStatus.PENDING]:
            return Status.RUNNING # 机械臂正在动，树继续 Tick，不卡死！
            
        elif state == action_msgs.GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            self.goal_sent = False # 重置状态，为下一次抓取做准备
            if result is not None and result.success:
                if rospy.get_param("~close_gripper_after_reach", True):
                    self.gripper_cmd_pub.publish(String(data="grasp"))
                    rospy.loginfo("[Brain] Sent gripper grasp command after reaching target pose")
                blackboard = get_blackboard()
                blackboard.set("executed_trajectory", result.trajectory)
                rospy.loginfo("[Brain] Pick and Place SUCCESS! %s", result.message)
                return Status.SUCCESS
            failure_message = result.message if result is not None else "path_planning action returned no result"
            rospy.logerr("[Brain] Path planning action failed after SUCCEEDED state: %s", failure_message)
            return Status.FAILURE
            
        else:
            result = self.client.get_result()
            failure_message = result.message if result is not None else f"path_planning action failed with state {state}"
            if result is not None and result.status == PlanExecutePoseResult.PREEMPTED:
                rospy.logwarn("[Brain] Path planning action preempted: %s", failure_message)
            else:
                rospy.logerr("[Brain] Path planning action failed: %s", failure_message)
            self.goal_sent = False
            return Status.FAILURE


def create_behavior_tree():
    """
    修改为循环抓取结构
    """
    # 主工作流序列（带记忆，成功后重置）
    main_sequence = py_trees.composites.Sequence(
        name="MainTaskSequence",
        memory=True, # 记住执行到了哪一步
        children=[
            MoveToOverviewBehavior(),
            EvaluateTargetsBehavior(),
            RequestGraspPoseBehavior(),
            ExecutePickAndPlaceBehavior()
        ]
    )
    

    # 根节点：main() 的 tick 循环会不断重试，无需 Loop 装饰器
    root = py_trees.composites.Selector(
        name="TaskOrRecovery",
        children=[
            main_sequence,
            # RecoveryBehavior() # 如果 MainTask 失败，走恢复逻辑
        ]
    )
    return root


# ... main() 函数保持不变 ...

def main():
    rospy.init_node('robocup_brain', anonymous=False)
    rospy.loginfo("=" * 50)
    rospy.loginfo("RoboCup Brain Node Starting")
    rospy.loginfo("=" * 50)
    rospy.loginfo("[Brain] test_mode=%s", is_test_mode())
    
    # 创建行为树
    root = create_behavior_tree()
    
    # 创建行为树管理器
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    if hasattr(behaviour_tree, "setup"):
        behaviour_tree.setup(timeout=15)
    
    # 设置更新频率
    rate = rospy.Rate(10)  # 10 Hz
    
    rospy.loginfo("[Brain] Behavior Tree initialized. Starting main loop...")
    
    try:
        while not rospy.is_shutdown():
            behaviour_tree.tick()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[Brain] Shutting down...")

#11
if __name__ == '__main__':
    main()
