#!/usr/bin/env python3
"""
RoboCup Brain Node - 基于行为树的决策系统
Author: Suhang Xia
Architecture: py_trees_ros
"""

import rospy
import py_trees
import py_trees_ros
from py_trees.common import Status

from common_msgs.msg import DetectedObject, GraspCandidate
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
import actionlib


class SearchBehavior(py_trees.behaviour.Behaviour):
    """搜索行为 - 控制相机或机械臂扫描环境"""
    
    def __init__(self, name="Search"):
        super(SearchBehavior, self).__init__(name)
        self.search_complete = False
        
    def setup(self):
        rospy.loginfo("[Brain] SearchBehavior: Setup")
        return True
        
    def update(self):
        rospy.loginfo("[Brain] SearchBehavior: Executing search pattern")
        # TODO: 实现搜索逻辑（例如：发送关节目标或视觉扫描指令）
        self.search_complete = True
        
        if self.search_complete:
            return Status.SUCCESS
        return Status.RUNNING


class DetectBehavior(py_trees.behaviour.Behaviour):
    """检测行为 - 等待感知模块发布检测结果"""
    
    def __init__(self, name="Detect"):
        super(DetectBehavior, self).__init__(name)
        self.detected_objects = []
        self.sub = None
        
    def setup(self):
        rospy.loginfo("[Brain] DetectBehavior: Setup")
        self.sub = rospy.Subscriber(
            "/perception/detected_objects",
            DetectedObject,
            self._detection_callback
        )
        return True
        
    def _detection_callback(self, msg):
        """接收检测到的物体"""
        self.detected_objects.append(msg)
        rospy.loginfo(f"[Brain] Detected: {msg.label} (score: {msg.score:.2f})")
        
    def update(self):
        if len(self.detected_objects) > 0:
            # 将检测到的物体存储到黑板
            self.feedback_message = f"Found {len(self.detected_objects)} objects"
            return Status.SUCCESS
        return Status.RUNNING
        
    def terminate(self, new_status):
        if self.sub:
            self.sub.unregister()


class PlanGraspBehavior(py_trees.behaviour.Behaviour):
    """规划抓取行为 - 等待抓取候选姿态"""
    
    def __init__(self, name="PlanGrasp"):
        super(PlanGraspBehavior, self).__init__(name)
        self.grasp_candidates = []
        self.sub = None
        
    def setup(self):
        rospy.loginfo("[Brain] PlanGraspBehavior: Setup")
        self.sub = rospy.Subscriber(
            "/perception/grasp_candidates",
            GraspCandidate,
            self._grasp_callback
        )
        return True
        
    def _grasp_callback(self, msg):
        """接收抓取候选姿态"""
        self.grasp_candidates.append(msg)
        rospy.loginfo(f"[Brain] GraspCandidate received (quality: {msg.quality:.2f})")
        
    def update(self):
        if len(self.grasp_candidates) > 0:
            # 选择最佳抓取姿态
            best_grasp = max(self.grasp_candidates, key=lambda x: x.quality)
            self.feedback_message = f"Best grasp quality: {best_grasp.quality:.2f}"
            # 存储到黑板供执行行为使用
            self.blackboard = py_trees.blackboard.Blackboard()
            self.blackboard.set("target_grasp", best_grasp)
            return Status.SUCCESS
        return Status.RUNNING
        
    def terminate(self, new_status):
        if self.sub:
            self.sub.unregister()


class ExecuteGraspBehavior(py_trees.behaviour.Behaviour):
    """执行抓取行为 - 通过 MoveIt 控制机械臂"""
    
    def __init__(self, name="ExecuteGrasp"):
        super(ExecuteGraspBehavior, self).__init__(name)
        self.move_group_client = None
        
    def setup(self):
        rospy.loginfo("[Brain] ExecuteGraspBehavior: Setup")
        # 连接到 VM 中的 MoveIt /move_group action server
        ros_master_uri = rospy.get_param("/ros_master_uri", "http://192.168.56.101:11311")
        rospy.loginfo(f"[Brain] Connecting to MoveIt at: {ros_master_uri}")
        
        self.move_group_client = actionlib.SimpleActionClient(
            '/move_group',
            MoveGroupAction
        )
        
        # 等待连接（非阻塞，使用 wait_for_server）
        rospy.loginfo("[Brain] Waiting for /move_group action server...")
        return True
        
    def update(self):
        if not self.move_group_client.wait_for_server(timeout=rospy.Duration(0.5)):
            rospy.logwarn("[Brain] /move_group not available yet")
            return Status.RUNNING
            
        # 从黑板获取目标抓取姿态
        blackboard = py_trees.blackboard.Blackboard()
        target_grasp = blackboard.get("target_grasp")
        
        if target_grasp is None:
            rospy.logerr("[Brain] No target grasp in blackboard!")
            return Status.FAILURE
            
        # 构造 MoveIt 目标
        goal = MoveGroupGoal()
        # TODO: 填充 goal 的详细内容（planning_options, request 等）
        rospy.loginfo(f"[Brain] Sending grasp goal to MoveIt: {target_grasp.pose.pose.position}")
        
        self.move_group_client.send_goal(goal)
        self.move_group_client.wait_for_result()
        
        result = self.move_group_client.get_result()
        if result:
            rospy.loginfo("[Brain] MoveIt execution SUCCESS")
            return Status.SUCCESS
        else:
            rospy.logerr("[Brain] MoveIt execution FAILED")
            return Status.FAILURE


class RecoveryBehavior(py_trees.behaviour.Behaviour):
    """恢复行为 - 失败时的应急处理"""
    
    def __init__(self, name="Recovery"):
        super(RecoveryBehavior, self).__init__(name)
        
    def update(self):
        rospy.logwarn("[Brain] RecoveryBehavior: Attempting recovery...")
        # TODO: 实现恢复逻辑（例如：返回初始姿态、清空缓存等）
        return Status.SUCCESS


def create_behavior_tree():
    """
    构建行为树结构：
    Selector (Recovery Branch)
      -> Sequence (Search -> Detect -> Plan -> Execute)
    """
    
    # 创建主序列：搜索 -> 检测 -> 规划 -> 执行
    main_sequence = py_trees.composites.Sequence(
        name="MainSequence",
        children=[
            SearchBehavior(),
            DetectBehavior(),
            PlanGraspBehavior(),
            ExecuteGraspBehavior()
        ]
    )
    
    # 创建恢复分支
    recovery_branch = RecoveryBehavior()
    
    # 创建选择器：尝试主序列，失败则执行恢复
    root = py_trees.composites.Selector(
        name="RootSelector",
        children=[main_sequence, recovery_branch]
    )
    
    return root


def main():
    rospy.init_node('robocup_brain', anonymous=False)
    rospy.loginfo("=" * 50)
    rospy.loginfo("RoboCup Brain Node Starting")
    rospy.loginfo("=" * 50)
    
    # 创建行为树
    root = create_behavior_tree()
    
    # 创建行为树管理器
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    
    # 设置更新频率
    rate = rospy.Rate(10)  # 10 Hz
    
    rospy.loginfo("[Brain] Behavior Tree initialized. Starting main loop...")
    
    try:
        while not rospy.is_shutdown():
            behaviour_tree.tick()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[Brain] Shutting down...")


if __name__ == '__main__':
    main()
