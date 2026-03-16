#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
夹爪力控制节点 - 使用 GripperCommand Action 通信
- 提供 grasp / release 命令（Topic + Service）
- 力控制夹取，根据 result.stalled 判断夹取成功
- 发布 /gripper/grasp_result (Bool)
"""

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

# Robotiq 85: arm_gazebo 模型关节方向与标准相反，故对调
# 0=张开, 0.79=闭合（适配 arm_gazebo URDF）
GRIPPER_CLOSED = 0.79
GRIPPER_OPEN = 0.0
DEFAULT_MAX_EFFORT = 100.0


class GripperControlNode:
    def __init__(self):
        rospy.init_node('gripper_control', anonymous=False)

        self.action_name = rospy.get_param(
            '~gripper_action', '/gripper_controller/gripper_cmd'
        )
        self.max_effort = rospy.get_param('~max_effort', DEFAULT_MAX_EFFORT)

        self.client = actionlib.SimpleActionClient(
            self.action_name, GripperCommandAction
        )
        rospy.loginfo("[GripperControl] Waiting for action %s...", self.action_name)
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("[GripperControl] Action server not available!")
            raise rospy.ROSException("Gripper action server timeout")
        rospy.loginfo("[GripperControl] Connected to gripper action server.")

        self.result_pub = rospy.Publisher(
            '/gripper/grasp_result', Bool, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber(
            '/gripper/command', String, self._cmd_callback, queue_size=1
        )
        self.grasp_srv = rospy.Service(
            '/gripper/grasp', Trigger, self._grasp_service
        )

    def _cmd_callback(self, msg):
        cmd = msg.data.lower().strip()
        if cmd in ('grasp', 'close'):
            self._do_grasp()
        elif cmd in ('release', 'open'):
            self._do_release()

    def _grasp_service(self, req):
        success = self._do_grasp()
        return TriggerResponse(
            success=success,
            message="grasp_success" if success else "grasp_empty"
        )

    def _do_grasp(self):
        """执行力控制夹取，返回 True 表示夹到物体，False 表示空夹"""
        goal = GripperCommandGoal()
        goal.command.position = GRIPPER_CLOSED
        goal.command.max_effort = self.max_effort

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        # stalled=True: 夹爪遇到阻力停止 -> 夹取成功
        # reached_goal=True 且 stalled=False: 空闭合 -> 夹取失败
        grasp_success = result.stalled
        self.result_pub.publish(Bool(data=grasp_success))

        rospy.loginfo(
            "[GripperControl] Grasp: %s (stalled=%s, reached_goal=%s)",
            "SUCCESS" if grasp_success else "EMPTY",
            result.stalled,
            result.reached_goal
        )
        return grasp_success

    def _do_release(self):
        """张开夹爪"""
        goal = GripperCommandGoal()
        goal.command.position = GRIPPER_OPEN
        goal.command.max_effort = self.max_effort

        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("[GripperControl] Release completed.")


def main():
    try:
        node = GripperControlNode()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
