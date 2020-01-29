#!/usr/bin/env python

import rospy
import actionlib
import logging
import sys

from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


def goal(pos, quater):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(
        pos[0], pos[1], p[2], Quaternion(quater[0], quater[1], quater[2], quater[3]))

    return goal


class navigation:
    def __init__(self):
        self.base_move = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)

        self.base_move.wait_for_server()

    def move_goal(self, pos=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.010, 1)):
        goal(pos, quat)

        self.base_move.send_goal(goal)

        self.base_move.wait_for_result()

        success = self.base_move.wait_for_result()
        state = self.base_move.get_state()

        return success, state


def main(args):
    rospy.init_node('navigation_node', anonymous=True)
    navigator = navigation()

    stack = []

    while not rospy.is_shutdown():
        


        if detected:
            stack.append()


if __name__ == "__main__":
    main(sys.argv)
