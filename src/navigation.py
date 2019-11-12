#!/usr/bin/env python

import rospy
import actionlib
import logging
import sys

from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class navigation:
    def __init__(self):
        self.base_move = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)

        self.base_move.wait_for_server(rospy.Duration(10))

    def move_goal(self, pos=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.010, 1)):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(
            Point(pos[0], pos[1], pos[2]), Quaternion(0, 0, 0.010, 1))

        self.base_move.send_goal(goal)

        success = self.base_move.wait_for_result(rospy.Duration(60))
        state = self.base_move.get_state()

        logging.info('{}, {}'.format(success, state))


def main(args):
    rospy.init_node('navigation_node', anonymous=True)
    navigator = navigation()

    running = True

    while running:
        try:
            print('Input your desired coordinate')
            coor = raw_input()
            coor = coor.split(' ')
            coor = [float(coor[0]), float(coor[1]), float(coor[2])]
            navigator.move_goal(coor, (-0.001, 0.0, -0.016, 1.0))
        except KeyboardInterrupt:
            running = False


if __name__ == "__main__":
    main(sys.argv)    
