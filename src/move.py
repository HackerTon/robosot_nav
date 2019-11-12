#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def move():
    running = True
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    while running:
        stop_twist = Twist()
        stop_twist.angular.z = 0.0
        stop_twist.linear.x = 0.05
        cmd_pub.publish(stop_twist)
        print('moving')
        rospy.sleep(5)
        stop_twist = Twist()
        stop_twist.angular.z = 0.0
        stop_twist.linear.x = 0.0
        cmd_pub.publish(stop_twist)
        print('stop')
        rospy.sleep(5)


if __name__ == "__main__":
    rospy.init_node('movement', anonymous=True)
    rospy.spin()
    move()