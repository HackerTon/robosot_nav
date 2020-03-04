#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point32

def compute_contours(image, org_image, range_min, range_max):
    binary_image = cv2.inRange(image, range_min, range_max)

    _, contours, _ = cv2.findContours(
        binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sorted_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 120]

    circles = []

    for contour in sorted_contours:
        (x, y), r = cv2.minEnclosingCircle(contour)
        circles.append([(int(x), int(y)), int(r)])

    return circles


class cv_converter:
    def __init__(self):
        self.image_sub = rospy.Subscriber(
            'camera/image/compressed', CompressedImage, self.callback)
        self.image_pub = rospy.Publisher(
            'after/image/compressed', CompressedImage, queue_size=1)

        self.point_pub = rospy.Publisher(
            '/pointer/point1', Point32, queue_size=1)

        self.bridge = CvBridge()
        rospy.loginfo('Detection starts')

    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        src = cv_image

        rows = src.shape[0]

        blur = 5
        dp = 1
        minDist = rows / 8
        param1 = 100
        param2 = 14
        minRadius = 0
        maxRadius = 40

        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)

        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp, minDist,
                                   param1=param1, param2=param2,
                                   minRadius=minRadius, maxRadius=maxRadius)

        if circles is not None:
            circles = np.around(circles).astype(np.int)
            for i in circles[0, :]:
                center = (i[0], i[1])
                radius = i[2]

                try:
                    is_red = self.colorsegmentation(src, center)
                except IndexError as e:
                    is_red = True

                if is_red:
                    cv2.circle(src, center, radius, (255, 0, 255), 3)
                    point = Point32(center[0], center[1], 0.0)
                    self.point_pub.publish(point)
        else:
            self.point_pub.publish(Point32(-1.0, -1.0, -1.0))
        try:
            org_image = self.bridge.cv2_to_compressed_imgmsg(src)
            self.image_pub.publish(org_image)
        except CvBridgeError as e:
            rospy.logerr(e.msg)

    @staticmethod
    def colorsegmentation(img, center):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
        hsv = img_hsv[center[1], center[0]]

        if 0 <= hsv[0] <= 10 and 100 <= hsv[1] <= 255 and 100 <= hsv[2] <= 255:
            return True
        else:
            return False


if __name__ == "__main__":
    rospy.init_node('red_camera', anonymous=True)

    imageconverter = cv_converter()

    rospy.spin()

