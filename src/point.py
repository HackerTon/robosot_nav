#!/usr/bin/env python2.7

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point32

# CONSTANT
# RED
MIN_RED = np.asarray([0, 25, 0])
MAX_RED = np.asarray([10, 255, 255])


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
            'camera/image', Image, self.callback)
        self.image_pub = rospy.Publisher(
            'after/image/compressed', CompressedImage, queue_size=1)

        self.point_pub = rospy.Publisher(
            '/pointer/point1', Point32, queue_size=1)

        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            # cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        src = cv_image

        rows = src.shape[0]

        blur = 5
        dp = 1
        minDist = rows / 8
        param1 = 77
        param2 = 19
        minRadius = 8
        maxRadius = 49

        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, blur)

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
                    is_red = False

                if is_red:
                    cv2.circle(src, center, radius, (255, 0, 255), 3)
        try:
            src = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
            org_image = self.bridge.cv2_to_compressed_imgmsg(src)
            self.image_pub.publish(org_image)
        except CvBridgeError as e:
            print(e.message)

    @staticmethod
    def colorsegmentation(img, center):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)

        hsv = img_hsv[center[0], center[1]]
        print(hsv)

        if 0 < hsv[0] < 180 and 100 < hsv[1] < 255 and 0 < hsv[2] < 255:
            return True
        else:
            return False


if __name__ == "__main__":
    rospy.init_node('red_camera', anonymous=True)

    imageconverter = cv_converter()

    print('red_camera running!')
    while not rospy.is_shutdown():
        pass
    print('red_camera stop!')