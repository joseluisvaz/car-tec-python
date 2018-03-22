#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from img_pipeline import img_pipeline
from line_detector import LineDetector


class LineDetectorImpl:

    def __init__(self):
        self.bridge = CvBridge()
        self.line_detection = LineDetector()
        self.image_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                          Image,
                                          self.callback,
                                          queue_size=rospy.get_param("~subs_queue_size"),
                                          buff_size=rospy.get_param("~buff_size"))

        #self.image_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
        #                                 Image,
        #                                 queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, data):

        cv_image = self._to_cv_image(data)

        _, white_lines = self.line_detection.detect(cv_image, "white")
        _, yellow_lines = self.line_detection.detect(cv_image, "yellow")
        _, red_lines = self.line_detection.detect(cv_image, "red")

        stack = np.hstack((yellow_lines, white_lines, red_lines))
        cv2.imshow("normal", cv_image)
        cv2.imshow("vstack", stack)
        cv2.waitKey(3)
        #img_cropped = img_pipeline(cv_image)

        #ros_img = self._to_ros_image(img_cropped)
        #self.image_pub.publish(ros_img)

    def _to_cv_image(self, data):
        try:
            return self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def _to_ros_image(self, data):
        try:
            return self.bridge.cv2_to_imgmsg(data, "mono8")
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('line_detector', anonymous=True)
    LineDetectorImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
