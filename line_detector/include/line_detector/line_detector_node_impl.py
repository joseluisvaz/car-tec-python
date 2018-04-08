#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from line_detector_hough import LineDetectorHough
from line_detector_plot import draw_lines

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)


class LineDetectorImpl:

    def __init__(self):
        self.bridge = CvBridge()
        self.detector = LineDetectorHough()
        self.image_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                          Image,
                                          self.callback,
                                          queue_size=rospy.get_param("~subs_queue_size"),
                                          buff_size=rospy.get_param("~buff_size"))

        self.image_pub = rospy.Publisher(rospy.get_param("~publisher_topic_1"),
                                         Image,
                                         queue_size=rospy.get_param("~pubs_queue_size"))

        self.edges_pub = rospy.Publisher(rospy.get_param("~publisher_topic_2"),
                                         Image,
                                         queue_size=rospy.get_param("~pubs_queue_size"))

        self.white_back_pub = rospy.Publisher(rospy.get_param("~publisher_topic_3"),
                                              Image,
                                              queue_size=rospy.get_param("~pubs_queue_size"))

        self.yellow_back_pub = rospy.Publisher(rospy.get_param("~publisher_topic_4"),
                                               Image,
                                               queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, data):

        cv_image = self._to_cv_image(data)

        self.detector.set_image(cv_image)

        white_info = self.detector.detect("white")
        yellow_info = self.detector.detect("yellow")

        draw_lines(cv_image, white_info.lines, COLOR_RED)
        draw_lines(cv_image, yellow_info.lines, COLOR_GREEN)

        ros_img = self._to_ros_image_color(cv_image)
        edges_img = self._to_ros_image_bw(self.detector.edges)
        white_back_img = self._to_ros_image_bw(white_info.area)
        yellow_back_img = self._to_ros_image_bw(yellow_info.area)

        self.image_pub.publish(ros_img)
        self.edges_pub.publish(edges_img)
        self.white_back_pub.publish(white_back_img)
        self.yellow_back_pub.publish(yellow_back_img)

    def _to_cv_image(self, data):
        try:
            return self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def _to_ros_image_color(self, data):
        try:
            return self.bridge.cv2_to_imgmsg(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def _to_ros_image_bw(self, data):
        try:
            return self.bridge.cv2_to_imgmsg(data, "mono8")
        except CvBridgeError as e:
            print(e)


def main():
    rospy.init_node('line_detector', anonymous=True)
    LineDetectorImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
