#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BirdsEyeImpl(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.segment_list = None
        self.image_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                          Image,
                                          self.callback,
                                          queue_size=rospy.get_param("~subs_queue_size"),
                                          buff_size=rospy.get_param("~buff_size"))

        self.image_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
                                         Image,
                                         queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(ros_image)


def main():
    rospy.init_node("birds_eye", anonymous=True)
    obj = BirdsEyeImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
