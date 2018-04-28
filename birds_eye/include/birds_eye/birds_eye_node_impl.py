#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from car_tec_msgs.msg import SegmentList
from birds_eye.utils.warper import Warper


class BirdsEyeImpl(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.warper = Warper()
        self.segment_list = None
        self.configured = False
        self.image_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic_1"),
                                          Image,
                                          self.callback,
                                          queue_size=rospy.get_param("~subs_queue_size"),
                                          buff_size=rospy.get_param("~buff_size"))

        #self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic_2"),
        #                                    SegmentList,
        #                                    self.callback,
        #                                    queue_size=rospy.get_param("~subs_queue_size"),
        #                                    buff_size=rospy.get_param("~buff_size"))

        self.image_pub = rospy.Publisher(rospy.get_param("~publisher_topic_1"),
                                         Image,
                                         queue_size=rospy.get_param("~pubs_queue_size"))

        self.segment_pub = rospy.Publisher(rospy.get_param("~publisher_topic_2"),
                                           SegmentList,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, img_msg):

        img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        if not self.configured:
            self.warper.set_image_size(img)
            self.warper.set_homography(img)
            self.configured = True

        warped_img = self.warper.warp_image(img)

        ros_image = self.bridge.cv2_to_imgmsg(warped_img, "bgr8")

        self.image_pub.publish(ros_image)


def main():
    rospy.init_node("birds_eye", anonymous=True)
    obj = BirdsEyeImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
