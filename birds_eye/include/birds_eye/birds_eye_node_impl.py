#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from car_tec_msgs.msg import SegmentList
from car_tec_msgs.msg import Segment
#from line_detector.line_detector_plot import draw_lines
from birds_eye.utils.warper import Warper

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)


class BirdsEyeImpl(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.warper = Warper()
        self.segment_list = None
        self.configured = False
        self.image_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic_1"),
                                          Image,
                                          self.callback_image,
                                          queue_size=rospy.get_param("~subs_queue_size"),
                                          buff_size=rospy.get_param("~buff_size"))

        self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic_2"),
                                            SegmentList,
                                            self.callback_segment,
                                            queue_size=rospy.get_param("~subs_queue_size"),
                                            buff_size=rospy.get_param("~buff_size"))

        self.image_pub = rospy.Publisher(rospy.get_param("~publisher_topic_1"),
                                         Image,
                                         queue_size=rospy.get_param("~pubs_queue_size"))

        self.image2_pub = rospy.Publisher(rospy.get_param("~publisher_topic_2"),
                                          Image,
                                          queue_size=rospy.get_param("~pubs_queue_size"))

        self.segment_pub = rospy.Publisher(rospy.get_param("~publisher_topic_3"),
                                           SegmentList,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def callback_image(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        if not self.configured:
            self.warper.set_image_size(img)
            self.warper.set_homography(img)
            self.configured = True

        self.warper.set_image(img)           # Set image for use in callback_segment
        warped_img = self.warper.warp_image(img)
        print(self.warper.H)
        ros_image = self.bridge.cv2_to_imgmsg(warped_img, "bgr8")
        self.image_pub.publish(ros_image)

    # TODO: VECTORIZE THIS CODE
    def warp_vector(self, vector):
        """
        Vector [2x1]
        :param vector:
        :return: transformed vector
        """
        vector = np.concatenate([vector, np.array([[1]])], axis=0)
        proy = np.dot(self.warper.H, vector)
        proy[0] = proy[0] / proy[2]
        proy[1] = proy[1] / proy[2]
        proy[2] = 0
        return proy[:-1]

    def callback_segment(self, segment_list_msg):
        if not self.configured:
            return None

        new_img = self.warper.warp_image(self.warper.img)

        # TODO: Big refactor and send only the segmens for better speed
        for item in segment_list_msg.segments:
            vec1 = np.array([[int(item.pixels_normalized[0].x)],
                             [int(item.pixels_normalized[0].y)]])
            vec2 = np.array([[int(item.pixels_normalized[1].x)],
                             [int(item.pixels_normalized[1].y)]])

            stacked = np.vstack((self.warp_vector(vec1), self.warp_vector(vec2)))

            x1 = stacked[0]
            y1 = stacked[1]
            x2 = stacked[2]
            y2 = stacked[3]

            cv2.line(new_img, (x1, y1), (x2, y2), COLOR_RED, 2)

        ros_image = self.bridge.cv2_to_imgmsg(new_img, "bgr8")
        self.image2_pub.publish(ros_image)

        # TODO: Big refactor and send only the segmens for better speed
        for item in segment_list_msg.segments:
            vec1 = np.array([[int(item.pixels_normalized[0].x)],
                             [int(item.pixels_normalized[0].y)]])
            vec2 = np.array([[int(item.pixels_normalized[1].x)],
                             [int(item.pixels_normalized[1].y)]])

            stacked = np.vstack((self.warp_vector(vec1), self.warp_vector(vec2)))

            item.pixels_normalized[0].x = stacked[0]
            item.pixels_normalized[0].y = stacked[1]
            item.pixels_normalized[1].x = stacked[2]
            item.pixels_normalized[1].y = stacked[3]

        self.segment_pub.publish(segment_list_msg)



def draw_lines(bgr, lines, paint, p1_color=(0, 255, 0), p2_color=(0, 0, 255)):
        for x1, y1, x2, y2 in lines:
            cv2.line(bgr, (x1, y1), (x2, y2), paint, 2)
            if p1_color is not None:
                cv2.circle(bgr, (x1, y1), 2, p1_color)
            if p2_color is not None:
                cv2.circle(bgr, (x2, y2), 2, p2_color)


def main():
    rospy.init_node("birds_eye", anonymous=True)
    obj = BirdsEyeImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
