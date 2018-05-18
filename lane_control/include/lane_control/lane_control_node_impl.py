#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from car_tec_msgs.msg import SegmentList
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

from utils.lines_to_control import line_means
from double_line_tracker import DoubleLineTracker
from single_line_tracker import SingleLineTracker

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)

MOVING_MEAN_SIZE = 250
LIN_BUFFER = 15
LINE_THRESHOLD = 35


class LaneControlImpl(object):
    def __init__(self):

        self.control_list_x = [480] * MOVING_MEAN_SIZE
        self.control_list_y = [0] * MOVING_MEAN_SIZE

        self.bridge = CvBridge()

        # Wait for sample message and get imagen size
        self.sample_img_message = rospy.wait_for_message(rospy.get_param("~image_topic_1"), Image)
        self.sample_cvimg = self.bridge.imgmsg_to_cv2(self.sample_img_message, "bgr8")
        self.sample_img_shape = self.sample_cvimg.shape

        # Set type of line tracker, single or double (normal traffic lane)
        if rospy.get_param("~mode") == 0:
            self.tracker = DoubleLineTracker(img_shape=self.sample_img_shape, lin_buffer=LIN_BUFFER, line_threshold=LINE_THRESHOLD)
        elif rospy.get_param("~mode") == 1:
            self.tracker = SingleLineTracker(img_shape=self.sample_img_shape, lin_buffer=LIN_BUFFER, line_threshold=LINE_THRESHOLD)

        self.img_size_x = self.sample_img_shape[1]
        self.img_size_y = self.sample_img_shape[0]
        self.reference_pixel_x = int(self.img_size_x/2)
        self.reference_pixel_y = int(self.img_size_y/2)

        self.control_x = None
        self.control_y = None

        self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                            SegmentList,
                                            self.callback,
                                            queue_size=rospy.get_param("~subs_queue_size"),
                                            buff_size=rospy.get_param("~buff_size"))

        self.control_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
                                           Float32MultiArray,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, segment_list):

        # control.u1 control.u2
        control_sample = line_means(segment_list)

        self.tracker.track_segments(seg_list=segment_list, control_sample=control_sample)

        # FIXME: CONTROL STAGE, REFACTOR THIS TO FUNCTION
        self.control_list_x.append(control_sample.mean_x)
        self.control_list_y.append(control_sample.mean_y)
        self.control_list_x.pop(0)
        self.control_list_y.pop(0)

        control_x = sum(self.control_list_x) / MOVING_MEAN_SIZE
        control_y = sum(self.control_list_y) / MOVING_MEAN_SIZE

        error_x = self.reference_pixel_x - control_x
        error_y = self.reference_pixel_y - control_y

        self.control_x = control_x
        self.control_y = control_y

        # GEAR RATIO
        Kp = 1
        array = Float32MultiArray()
        array.data = [Kp*error_x, Kp*error_y]

        self.control_pub.publish(array)


def main():
    rospy.init_node("lane_control", anonymous=True)
    obj = LaneControlImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
