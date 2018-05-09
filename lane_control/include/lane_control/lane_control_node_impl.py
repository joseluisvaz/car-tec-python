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
from utils.linear_regression import fit_line
from utils.lines_to_control import get_left_segments
from utils.lines_to_control import get_right_segments
from utils.draw_functions import draw_linear_regression

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)

MOVING_MEAN_SIZE = 250
LIN_BUFFER = 15
LINE_THRESHOLD = 35


class LaneControlImpl(object):
    def __init__(self):
        self.control_list_x = [480] * MOVING_MEAN_SIZE
        self.control_list_y = [0] * MOVING_MEAN_SIZE
        self.dataset_list = []
        self.centers_array = None

        self.MESSAGE_COUNTER = 0
        self.bridge = CvBridge()

        self.sample_img_message = rospy.wait_for_message(rospy.get_param("~image_topic_1"), Image)
        self.sample_cvimg = self.bridge.imgmsg_to_cv2(self.sample_img_message, "bgr8")
        self.sample_img_shape = self.sample_cvimg.shape

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

        # Draws activated pixels for each segment in image
        activated_pixels = np.zeros(self.sample_img_shape, dtype=np.uint8)

        for x1, x2, y1, y2 in get_left_segments(segment_list):
            cv2.line(activated_pixels, (int(x1), int(y1)), (int(x2), int(y2)), COLOR_RED, 2)
        for x1, x2, y1, y2 in get_right_segments(segment_list):
            cv2.line(activated_pixels, (int(x1), int(y1)), (int(x2), int(y2)), COLOR_GREEN, 2)

        index_left_segments = np.argwhere(activated_pixels[:, :, 2] == 255)
        index_right_segments = np.argwhere(activated_pixels[:, :, 1] == 255)

        # FIXME: PROVISIONAL STUFF TO SAVE PAST STATES

        if self.MESSAGE_COUNTER < LIN_BUFFER:
            self.MESSAGE_COUNTER += 1
            self.dataset_list.append(index_left_segments)
            self.centers_array = np.concatenate(self.dataset_list, axis=0)
        elif control_sample.count < LINE_THRESHOLD:
            # HOLDS RESULT OF LAST LINEAR REGRESSION
            self.dataset_list.append(index_left_segments)
            self.dataset_list.pop(0)
            self.centers_array = np.concatenate(self.dataset_list, axis=0)
        else:
            self.dataset_list.append(index_left_segments)
            self.dataset_list.pop(0)
            self.centers_array = np.concatenate(self.dataset_list, axis=0)

        # FIXME: FIT LINE TO CENTERS ARRAY OR INDEX_MATRIX
        # FIXME: WHAT HAPPENS WHEN INPUTS ARE EMPTY, NO LINEAR REGRESSION TO CALCULATE
        lane_left = None
        lane_right = None

        if index_left_segments is not None:
            lane_left = fit_line(index_left_segments)
        if index_right_segments is not None:
            lane_right = fit_line(index_right_segments)

        if rospy.get_param("~verbose") == 1 and lane_left is not None and lane_right is not None:
            draw_linear_regression(activated_pixels,
                                   lane_left.slope,
                                   lane_right.slope, 
                                   lane_left.intercept, 
                                   lane_right.intercept)

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
