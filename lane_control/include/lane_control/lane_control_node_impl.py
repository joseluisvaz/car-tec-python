#!/usr/bin/env python

from __future__ import print_function

import rospy
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

MOVING_MEAN_SIZE = 50
LIN_BUFFER = 15
LINE_THRESHOLD = 35
MAX_GRAD = 23


class LaneControlImpl(object):
    def __init__(self):

        self.control_list_steer = [0] * MOVING_MEAN_SIZE
        self.control_list_brake = [0] * MOVING_MEAN_SIZE
        self.control_list_acc = [0] * MOVING_MEAN_SIZE

        self.bridge = CvBridge()

        # Wait for sample message and get imagen size
        self.sample_img_message = rospy.wait_for_message(rospy.get_param("~image_topic_2"), Image)
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

        lane = self.tracker.track_segments(seg_list=segment_list, control_sample=control_sample)

        reference_point = None
        if lane is not None:
            slice_y = 300
            reference_point = (int(slice_y*lane.slope + lane.intercept), slice_y)

        if reference_point is not None:
            delta_x = self.reference_pixel_x - reference_point[0]
            delta_y = reference_point[1]
            rho = np.sqrt(delta_x**2 + delta_y**2)
            theta = np.arctan2(delta_x, delta_y)

            ctrl_steer = np.rad2deg(theta)
            ctrl_brake = rho
            # TODO: Control acceleration
            ctrl_acc = 0
        else:
            ctrl_steer = 0
            ctrl_brake = 0
            ctrl_acc = 0

        # ctrl_x = control_sample.mean_x
        # ctrl_y = control_sample.mean_y

        # FIXME: CONTROL STAGE, REFACTOR THIS TO FUNCTION
        self.control_list_steer.append(ctrl_steer)
        self.control_list_brake.append(ctrl_brake)
        self.control_list_acc.append(ctrl_acc)
        self.control_list_steer.pop(0)
        self.control_list_brake.pop(0)
        self.control_list_acc.pop(0)

        control_steer = sum(self.control_list_steer) / MOVING_MEAN_SIZE
        control_brake = sum(self.control_list_brake) / MOVING_MEAN_SIZE
        control_acc = sum(self.control_list_acc) / MOVING_MEAN_SIZE

        if control_steer > 23:
            control_steer = 23
        elif control_steer < -23:
            control_steer = -23

        # error_x = self.reference_pixel_x - control_x
        # error_y = self.reference_pixel_y - control_y

        # self.control_x = control_x
        # self.control_y = control_y

        # GEAR RATIO
        Kp = 1

        # Testing purposes set controls to 0 for debugging
        control_brake = 0
        control_acc = 0
        array = Float32MultiArray()
        array.data = [Kp*control_steer, Kp*control_brake, Kp*control_acc]

        self.control_pub.publish(array)


def main():
    rospy.init_node("lane_control", anonymous=True)
    obj = LaneControlImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
