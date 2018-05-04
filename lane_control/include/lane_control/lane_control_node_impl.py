#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from car_tec_msgs.msg import Segment
from car_tec_msgs.msg import SegmentList
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

from utils.lines_to_control import line_means
from utils.lines_to_control import lines_to_array
from utils.linear_regression import fit_line
from utils.lines_to_control import get_point_array
from utils.lines_to_control import unwrapslines

COLOR_RED = (0, 0, 255)

MOVING_MEAN_SIZE = 250
LIN_BUFFER = 15
LINE_THRESHOLD = 35


class LaneControlImpl(object):
    def __init__(self):
        self.control_list_x = [480 for x in range(MOVING_MEAN_SIZE)]
        self.control_list_y = [0 for x in range(MOVING_MEAN_SIZE)]
        self.centers_list = []
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

        self.slope = 0.01
        self.intercept = 0

        self.control_x = None
        self.control_y = None

        self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                            SegmentList,
                                            self.callback,
                                            queue_size=rospy.get_param("~subs_queue_size"),
                                            buff_size=rospy.get_param("~buff_size"))

        self.img_sub = rospy.Subscriber(rospy.get_param("~image_topic_1"),
                                        Image,
                                        self.img_callback,
                                        queue_size=rospy.get_param("~subs_queue_size"),
                                        buff_size=rospy.get_param("~buff_size"))

        self.control_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
                                           Float32MultiArray,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def img_callback(self, img_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        offset_x = 0 #280
        point1 = (int(450*self.slope + self.intercept) + offset_x, 450)
        point2 = (int(600*self.slope + self.intercept) + offset_x, 600)

        cv2.arrowedLine(cv_img,
                        point2,
                        point1,
                        (0, 255, 0),
                        2)

        if self.control_x is not None and self.control_y is not None:
            cv2.circle(cv_img, (int(self.control_x), int(self.control_y)), 10, (255, 0, 0), thickness=4)

        #cv2.imshow("direction", cv_img)
        #cv2.waitKey(1)

    def callback(self, segment_list):

        # control.u1 control.u2
        control_sample = line_means(segment_list)
        lines_array = lines_to_array(segment_list)

        prov_img = np.zeros(self.sample_img_shape, dtype=np.uint8)
        unwraps = unwrapslines(segment_list)

        for x1, x2, y1, y2 in unwraps:
            cv2.line(prov_img, (int(x1), int(y1)), (int(x2), int(y2)), COLOR_RED, 2)

        index_matrix = np.argwhere(prov_img == 255)

        if self.MESSAGE_COUNTER < LIN_BUFFER:
            self.MESSAGE_COUNTER += 1
            self.centers_list.append(index_matrix)
            self.centers_array = np.concatenate(self.centers_list, axis=0)
        elif control_sample.count < LINE_THRESHOLD:
            # HOLDS RESULT OF LAST LINEAR REGRESSION
            self.centers_list.append(index_matrix)
            self.centers_list.pop(0)
            self.centers_array = np.concatenate(self.centers_list, axis=0)
        else:
            self.centers_list.append(index_matrix)
            self.centers_list.pop(0)
            self.centers_array = np.concatenate(self.centers_list, axis=0)



        # FIXME: FIT LINE TO CENTERS ARRAY OR INDEX_MATRIX
        if control_sample.count > 0:
            self.slope, self.intercept = fit_line(self.centers_array, self.sample_img_shape)
            #self.slope, self.intercept = fit_line(index_matrix, self.sample_img_shape)


        # TODO: ERASE THIS
        offset_x = 0 #280
        point1 = (int(450*self.slope + self.intercept) + offset_x, 450)
        point2 = (int(600*self.slope + self.intercept) + offset_x, 600)

        cv2.arrowedLine(prov_img,
                        point2,
                        point1,
                        (0, 255, 0),
                        2)

        cv2.imshow("chucutru", prov_img)
        cv2.waitKey(1)


        # FIXME: CONTROL STAGE, REFACTOR THIS TO FUNCTION
        self.control_list_x.append(control_sample.u1)
        self.control_list_y.append(control_sample.u2)
        self.control_list_x.pop(0)
        self.control_list_y.pop(0)

        control_x = sum(self.control_list_x) / MOVING_MEAN_SIZE
        control_y = sum(self.control_list_y) / MOVING_MEAN_SIZE

        error_x = self.reference_pixel_x - control_x
        error_y = self.reference_pixel_y - control_y

        self.control_x = control_x
        self.control_y = control_y

        # GEAR RELATION
        Kp = 5
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
