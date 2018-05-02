#!/usr/bin/env python

from __future__ import print_function

import rospy
from cv_bridge import CvBridge

from car_tec_msgs.msg import Segment
from car_tec_msgs.msg import SegmentList
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

from utils.lines_to_control import line_means

MOVING_MEAN_SIZE = 100


class LaneControlImpl(object):
    def __init__(self):
        self.control_list_x = [0 for x in range(MOVING_MEAN_SIZE)]
        self.control_list_y = [0 for x in range(MOVING_MEAN_SIZE)]
        self.MESSAGE_COUNTER = 0
        bridge = CvBridge()

        sample_img_message = rospy.wait_for_message(rospy.get_param("~image_topic"), Image)
        sample_img_shape = bridge.imgmsg_to_cv2(sample_img_message, "bgr8").shape

        self.img_size_x = sample_img_shape[1]
        self.img_size_y = sample_img_shape[0]
        self.reference_pixel_x = int(self.img_size_x/2)
        self.reference_pixel_y = int(self.img_size_y/2)

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
        self.control_list_x.append(control_sample.u1)
        self.control_list_y.append(control_sample.u2)
        self.control_list_x.pop(0)
        self.control_list_y.pop(0)

        control_x = sum(self.control_list_x) / MOVING_MEAN_SIZE
        control_y = sum(self.control_list_y) / MOVING_MEAN_SIZE

        error_x = self.reference_pixel_x - control_x
        error_y = self.reference_pixel_y - control_y

        # Proportional Control
        Kp = 4
        array = Float32MultiArray()
        array.data = [Kp*error_x, Kp*error_y]

        if self.MESSAGE_COUNTER == 5:
            self.control_pub.publish(array)
            self.MESSAGE_COUNTER = 0
        self.MESSAGE_COUNTER += 1


def main():
    rospy.init_node("lane_control", anonymous=True)
    obj = LaneControlImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
