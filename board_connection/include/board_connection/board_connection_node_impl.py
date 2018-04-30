#!/usr/bin/env python

from __future__ import print_function

import rospy

from car_tec_msgs.msg import Segment
from car_tec_msgs.msg import SegmentList
from std_msgs.msg import Float32MultiArray


class BoardConnectionImpl(object):
    def __init__(self):
        self.segment_list = None
        self.counter = 0
        self.direction = "right"
        rospy.Rate(10)
        self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                            SegmentList,
                                            self.callback,
                                            queue_size=rospy.get_param("~subs_queue_size"),
                                            buff_size=rospy.get_param("~buff_size"))

        self.control_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
                                           Float32MultiArray,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, segment_list):
        self.segment_list = segment_list
        if self.counter <= 270 and self.direction == "right":
            self.counter += 3
        elif self.counter > 270 and self.direction == "right":
            self.direction = "left"
        elif self.counter < -270 and self.direction == "left":
            self.direction = "right"
        elif self.counter >= -270 and self.direction == "left":
            self.counter -= 3

        a = len(self.segment_list.segments)*1.111
        array = Float32MultiArray()
        array.data = [270, 270]
        self.control_pub.publish(array)


def main():
    rospy.init_node("board_connection", anonymous=True)
    obj = BoardConnectionImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
