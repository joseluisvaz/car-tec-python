#!/usr/bin/env python

from __future__ import print_function

import rospy

from car_tec_msgs.msg import Segment
from car_tec_msgs.msg import SegmentList
from std_msgs.msg import Int32


class BoardConnectionImpl(object):
    def __init__(self):
        self.segment_list = None
        self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                            SegmentList,
                                            self.callback,
                                            queue_size=rospy.get_param("~subs_queue_size"),
                                            buff_size=rospy.get_param("~buff_size"))

        self.control_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
                                           Int32,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, segment_list):
        self.segment_list = segment_list
        self.control_pub.publish(len(self.segment_list.segments))


def main():
    rospy.init_node("board_connection", anonymous=True)
    obj = BoardConnectionImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
