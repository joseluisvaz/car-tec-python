#!/usr/bin/env python

from __future__ import print_function

import rospy

from std_msgs.msg import Float32MultiArray


class BoardConnectionImpl(object):
    def __init__(self):
        self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                            Float32MultiArray,
                                            self.callback,
                                            queue_size=rospy.get_param("~subs_queue_size"),
                                            buff_size=rospy.get_param("~buff_size"))

        self.control_pub = rospy.Publisher(rospy.get_param("~publisher_topic"),
                                           Float32MultiArray,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, control):
        self.control_pub.publish(control)


def main():
    rospy.init_node("board_connection", anonymous=True)
    obj = BoardConnectionImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
