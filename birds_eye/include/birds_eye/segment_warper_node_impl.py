#!/usr/bin/env python

from __future__ import print_function

import rospy
import numpy as np

from cv_bridge import CvBridge

from car_tec_msgs.msg import SegmentList
from birds_eye.utils.warper import Warper


class SegmentWarperImpl(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.warper = Warper()
        self.segment_list = None
        self.configured = False
        self.H = np.array(rospy.get_param("~homography"))
        self.segment_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic_1"),
                                            SegmentList,
                                            self.callback,
                                            queue_size=rospy.get_param("~subs_queue_size"),
                                            buff_size=rospy.get_param("~buff_size"))

        self.segment_pub = rospy.Publisher(rospy.get_param("~publisher_topic_1"),
                                           SegmentList,
                                           queue_size=rospy.get_param("~pubs_queue_size"))

    # TODO: VECTORIZE THIS CODE
    def warp_vector(self, vector):
        """
        Vector [2x1]
        :param vector:
        :return: transformed vector
        """
        vector = np.concatenate([vector, np.array([[1]])], axis=0)
        proy = np.dot(self.H, vector)
        proy[0] = proy[0] / proy[2]
        proy[1] = proy[1] / proy[2]
        proy[2] = 0
        return proy[:-1]

    def callback(self, segment_list_msg):

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


def main():
    rospy.init_node("segment_warper", anonymous=True)
    obj = SegmentWarperImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
