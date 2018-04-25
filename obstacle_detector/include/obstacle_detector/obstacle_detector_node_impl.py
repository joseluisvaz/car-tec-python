#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2

from sensor_msgs.msg import Image
from car_tec_msgs.msg import Segment
from car_tec_msgs.msg import SegmentList
from cv_bridge import CvBridge
import depthimage_to_laserscan as laserimg


class ObstacleDetectorImpl(object):
    def __init__(self):
        self.message = "hola"


def main():
    rospy.init_node("obstacle_detectot", anonymous=True)
    obj = ObstacleDetectorImpl()
    print(obj.message)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
