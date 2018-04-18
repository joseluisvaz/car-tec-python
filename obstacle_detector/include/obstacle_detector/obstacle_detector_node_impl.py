#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2

from sensor_msgs.msg import Image
from car_tec_msgs.msg import Segment
from car_tec_msgs.msg import SegmentList
from cv_bridge import CvBridge


class ObstacleDetector():
    def __init__(self):
        self.message = "hola"

    
def main():
    obj = ObstacleDetector()
    print(obj.message)
