import numpy as np
import cv2
import rospy

from tracker_interface import TrackerInterface

class DoubleLineTracker(TrackerInterface):

    def __init__(self):
        self.segments = None

    def draw_segments(self, segment_list):
        return None
