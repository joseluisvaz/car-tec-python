import numpy as np
import cv2
import rospy

from tracker_interface import TrackerInterface
from utils.lines_to_control import get_correct_segments
from utils.linear_regression import fit_line
from utils.draw_functions import draw_linear_regression_single

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)


class SingleLineTracker(TrackerInterface):
    def __init__(self, img_shape=None, lin_buffer=None, line_threshold=None):
        self.segments = None
        self.activated_pixels = None
        self.dataset_list = []
        self.MESSAGE_COUNTER = 0
        self.centers_array = None

        if img_shape is None:
            raise ValueError("Img size not setted in DoubleLineTracker instantiation")
        if lin_buffer is None:
            raise ValueError("lin_buffer size not setted in DoubleLineTracker instantiation")
        if line_threshold is None:
            raise ValueError("line_threshold size not setted in DoubleLineTracker instantiation")

        self.img_shape = img_shape
        self.lin_buffer = lin_buffer
        self.line_threshold = line_threshold

    def _reset_activated_pixels(self):
        self.activated_pixels = np.zeros(self.img_shape, dtype=np.uint8)

    def track_segments(self, seg_list=None, control_sample=None):

        if seg_list is None:
            raise ValueError("seg list not setted")
        if control_sample is None:
            raise ValueError("ctrl not setted")

        self._reset_activated_pixels()

        for x1, x2, y1, y2 in get_correct_segments(seg_list):
            cv2.line(self.activated_pixels, (int(x1), int(y1)), (int(x2), int(y2)), COLOR_RED, 2)

        index_segments = np.argwhere(self.activated_pixels[:, :, 2] == 255)

        # FIXME: PROVISIONAL STUFF TO SAVE PAST STATES

        if self.MESSAGE_COUNTER < self.lin_buffer:
            self.MESSAGE_COUNTER += 1
            self.dataset_list.append(index_segments)
            self.centers_array = np.concatenate(self.dataset_list, axis=0)
        elif control_sample.count < self.line_threshold:
            # HOLDS RESULT OF LAST LINEAR REGRESSION
            self.dataset_list.append(index_segments)
            self.dataset_list.pop(0)
            self.centers_array = np.concatenate(self.dataset_list, axis=0)
        else:
            self.dataset_list.append(index_segments)
            self.dataset_list.pop(0)
            self.centers_array = np.concatenate(self.dataset_list, axis=0)

        # FIXME: FIT LINE TO CENTERS ARRAY OR INDEX_MATRIX
        # FIXME: WHAT HAPPENS WHEN INPUTS ARE EMPTY, NO LINEAR REGRESSION TO CALCULATE
        lane = None

        if index_segments is not None:
            lane = fit_line(index_segments)

        if rospy.get_param("~verbose") == 1 and lane is not None:
            draw_linear_regression_single(self.activated_pixels, lane.slope, lane.intercept)

        return lane
