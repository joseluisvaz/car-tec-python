import numpy as np
import scipy
from collections import namedtuple

from car_tec_msgs.msg import SegmentList
from car_tec_msgs.msg import Segment

LineInfo = namedtuple('LineInfo', ['x1', 'y1', 'x2', 'y2', 'center_x', 'center_y', 'length', 'inclination', 'color'])
ControlInfo = namedtuple('Control', ['u1', 'u2'])

min_length = 3
min_inclination = 0.8


# TODO Implement kalman or particle filter for a dynamic model of the lanes

def segments_to_lines(segment_list):

    for segment in segment_list.segments:
        x1 = segment.pixels_normalized[0].x
        y1 = segment.pixels_normalized[0].y
        x2 = segment.pixels_normalized[1].x
        y2 = segment.pixels_normalized[1].y

        if (x2-x1) == 0:
            inclination = float('inf')
        else:
            inclination = (y2-y1) / (x2-x1)

        line = LineInfo(x1=x1, y1=y1, x2=x2, y2=y2,
                        length=np.sqrt((x1-x2)**2 + (y1+y2**2)),
                        inclination=inclination,
                        center_x=(x2+x1)/2,
                        center_y=(y2+y1)/2,
                        color=segment.color)
        yield line


def line_means(segment_list):

    count_valid_line = 0
    mean_center_x = 0
    mean_center_y = 0

    for line in segments_to_lines(segment_list):
        if line.inclination == float('inf') or (line.length > min_length and line.inclination > min_inclination):
            count_valid_line += 1
            mean_center_x += line.center_x
            mean_center_y += line.center_y

    if count_valid_line == 0:
        mean_center_x = 0
        mean_center_y= 0
    else:
        mean_center_x /= count_valid_line
        mean_center_y /= count_valid_line

    return ControlInfo(u1=mean_center_x, u2=mean_center_y)


