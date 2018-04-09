import numpy as np

from car_tec_msgs.msg import Segment


def to_segment_msg(lines, normals, color):

    segment_msg_list = []
    for x1, y1, x2, y2, norm_x, norm_y in np.hstack((lines, normals)):
        segment = Segment()
        segment.color = color
        segment.pixels_normalized[0].x = x1
        segment.pixels_normalized[0].y = y1
        segment.pixels_normalized[1].x = x2
        segment.pixels_normalized[1].y = y2
        segment.normal.x = norm_x
        segment.normal.y = norm_y

        segment_msg_list.append(segment)
    return segment_msg_list
