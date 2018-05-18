import rospy
import numpy as np
from collections import namedtuple


LineInfo = namedtuple('LineInfo', ['x1', 'y1', 'x2', 'y2', 'center_x', 'center_y', 'length', 'inclination', 'color'])
LineMeansInfo = namedtuple('LineMeansInfo', ['mean_x', 'mean_y', 'count'])

min_length = 1
min_inclination = 0.4


""" TODO Implement kalman or particle filter for a dynamic model of the lanes, maybe this needs to be implemented
in another node or even a package named lane_filter """


def segment_to_line(segment_list):
    """
    Generator to return a LineInfo structure from a segment message, this can be implemented
    to create an array with line information an easily utilize this array to perform fast vectorized code
    :param segment_list:
    :yields: LineInfo object
    """

    for segment in segment_list.segments:
        x1 = segment.pixels_normalized[0].x
        y1 = segment.pixels_normalized[0].y
        x2 = segment.pixels_normalized[1].x
        y2 = segment.pixels_normalized[1].y

        if (x2-x1) == 0:
            inclination = float('inf')
        else:
            inclination = (y2-y1) / (x2-x1)

        line = LineInfo(x1=int(x1), y1=int(y1), x2=int(x2), y2=int(y2),
                        length=np.sqrt((x1-x2)**2 + (y1+y2**2)),
                        inclination=inclination,
                        center_x=(x2+x1)/2,
                        center_y=(y2+y1)/2,
                        color=segment.color)
        yield line


def get_right_segments(segment_list):
    """
    Takes list of segments an returns a line of the filtered segments of the rightmost curve (lane)
    :param segment_list:
    :return: Array with segment's start and end point coordinates
    """

    data_array = np.zeros((1, 4))

    center_x_pixel = rospy.get_param("~img_size")[1]/2

    for line in segment_to_line(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and abs(line.inclination) > min_inclination
                                                and line.center_x > center_x_pixel):
            vector = np.array([[line.x1, line.x2, line.y1, line.y2]])
            data_array = np.concatenate((data_array, vector), axis=0)

    # Index array to erase first row
    return np.array(data_array[1:])


def get_left_segments(segment_list):
    """
    Takes list of segments an returns a line of the filtered segments of the leftmost curve (lane)
    :param segment_list:
    :return: Array with segment's start and end point coordinates
    """

    data_array = np.zeros((1, 4))

    center_x_pixel = rospy.get_param("~img_size")[1]/2

    for line in segment_to_line(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and abs(line.inclination) > min_inclination
                                                and line.center_x < center_x_pixel):
            vector = np.array([[line.x1, line.x2, line.y1, line.y2]])
            data_array = np.concatenate((data_array, vector), axis=0)

    # Index array to erase first row
    return np.array(data_array[1:])


def get_correct_segments(segment_list):
    """
    Takes list of segments an returns a line of the filtered segments of the leftmost curve (lane)
    :param segment_list:
    :return: Array with segment's start and end point coordinates
    """

    data_array = np.zeros((1, 4))

    for line in segment_to_line(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and abs(line.inclination) > min_inclination):
            vector = np.array([[line.x1, line.x2, line.y1, line.y2]])
            data_array = np.concatenate((data_array, vector), axis=0)

    # Index array to erase first row
    return np.array(data_array[1:])


def line_means(segment_list):
    """
    Returns an named tuple with the information about the mean of the x points and the mean of the y points from
    the segment list. Take into account that this may be biased to the bottom of the image due to strong
    appearance of short segments.
    :param segment_list:
    :return: Named tuple
    """

    count_valid_line = 0
    mean_center_x = 0
    mean_center_y = 0

    for line in segment_to_line(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and abs(line.inclination) > min_inclination):
            count_valid_line += 1
            mean_center_x += line.center_x
            mean_center_y += line.center_y

    if count_valid_line == 0:
        mean_center_x = 0
        mean_center_y = 0
    else:
        mean_center_x /= count_valid_line
        mean_center_y /= count_valid_line

    return LineMeansInfo(mean_x=mean_center_x, mean_y=mean_center_y, count=count_valid_line)
