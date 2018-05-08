import numpy as np
from collections import namedtuple


LineInfo = namedtuple('LineInfo', ['x1', 'y1', 'x2', 'y2', 'center_x', 'center_y', 'length', 'inclination', 'color'])
ControlInfo = namedtuple('Control', ['u1', 'u2', 'count'])

min_length = 1
min_inclination = 0.4


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

        line = LineInfo(x1=int(x1), y1=int(y1), x2=int(x2), y2=int(y2),
                        length=np.sqrt((x1-x2)**2 + (y1+y2**2)),
                        inclination=inclination,
                        center_x=(x2+x1)/2,
                        center_y=(y2+y1)/2,
                        color=segment.color)
        yield line


def line_to_points(line):
    if line.inclination < 0:
        in_x = list(range(line.x2, line.x1 + 1))
        in_y = list(range(line.y2, line.y1 + 1))
    else:
        in_x = list(range(line.x1, line.x2 + 1))
        in_y = list(range(line.y1, line.y2 + 1))

    print(len(in_x), len(in_y))
    array = np.concatenate((np.array(in_x, ndmin=2), np.array(in_y, ndmin=2)), axis=0)
    return np.transpose(array)


def line_means(segment_list):

    count_valid_line = 0
    mean_center_x = 0
    mean_center_y = 0

    for line in segments_to_lines(segment_list):
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

    return ControlInfo(u1=mean_center_x, u2=mean_center_y, count=count_valid_line)


def get_point_array(segment_list):

    count_valid_line = 0
    mean_center_x = 0
    mean_center_y = 0
    point_list = []
    point_array = None
    count = 0

    for line in segments_to_lines(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and abs(line.inclination) > min_inclination):
            point_list.append(line_to_points(line))
            count_valid_line += 1
            mean_center_x += line.center_x
            mean_center_y += line.center_y

    if count_valid_line == 0:
        mean_center_x = 0
        mean_center_y = 0
    else:
        point_array = np.concatenate(point_list, axis=0)
        mean_center_x /= count_valid_line
        mean_center_y /= count_valid_line
        count = point_array.shape[0]

    return point_array, count


def unwraps_left(segment_list):

    data_array = np.zeros((1, 4))

    for line in segments_to_lines(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and abs(line.inclination) > min_inclination#):
                                                and line.center_x < 640):
            vector = np.array([[line.x1, line.x2, line.y1, line.y2]])
            data_array = np.concatenate((data_array, vector), axis=0)

    return np.array(data_array[1:])


def unwraps_right(segment_list):

    data_array = np.zeros((1, 4))

    for line in segments_to_lines(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and abs(line.inclination) > min_inclination#):
                                                and line.center_x > 640):
            vector = np.array([[line.x1, line.x2, line.y1, line.y2]])
            data_array = np.concatenate((data_array, vector), axis=0)

    return np.array(data_array[1:])


def lines_to_array(segment_list):

    data_array = np.zeros((1, 2))

    for line in segments_to_lines(segment_list):
        if line.inclination == float('inf') or (line.length > min_length
                                                and line.inclination > min_inclination
                                                and line.center_x < 640):

            vector = np.array([[line.center_x, line.center_y]])
            data_array = np.concatenate((data_array, vector), axis=0)

    return np.array(data_array[1:])




