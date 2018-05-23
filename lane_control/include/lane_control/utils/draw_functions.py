import cv2
import numpy as np

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)


def draw_linear_regression_double(activated_pixels, slope_left, slope_right, intercept_left, intercept_right):
    """
    Prints verbously a cv image in a window just for debuggins purposes
    :param activated_pixels:
    :param slope_left:
    :param slope_right:
    :param intercept_left:
    :param intercept_right:
    :return:
    """

    offset_x = 0

    if slope_left is not None:
        point1 = (int(450 * slope_left + intercept_left) + offset_x, 450)
        point2 = (int(600 * slope_left + intercept_left) + offset_x, 600)

        cv2.arrowedLine(activated_pixels,
                        point2,
                        point1,
                        COLOR_GREEN,
                        2)

    if slope_right is not None:
        point3 = (int(450 * slope_right + intercept_right) + offset_x, 450)
        point4 = (int(600 * slope_right + intercept_right) + offset_x, 600)
        cv2.arrowedLine(activated_pixels,
                        point4,
                        point3,
                        COLOR_RED,
                        2)

    cv2.imshow("linear_regression", activated_pixels)
    cv2.waitKey(1)


def draw_linear_regression_single(activated_pixels, slope, intercept):
    """
    Prints verbously a cv image in a window just for debuging purposes
    :param activated_pixels:
    :param slope:
    :param intercept:
    :return:
    """

    offset_x = 0

    y1 = 300
    y2 = 400
    slice_y = 250

    if slope is not None:
        point1 = (int(y1 * slope + intercept) + offset_x, y1)
        point2 = (int(y2 * slope + intercept) + offset_x, y2)

        point_to_follow = (int(slice_y*slope + intercept), slice_y)

        cv2.circle(activated_pixels, point_to_follow, 10, COLOR_RED, thickness=3)
        cv2.arrowedLine(activated_pixels,
                        point2,
                        point1,
                        COLOR_GREEN,
                        2)

    cv2.imshow("linear_regression", activated_pixels)
    cv2.waitKey(1)
