import cv2
import numpy as np

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)


def draw_linear_regression(self, activated_pixels, slope_left, slope_right, intercept_left, intercept_right):
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
    point1 = (int(450 * slope_left + intercept_left) + offset_x, 450)
    point2 = (int(600 * slope_left + intercept_left) + offset_x, 600)
    point3 = (int(450 * slope_right + intercept_right) + offset_x, 450)
    point4 = (int(600 * slope_right + intercept_right) + offset_x, 600)

    theta = np.arctan2(point1[0], point1[1]) * 180/np.pi

    cv2.arrowedLine(activated_pixels,
                    point2,
                    point1,
                    COLOR_GREEN,
                    2)
    cv2.arrowedLine(activated_pixels,
                    point4,
                    point3,
                    COLOR_RED,
                    2)

    cv2.imshow("linear_regression", activated_pixels)
    cv2.waitKey(1)
