import cv2
import rospy
import numpy as np

from line_detector_interface import LineDetectorInterface


class LineDetector(LineDetectorInterface):

    def __init__(self):
        self.bw_image = None
        self.hsv_image = None
        self.edges = None
        self.canny_thresholds = rospy.get_param("~color_config/canny_threshold")

    def set_image(self, rgb_image):
        self.bw_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        self.hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        self.edges = self._find_edges(self.bw_image)

    def detect(self, rgb_image, color):
        self.set_image(rgb_image)
        return self._color_filter(color)

    def _color_filter(self, color):
        if color == "white":
            filtered = cv2.inRange(self.hsv_image,
                                   tuple(rospy.get_param("~color_config/hsv_white1")),
                                   tuple(rospy.get_param("~color_config/hsv_white2")))
        elif color == "yellow":
            filtered = cv2.inRange(self.hsv_image,
                                   tuple(rospy.get_param("~color_config/hsv_yellow1")),
                                   tuple(rospy.get_param("~color_config/hsv_yellow2")))
        elif color == "red":
            filtered1 = cv2.inRange(self.hsv_image,
                                    tuple(rospy.get_param("~color_config/hsv_red1")),
                                    tuple(rospy.get_param("~color_config/hsv_red2")))
            filtered2 = cv2.inRange(self.hsv_image,
                                    tuple(rospy.get_param("~color_config/hsv_red3")),
                                    tuple(rospy.get_param("~color_config/hsv_red4")))
            filtered = cv2.bitwise_and(filtered1, filtered2)
        else:
            raise Exception("Incorrect color, choose [white, yellow, red]")

        # BINARY DILATATION
        dilatation_kernel_size = rospy.get_param("~color_config/dilatation_kernel_size")
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (dilatation_kernel_size, dilatation_kernel_size))
        # EDGES FOR CERTAIN COLOR
        dilatation = cv2.dilate(filtered, kernel)

        edge_color = cv2.bitwise_and(dilatation,
                                     self.edges)
        return filtered, edge_color


    def _find_edges(self, bw_image):
        return cv2.Canny(bw_image,
                         self.canny_thresholds[0],
                         self.canny_thresholds[1],
                         apertureSize=3)
