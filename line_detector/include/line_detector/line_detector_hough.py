#!/usr/bin/env python

import cv2
import rospy
import numpy as np

from line_detector_interface import LineDetectorInterface
from line_detector_interface import Detections
from roi_cutter import RoiCutter


class LineDetectorHough(LineDetectorInterface):

    def __init__(self):
        self.bw_image = None
        self.hsv_image = None
        self.hls_image = None
        self.image_size = None
        self.edges = None
        self.roi_cutter = RoiCutter()
        self.canny_thresholds = rospy.get_param("~color_config/canny_threshold")

    def set_image(self, bgr_image):
        self.bw_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        self.hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        self.hls_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HLS)
        self.image_size = bgr_image.shape[0:2]

        self.roi_cutter.set_img_shape(self.bw_image.shape)
        self.roi_cutter.set_vertices()

        self.edges = self._find_edges(bgr_image)

    def detect(self, color):
        bw, edge_color = self._color_filter(color)
        lines = self._hough_filter(edge_color)
        centers, normals = self._find_normal(bw, lines)
        return Detections(lines=lines, normals=normals, area=bw, centers=centers)

    def _color_filter(self, color):
        if color == "white":
            filtered = cv2.inRange(self.hls_image,
                                   tuple(rospy.get_param("~color_config/hls_white1")),
                                   tuple(rospy.get_param("~color_config/hls_white2")))
        elif color == "yellow":
            filtered = cv2.inRange(self.hls_image,
                                   tuple(rospy.get_param("~color_config/hls_yellow1")),
                                   tuple(rospy.get_param("~color_config/hls_yellow2")))
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

    @staticmethod
    def _hough_filter(edge):
        lines = cv2.HoughLinesP(edge,                                              # image
                                1,                                                 # rho
                                np.pi/180,                                         # theta
                                rospy.get_param("~color_config/hough_threshold"),  # threshold
                                lines=np.empty(1),
                                minLineLength=rospy.get_param("~color_config/hough_min_line_length"),
                                maxLineGap=rospy.get_param("~color_config/hough_max_line_gap"))
        if lines is not None:
            lines = np.array(lines[:, 0])
        else:
            lines = []
        return lines

    def _find_edges(self, bw_image):
        canny = cv2.Canny(bw_image,
                          self.canny_thresholds[0],
                          self.canny_thresholds[1],
                          apertureSize=3)

        self.roi_cutter.set_img_shape(canny.shape)
        self.roi_cutter.set_vertices()

        return self.roi_cutter.cut_region(canny)

    @staticmethod
    def _check_bounds(val, bound):
        val[val < 0] = 0
        val[val >= bound] = bound - 1
        return val

    @staticmethod
    def _correct_pixel_ordering(lines, normals):
        flag = ((lines[:, 2]-lines[:, 0])*normals[:, 1] - (lines[:, 3]-lines[:, 1])*normals[:, 0]) > 0
        for i in range(len(lines)):
            if flag[i]:
                x1, y1, x2, y2 = lines[i, :]
                lines[i, :] = [x2, y2, x1, y1]

    def _find_normal(self, bw, lines):
        normals = []
        centers = []
        if len(lines) > 0:
            length = np.sum((lines[:, 0:2] - lines[:, 2:4])**2, axis=1, keepdims=True)**0.5
            dx = 1. * (lines[:, 3:4]-lines[:, 1:2])/length
            dy = 1. * (lines[:, 0:1]-lines[:, 2:3])/length

            centers = np.hstack([(lines[:, 0:1]+lines[:, 2:3])/2, (lines[:, 1:2]+lines[:, 3:4])/2])
            x3 = (centers[:, 0:1] - 3.*dx).astype('int')
            y3 = (centers[:, 1:2] - 3.*dy).astype('int')
            x4 = (centers[:, 0:1] + 3.*dx).astype('int')
            y4 = (centers[:, 1:2] + 3.*dy).astype('int')
            x3 = self._check_bounds(x3, bw.shape[1])
            y3 = self._check_bounds(y3, bw.shape[0])
            x4 = self._check_bounds(x4, bw.shape[1])
            y4 = self._check_bounds(y4, bw.shape[0])
            flag_signs = 2*(np.logical_and(bw[y3, x3] > 0, bw[y4, x4] == 0)).astype('int') - 1
            normals = np.hstack([dx, dy]) * flag_signs

            self._correct_pixel_ordering(lines, normals)
        return centers, normals

    def get_norm_ratio(self):
        return np.array((1./self.image_size[1],
                         1./self.image_size[0],
                         1./self.image_size[1],
                         1./self.image_size[0]))
