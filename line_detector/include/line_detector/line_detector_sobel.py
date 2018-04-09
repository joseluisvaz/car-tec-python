#!/usr/bin/env python

import cv2
import rospy
import numpy as np

from line_detector_interface import LineDetectorInterface
from line_detector_interface import Detections
from roi_cutter import RoiCutter


class LineDetectorSobel(LineDetectorInterface):

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

        self.edges = self._find_edges(self.bw_image)

    def detect(self, color):
        bw, edge_color = self._color_filter(color)
        lines, normals, centers = self._line_filter(bw, edge_color)
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
            filtered1 = cv2.inRange(self.hls_image,
                                    tuple(rospy.get_param("~color_config/hsv_red1")),
                                    tuple(rospy.get_param("~color_config/hsv_red2")))
            filtered2 = cv2.inRange(self.hls_image,
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
        # QUICK HACK ERASE AFTER DEMO  ------ dilatation = cv2.dilate(filtered, kernel)
        dilatation = cv2.dilate(filtered, kernel)
        edge_color = cv2.bitwise_and(dilatation,
                                     self.edges)
        return filtered, edge_color

    def _line_filter(self, bw, edge_color):
        # find gradient of the bw image
        grad_x = -cv2.Sobel(bw/255, cv2.CV_32F, 1, 0, ksize=5)
        grad_y = -cv2.Sobel(bw/255, cv2.CV_32F, 0, 1, ksize=5)
        grad_x *= (edge_color == 255)
        grad_y *= (edge_color == 255)

        # compute gradient and thresholding
        grad = np.sqrt(grad_x**2 + grad_y**2)
        roi = (grad > rospy.get_param("~color_config/sobel_threshold"))

        # turn into a list of points and normals
        roi_y, roi_x = np.nonzero(roi)
        centers = np.vstack((roi_x, roi_y)).transpose()
        normals = np.vstack((grad_x[roi], grad_y[roi])).transpose()
        normals /= np.sqrt(np.sum(normals**2, axis=1, keepdims=True))

        lines = self._synthesize_lines(centers, normals)

        return lines, normals, centers

    def _synthesize_lines(self, centers, normals):
        lines = []
        if len(centers) > 0:
            x1 = (centers[:, 0:1] + normals[:, 1:2] * 6.).astype('int')
            y1 = (centers[:, 1:2] - normals[:, 0:1] * 6.).astype('int')
            x2 = (centers[:, 0:1] - normals[:, 1:2] * 6.).astype('int')
            y2 = (centers[:, 1:2] + normals[:, 0:1] * 6.).astype('int')
            x1 = self._check_bounds(x1, self.hsv_image.shape[1])
            y1 = self._check_bounds(y1, self.hsv_image.shape[0])
            x2 = self._check_bounds(x2, self.hsv_image.shape[1])
            y2 = self._check_bounds(y2, self.hsv_image.shape[0])
            lines = np.hstack([x1, y1, x2, y2])
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

    def get_norm_ratio(self):
        return np.array((1./self.image_size[1],
                         1./self.image_size[0],
                         1./self.image_size[1],
                         1./self.image_size[0]))
