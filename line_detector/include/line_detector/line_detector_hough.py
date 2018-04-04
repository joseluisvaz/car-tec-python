import cv2
import rospy
import numpy as np

from line_detector_interface import LineDetectorInterface
from line_detector_interface import Detections
from img_pipeline import img_pipeline
from roi_cutter import RoiCutter

class LineDetector2(LineDetectorInterface):

    def __init__(self):
        self.bw_image = None
        self.hsv_image = None
        self.edges = None
        self.roi_cutter = RoiCutter()
        self.canny_thresholds = rospy.get_param("~color_config/canny_threshold")

    def set_image(self, rgb_image):
        self.bw_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        self.hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        self.roi_cutter.set_img_shape(self.bw_image.shape)
        self.roi_cutter.set_vertices()

        self.edges = self._find_edges(self.bw_image)

    def detect(self, color):
        bw, edge_color = self._color_filter(color)
        lines = self._hough_filter(edge_color)
        centers, normals = self._find_normal(bw, lines)
        return Detections(lines=lines, normals=normals, area=bw, centers=centers)

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
        # QUICK HACK ERASE AFTER DEMO  ------ dilatation = cv2.dilate(filtered, kernel)
        dilatation = cv2.dilate(filtered, kernel)
        edge_color = cv2.bitwise_and(dilatation,
                                     self.edges)
        return filtered, edge_color

    def _hough_filter(self, edge):
        lines = cv2.HoughLinesP(edge,                                 # image
                                1,                                    # rho
                                np.pi/180,                            # theta
                                rospy.get_param("~color_config/hough_threshold"),  # threshold
                                lines=np.empty(1),
                                minLineLength=rospy.get_param("~color_config/hough_min_line_length"),
                                maxLineGap=rospy.get_param("~color_config/hough_max_line_gap"))
        if lines is not None:
            lines = np.array(lines[:, 0])
        else:
            lines = []
        return lines

    def _lineFilter(self, bw, edge_color):
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

        lines = self._synthesizeLines(centers, normals)

        return lines, normals, centers

    def _synthesizeLines(self, centers, normals):
        lines = []
        if len(centers)>0:
            x1 = (centers[:,0:1] + normals[:, 1:2] * 6.).astype('int')
            y1 = (centers[:,1:2] - normals[:, 0:1] * 6.).astype('int')
            x2 = (centers[:,0:1] - normals[:, 1:2] * 6.).astype('int')
            y2 = (centers[:,1:2] + normals[:, 0:1] * 6.).astype('int')
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

    def _check_bounds(self, val, bound):
        val[val < 0] = 0
        val[val >= bound] = bound - 1
        return val

    def _correct_pixel_ordering(self, lines, normals):
        flag = ((lines[:, 2]-lines[:, 0])*normals[:, 1] - (lines[:, 3]-lines[:, 1])*normals[:, 0]) > 0
        for i in range(len(lines)):
            if flag[i]:
                x1, y1, x2, y2 = lines[i, :]
                lines[i, :] = [x2, y2, x1, y1]

    def _find_normal(self, bw, lines):
        normals = []
        centers = []
        if len(lines)>0:
            length = np.sum((lines[:, 0:2] -lines[:, 2:4])**2, axis=1, keepdims=True)**0.5
            dx = 1. * (lines[:, 3:4]-lines[:, 1:2])/length
            dy = 1. * (lines[:, 0:1]-lines[:, 2:3])/length

            centers = np.hstack([(lines[:,0:1]+lines[:,2:3])/2, (lines[:,1:2]+lines[:,3:4])/2])
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
