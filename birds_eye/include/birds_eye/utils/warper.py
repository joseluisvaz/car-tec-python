import cv2
import rospy
import numpy as np

from geometry_msgs.msg import Point
from car_tec_msgs.msg import Pixel
from car_tec_msgs.msg import Vector2D

AREA_OF_INTEREST_2 = [[150 + 430, 460], [1150 - 440, 460], [1150, 720], [150, 720]]
AREA_OF_INTEREST_3 = [[580, 290], [700, 290], [1280, 650], [0, 650]]

# apex 1; apex 2; right_bottom; left bottom
AREA_OF_INTEREST = [[280, 240], [380, 240], [550, 390], [110, 390]]


class Warper(object):

    def __init__(self):
        self.img = None
        self.warped_img = None
        self.width = None
        self.height = None
        self.img_size = None
        self.H = None
        self.H_inv = None

    def check_img_size(self, img):
        img_size = (img.shape[1], img.shape[0])
        if img_size == self.img_size:
            return True
        return False

    def set_image(self, img):
        if not self.check_img_size(img):
            raise ValueError("img_sizes do not match")
        self.img = img

    def set_image_size(self, img):
        self.width = img.shape[0]
        self.height = img.shape[1]
        self.img_size = (img.shape[1], img.shape[0])

    def set_homography(self, img):
        offset1 = rospy.get_param("~offset_x")          # offset for dst points x value
        offset2 = rospy.get_param("~offset_y_bottom")             # offset for dst points bottom y value
        offset3 = rospy.get_param("~offset_y_top")             # offset for dst points top y value

        if not self.check_img_size(img):
            raise ValueError("img_sizes do not match")

        src = np.float32(AREA_OF_INTEREST)

        dst = np.float32([[offset1, offset3],
                          [self.img_size[0]-offset1, offset3],
                          [self.img_size[0]-offset1, self.img_size[1]-offset2],
                          [offset1, self.img_size[1]-offset2]])

        self.H = cv2.getPerspectiveTransform(src, dst)
        self.H_inv = cv2.getPerspectiveTransform(dst, src)
        self.warped_img = cv2.warpPerspective(img, self.H, self.img_size)

        return self.H, self.H_inv

    def warp_image(self, img):
        if not self.check_img_size(img):
            raise ValueError("img_sizes do not match")

        if self.H is None:
            raise ValueError("Homography matrix is not initialized"
                             "Run set_homography(img)")

        return cv2.warpPerspective(img, self.H, self.img_size)

    def vector2ground(self, vec):
        pixel = self.vector2pixel(vec)
        return self.pixel2ground(pixel)

    def vector2pixel(self, vec):
        pixel = Pixel()
        pixel.u = self.width * vec.x
        pixel.v = self.height * vec.y

        if pixel.u < 0:
            pixel.u = 0
        if pixel.u > self.width - 1:
            pixel.u = self.width - 1
        if pixel.v < 0:
            pixel.v = 0
        if pixel.v > self.height - 1:
            pixel.v = 0
        return pixel

    def pixel2ground(self,pixel):
        uv_raw = np.array([pixel.u, pixel.v])
        uv_raw = np.append(uv_raw, np.array([1]))
        ground_point = np.dot(self.H, uv_raw)

        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x/z
        point.y = y/z
        point.z = 0.0
        return point

    def pixel2vector(self, pixel):
        vec = Vector2D()
        vec.x = pixel.u / self.width
        vec.y = pixel.v / self.height
        return vec

