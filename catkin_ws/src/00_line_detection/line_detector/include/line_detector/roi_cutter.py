import cv2
import numpy as np

from img_functions import fill_poly


class RoiCutter(object):

    def __init__(self):
        """
        Initializes Cutter with shape
        """
        self.shape = None
        self.vertices = None

    def set_img_shape(self, shape):
        self.shape = shape

    def set_vertices(self):
        if self.shape is None:
            raise ValueError

        left_bottom = (20, self.shape[0])
        right_bottom = (self.shape[1] - 20, self.shape[0])
        apex1 = (310, 140)
        apex2 = (350, 140)
        inner_left_bottom = (40, self.shape[0])
        inner_right_bottom = (self.shape[1] - 40, self.shape[0])
        inner_apex1 = (340, 480)
        inner_apex2 = (320, 480)
        self.vertices = np.array([[left_bottom, apex1, apex2,
                                   right_bottom, inner_right_bottom,
                                   inner_apex1, inner_apex2, inner_left_bottom]],
                                 dtype=np.int32)

    def cut_region(self, img):
        if self.shape is None:
            print("Shape is None Error")
            raise ValueError

        if self.shape != img.shape:
            print("Shape != img.shape Error")
            raise ValueError

        if self.vertices is None:
            print("vertices is None Error")
            raise ValueError

        mask = np.zeros_like(img)

        fill_poly(mask, self.vertices)

        return cv2.bitwise_and(img, mask)
