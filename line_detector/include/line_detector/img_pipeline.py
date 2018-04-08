#!/usr/bin/env python

import cv2

from img_functions import component_sobel_tresh
from img_functions import magnitude_sobel_thresh
from img_functions import direction_sobel_thresh
from roi_cutter import RoiCutter

DEFAULT_KERNEL_SIZE = 7


def img_pipeline(img, kernel_size=DEFAULT_KERNEL_SIZE):

    # Color Conversions
    img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img_hsl = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s_channel = img_hsl[:, :, 2]

    # Initializes Region Cutter and sets its variables
    region_cutter = RoiCutter()
    region_cutter.set_img_shape(img_gray.shape)
    region_cutter.set_vertices()

    sobel_x_binary = component_sobel_tresh(img_gray, orientation="x", kernel_size=kernel_size, thresh=(10, 255))
    sobel_y_binary = component_sobel_tresh(img_gray, orientation="y", kernel_size=kernel_size, thresh=(60, 255))
    mag_img = magnitude_sobel_thresh(img_gray, kernel_size=kernel_size, thresh=(40, 255))
    dir_img = direction_sobel_thresh(img_gray, kernel_size=kernel_size, thresh=(.65, 1.05))

    combined_components = cv2.bitwise_and(sobel_x_binary, sobel_y_binary)
    combined_mag_dir = cv2.bitwise_and(mag_img, dir_img)

    combined_binary = cv2.bitwise_or(combined_components, combined_mag_dir)

    s_channel_binary = cv2.inRange(s_channel, 160, 255)
    s_channel_binary[s_channel_binary == 255] = 1

    color_binary = cv2.bitwise_or(s_channel_binary, combined_binary)
    color_binary_cropped = region_cutter.cut_region(color_binary)

    # Setting binary 1 to 255 (White)
    color_binary_cropped[color_binary_cropped == 1] = 255

    return color_binary_cropped
