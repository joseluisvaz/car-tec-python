#!/usr/bin/env python

import cv2
import numpy as np


def component_sobel_tresh(img_grayscale, orientation="x", kernel_size=3, thresh=(0, 255)):

    if orientation == "x":
        img_sobel = cv2.Sobel(img_grayscale, cv2.CV_64F, 1, 0, ksize=kernel_size)
    elif orientation == "y":
        img_sobel = cv2.Sobel(img_grayscale, cv2.CV_64F, 0, 1, ksize=kernel_size)
    else:
        raise ValueError

    img_sobel = np.absolute(img_sobel)
    img_sobel_scaled = np.uint8(255*img_sobel/np.max(img_sobel))

    binary_output = cv2.inRange(img_sobel_scaled, thresh[0], thresh[1])
    binary_output[binary_output == 255] = 1

    return binary_output


def magnitude_sobel_thresh(img_grayscale, kernel_size=3, thresh=(0, 255)):

    grad_x = cv2.Sobel(img_grayscale, cv2.CV_64F, 1, 0, ksize=kernel_size)
    grad_y = cv2.Sobel(img_grayscale, cv2.CV_64F, 0, 1, ksize=kernel_size)

    gradient_mag = np.sqrt(grad_x**2 + grad_y**2)
    gradient_mag_scaled = np.uint8(255*gradient_mag / np.max(gradient_mag))

    binary_output = cv2.inRange(gradient_mag_scaled, thresh[0], thresh[1])
    binary_output[binary_output == 255] = 1

    return binary_output


def direction_sobel_thresh(img_grayscale, kernel_size=3, thresh=(0, np.pi/2)):

    grad_x = cv2.Sobel(img_grayscale, cv2.CV_64F, 1, 0, ksize=kernel_size)
    grad_y = cv2.Sobel(img_grayscale, cv2.CV_64F, 0, 1, ksize=kernel_size)

    # To ignore division and invalid errors
    with np.errstate(divide="ignore", invalid="ignore"):
        gradient_abs_direction = np.absolute(np.arctan(grad_y/grad_x))
        binary_output = cv2.inRange(gradient_abs_direction, thresh[0], thresh[1])
        binary_output[binary_output == 255] = 1

    return binary_output


def fill_poly(mask, vertices):
    """
    Wraps the opencv method of cv2.fillPoly
    """

    if len(mask.shape) > 2:
        channel_count = mask.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    cv2.fillPoly(mask, vertices, ignore_mask_color)


def draw_ROI(img, vertices):
    mask = np.zeros_like(img)
    fill_poly(mask, vertices)

    # scale binary image to 255
    mask *= 255

    # Clear blue and red channels
    mask[:, :, 0] = 0
    mask[:, :, 2] = 0
    return cv2.addWeighted(img, 1, mask, 20, 0)
