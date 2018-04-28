import cv2
import numpy as np


area_of_interest = [[150+430, 460], [1150-440, 460], [1150, 720], [150, 720]]


def corners_unwarp(img):
    offset1 = 200           # offset for dst points x value
    offset2 = 0             # offset for dst points bottom y value
    offset3 = 0             # offset for dst points top y value

    img_size = (img.shape[1], img.shape[0])

    src = np.float32(area_of_interest)

    dst = np.float32([[offset1, offset3],
                      [img_size[0]-offset1, offset3],
                      [img_size[0]-offset1, img_size[1]-offset2],
                      [offset1, img_size[1]-offset2]])

    homography_matrix = cv2.getPerspectiveTransform(src, dst)
    homography_matrix_inv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, homography_matrix, img_size)

    return warped, homography_matrix, homography_matrix_inv
