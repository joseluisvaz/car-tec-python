#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from img_pipeline import img_pipeline


class LineDetectorImpl:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed/rgb/image_raw_color",
                                          Image,
                                          self.callback,
                                          queue_size=1,
                                          buff_size=52428800)

        self.image_pub = rospy.Publisher("/line_detector/output_image",
                                         Image,
                                         queue_size=1)

    def callback(self, data):

        cv_image = self._to_cv_image(data)

        img_cropped = img_pipeline(cv_image)

        ros_img = self._to_ros_image(img_cropped)
        self.image_pub.publish(ros_img)

    def _to_cv_image(self, data):
        try:
            return self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def _to_ros_image(self, data):
        try:
            return self.bridge.cv2_to_imgmsg(data, "mono8")
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = LineDetectorImpl()
    rospy.init_node('line_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
