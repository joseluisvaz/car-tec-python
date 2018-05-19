#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2

from sensor_msgs.msg import Image
from car_tec_msgs.msg import Segment
from car_tec_msgs.msg import SegmentList
from cv_bridge import CvBridge
from line_detector_hough import LineDetectorHough
from line_detector_plot import draw_lines
from img_functions import draw_ROI
from utils.segments import to_segment_msg


COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)


class LineDetectorImpl:

    def __init__(self):
        self.bridge = CvBridge()
        self.detector = LineDetectorHough()

        self.image_sub = rospy.Subscriber(rospy.get_param("~subscriber_topic"),
                                          Image,
                                          self.callback,
                                          queue_size=rospy.get_param("~subs_queue_size"),
                                          buff_size=rospy.get_param("~buff_size"))

        self.image_pub = rospy.Publisher(rospy.get_param("~publisher_topic_1"),
                                         Image,
                                         queue_size=rospy.get_param("~pubs_queue_size"))

        self.edges_pub = rospy.Publisher(rospy.get_param("~publisher_topic_2"),
                                         Image,
                                         queue_size=rospy.get_param("~pubs_queue_size"))

        self.white_back_pub = rospy.Publisher(rospy.get_param("~publisher_topic_3"),
                                              Image,
                                              queue_size=rospy.get_param("~pubs_queue_size"))

        self.yellow_back_pub = rospy.Publisher(rospy.get_param("~publisher_topic_4"),
                                               Image,
                                               queue_size=rospy.get_param("~pubs_queue_size"))

        self.segments_pub = rospy.Publisher(rospy.get_param("~publisher_topic_segments"),
                                            SegmentList,
                                            queue_size=rospy.get_param("~pubs_queue_size"))

    def callback(self, img_msg):

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        self.detector.set_image(cv_image)

        white_info = self.detector.detect("white")
        yellow_info = self.detector.detect("yellow")

        segment_list = SegmentList()
        segment_list.header.stamp = img_msg.header.stamp


        # TODO: Change name of functions which say unormalized
        if len(white_info.lines) > 0:
            lines_normalized_white = white_info.lines #* self.detector.get_norm_ratio()
            segment_list.segments.extend(to_segment_msg(lines_normalized_white, white_info.normals, Segment.WHITE))
        if len(yellow_info.lines) > 0:
            lines_normalized_yellow = yellow_info.lines #* self.detector.get_norm_ratio()
            segment_list.segments.extend(to_segment_msg(lines_normalized_yellow, yellow_info.normals, Segment.YELLOW))

        draw_lines(cv_image, white_info.lines, COLOR_RED)
        draw_lines(cv_image, yellow_info.lines, COLOR_GREEN)

        cv_image = draw_ROI(cv_image, self.detector.roi_cutter.vertices)

        image_with_lines = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        edges_img = self.bridge.cv2_to_imgmsg(self.detector.edges, "mono8")
        white_back_img = self.bridge.cv2_to_imgmsg(white_info.area, "mono8")
        yellow_back_img = self.bridge.cv2_to_imgmsg(yellow_info.area, "mono8")

        self.image_pub.publish(image_with_lines)
        self.edges_pub.publish(edges_img)
        self.white_back_pub.publish(white_back_img)
        self.yellow_back_pub.publish(yellow_back_img)
        self.segments_pub.publish(segment_list)


def main():
    rospy.init_node('line_detector', anonymous=True)
    LineDetectorImpl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
