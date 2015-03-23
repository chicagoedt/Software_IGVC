#!/usr/bin/env python
import sys
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig
import cv2
import numpy as np

###############################################################################
# Chicago Engineering Design Team
# Skeletonize filter using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
# @author Basheer Subei
# @email basheersubei@gmail.com


class Skeletonize(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000
    max_erode_iterations = 150

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)

        assert self.use_mono
        # skeletonize image
        size = np.size(roi)
        skel = np.zeros(roi.shape, np.uint8)
        element = cv2.getStructuringElement(
            shape=cv2.MORPH_CROSS,
            ksize=(3, 3)
        )
        # iteratively erode, dilate, subtract, then OR the image
        # until it's 1 pixel thick
        for i in range(self.max_erode_iterations):
            eroded = cv2.erode(src=roi, kernel=element)
            temp = cv2.dilate(src=eroded, kernel=element)
            temp = cv2.subtract(src1=roi, src2=temp)
            skel = cv2.bitwise_or(src1=skel, src2=temp)
            roi = eroded.copy()

            zeros = size - cv2.countNonZero(roi)
            if zeros == size:
                break

        final_image = skel
        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )

        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "skeletonize"
    namespace = rospy.get_namespace()

    # create a Skeletonize object
    s = Skeletonize(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("skeletonize", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, s.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
