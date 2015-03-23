#!/usr/bin/env python
import sys
import cv2
import rospy
import math
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
# Chicago Engineering Design Team
# Gabor filter using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
# @author Basheer Subei
# @email basheersubei@gmail.com


class Gabor(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000
    gabor_ksize = 4
    gabor_sigma = 7
    gabor_theta = 0
    gabor_lambda = 27
    gabor_gamma = 4.0

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)

        # apply Gabor filter
        gabor_kernel = cv2.getGaborKernel(
            ksize=(self.gabor_ksize, self.gabor_ksize),
            sigma=self.gabor_sigma,
            theta=self.gabor_theta * math.pi / 180,
            lambd=self.gabor_lambda,
            gamma=self.gabor_gamma
        )

        final_image = cv2.filter2D(src=roi, ddepth=-1, kernel=gabor_kernel)

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "gabor"
    namespace = rospy.get_namespace()

    # create a gabor object
    g = Gabor(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("gabor", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, g.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
