#!/usr/bin/env python
import sys
import numpy as np
import cv2
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
# Chicago Engineering Design Team
# Dilate filter using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
# @author Basheer Subei
# @email basheersubei@gmail.com


class Dilate(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000
    dilate_size = 0
    dilate_iterations = 0

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)

        # dilate each pixel using kernel with dilate_size
        if self.dilate_size > 0 and self.dilate_iterations > 0:
            kernel = np.ones((self.dilate_size, self.dilate_size), np.uint8)
            final_image = cv2.dilate(src=roi,
                                     kernel=kernel,
                                     iterations=self.dilate_iterations)
        else:
            rospy.logwarn("dilate parameters invalid! Won't perform dilate!")
            final_image = roi

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "dilate"
    namespace = rospy.get_namespace()

    # create a dilate object
    d = Dilate(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("dilate", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, d.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
