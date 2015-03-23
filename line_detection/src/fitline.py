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
# Fitline filter using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
# @author Basheer Subei
# @email basheersubei@gmail.com


class Fitline(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)

        # make sure the image is mono
        assert self.use_mono

        # extracts nonzero pixels and formats them as separate points
        # (each point is a vector of two elements, x and y)
        points = np.float32(np.transpose(roi.nonzero()))

        # code from http://stackoverflow.com/a/14192660/341505
        # then apply fitline() function
        [vx, vy, x, y] = cv2.fitLine(
            points=points,
            distType=cv2.cv.CV_DIST_L2,
            param=0,
            reps=0.001,
            aeps=0.001
        )

        # Now find two extreme points on the line to draw line
        lefty = int((-x * vy / vx) + y)
        righty = int(((roi.shape[1] - x) * vy / vx) + y)

        # Finally draw the line
        cv2.line(
            img=roi,
            pt1=(roi.shape[1] - 1, righty),
            pt2=(0, lefty),
            color=255,
            thickness=2
        )

        final_image = roi

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "fitline"
    namespace = rospy.get_namespace()

    # create a fitline object
    f = Fitline(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("fitline", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, f.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
