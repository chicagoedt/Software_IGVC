#!/usr/bin/env python
import sys
import cv2
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig

###############################################################################
# Chicago Engineering Design Team
# Image Resizer using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
# @author Basheer Subei
# @email basheersubei@gmail.com


class ImageResizer(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is recieved
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)

        if (cv2_image.shape[0] > self.image_height or
                cv2_image.shape[1] > self.image_width):
            final_image = cv2.resize(
                src=cv2_image,
                dsize=(self.image_width, self.image_height),
                fx=0,
                fy=0
            )
        else:
            final_image = cv2_image

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "image_resizer"
    namespace = rospy.get_namespace()

    # create an ImageResizer object
    ir = ImageResizer(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("image_resizer", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, ir.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
