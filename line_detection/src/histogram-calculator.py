#!/usr/bin/env python
import sys
import numpy as np
import cv2
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig
from sensor_msgs.msg import CompressedImage

###############################################################################
# Chicago Engineering Design Team
# Histogram Calculator node
#
# This ROS node reads image rosbags and sums up the
# HSV histograms, and writes it (every frame) to a text file.
#
# The histogram can then be used as input to the histogram backprojection node.
# (IGVC competition).
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class HistogramCalculator(LaneDetection):
    roi_top_left_x = 16
    roi_top_left_y = 143
    roi_width = 2000
    roi_height = 2000

    cumulative_histogram = np.zeros((180, 256), dtype=float)
    # holds total number of frames we processed already
    frames_processed = 0

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)
        # set hsv histogram plot ROS image publisher
        self.current_histogram_plot_pub = rospy.Publisher(
            '/histogram_calculator/current_histogram_plot/compressed',
            CompressedImage,
            queue_size=1
        )
        # set cumulative histogram ROS image publisher
        self.cumulative_histogram_plot_pub = rospy.Publisher(
            '/histogram_calculator/cumulative_histogram_plot/compressed',
            CompressedImage,
            queue_size=1
        )

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)
        roi = LaneDetection.get_roi(self, cv2_image)
        # convert from BGR to HSV
        hsv = cv2.cvtColor(src=roi, code=cv2.COLOR_BGR2HSV)

        histogram = cv2.calcHist(
            images=[hsv],
            channels=[0, 1],
            mask=None,
            histSize=[180, 256],
            ranges=[0, 179, 0, 255]
        )

        # publishes current histogram plot image
        histogram_message = LaneDetection.cv2_to_ros_message(
            self, histogram
        )
        self.current_histogram_plot_pub.publish(histogram_message)

        # now calculate cumulative histogram
        # just add the current histogram to the cumulative histogram
        self.cumulative_histogram += histogram
        # publish cumulative histogram
        cumulative_message = LaneDetection.cv2_to_ros_message(
            self, self.cumulative_histogram
        )
        self.cumulative_histogram_plot_pub.publish(cumulative_message)

        # now write the new cumulative histogram data to a file
        np.savetxt(
            self.package_path + "/misc/training_images/histogram.txt",
            self.cumulative_histogram
        )

        # now publish roi image for debugging
        roi_message = LaneDetection.cv2_to_ros_message(
            self, roi
        )
        self.line_image_pub.publish(roi_message)

        # increment number of frames since this frame is done
        self.frames_processed += 1

    # end image_callback()


def main(args):
    node_name = "histogram_calculator"
    namespace = rospy.get_namespace()

    # create a HistogramCalculator object
    hc = HistogramCalculator(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("histogram_calculator", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, hc.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
