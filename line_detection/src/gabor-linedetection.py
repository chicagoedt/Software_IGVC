#!/usr/bin/env python
import sys
import numpy as np
import cv2
import math
#import time
import rospy
import sensor_msgs
from sensor_msgs.msg import CompressedImage
import rospkg
from dynamic_reconfigure.server import Server
from line_detection.cfg import LineDetectionConfig

###############################################################################
## Chicago Engineering Design Team
## Line Detection Example using Python OpenCV for autonomous robot Scipio
##    (IGVC competition).
## @author Basheer Subei
## @email basheersubei@gmail.com
#######################################################
##
## Gabor Filter example
##
###############################################################################


class line_detection:

    # this is where we define our variables in the class.
    # these are changed dynamically using dynamic_reconfig and affect
    # the image processing algorithm. A lot of these are not used in the
    # current algorithm.
    blur_size = 49

    # hsv threshold variables
    hue_low = 20
    hue_high = 50

    saturation_low = 0
    saturation_high = 255

    value_low = 0
    value_high = 255

    backprojection_threshold = 50

    # gabor filter parameters
    gabor_ksize = 4
    gabor_sigma = 7
    gabor_theta = 0
    gabor_lambd = 27
    gabor_gamma = 4

    training_file_name = 'training_for_backprojection_1.png'
    package_path = ''

    def __init__(self):

        # initialize ROS stuff

        # set publisher and subscriber

        # publisher for pointcloud data.
        # the code for this is not implemented yet.
        # self.line_pub = rospy.Publisher(
        #    'line_data',
        #    sensor_msgs.msg.PointCloud2)

        # publisher for image of line pixels (only for debugging, not used in
        # map)
        self.line_image_pub = rospy.Publisher('line_image/compressed',
                                              sensor_msgs.msg.CompressedImage)

        # subscriber for ROS image topic
        self.image_sub = rospy.Subscriber("/camera/image_raw/compressed",
                                          CompressedImage, self.image_callback,
                                          queue_size=1)

        # this returns the path to the current package
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('line_detection')

        # use this for uncompressed raw format
        # self.image_sub = rospy.Subscriber("/camera/image_raw", Image,
        #                                    self.image_callback, queue_size=1)
        
        # use this if you need to use the camera_info topic (has intrinsic
        #                                                     parameters)
        # self.camera_info_sub = rospy.Subscriber("/camera/camera_info",
        #                                          CameraInfo,
        #                                          self.camera_info_callback,
        #                                          queue_size=1 )
    
    # returns packprojection image as OpenCV image in HSV format
    def get_backprojection_training_image(self, filename):
        
        training_file_path = (self.package_path
                              + '/misc/training_images/'
                              + filename)
        # reads in the backprojection training image from file
        backprojection_training = cv2.imread(training_file_path)
        # converts it from BGR to HSV
        backprojection_training = cv2.cvtColor(backprojection_training,
                                               cv2.COLOR_BGR2HSV)
        return backprojection_training

    # returns mask based on backprojection
    def get_backprojection_mask(self, image, filename):

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # make HSV histogram and look for regions with lots of green Hue. For
        # those regions find their range of HSV so that they can be masked out.
        # Grass Hue ranges from 20:50 , Saturation ranges from 50:125 , Value
        # ranges from 0:150.
        # White lines Hue ranges from 120:150 , Saturation ranges from 0:50 ,
        # Value ranges from 150:255.
        # all above values are in OpenCV HSV ranges

        # note that Values depend on overall brightness (need to use adaptive
        # method or dynamic one).

        backprojection_training = self.get_backprojection_training_image(
            filename)
        
        ## begin HISTOGRAM BACKPROJECTION
        # calculating object histogram
        roihist = cv2.calcHist([backprojection_training],
                               [0, 1],
                               None,
                               [180, 256],
                               [self.hue_low,
                                self.hue_high,
                                0,
                                256]
                               )

        # normalize histogram and apply backprojection
        cv2.normalize(roihist, roihist, 1, 255, cv2.NORM_MINMAX)

        dst = cv2.calcBackProject([hsv], [0, 1], roihist,
                                  [self.hue_low,
                                   self.hue_high,
                                   0,
                                   256],
                                  1)

        # Now convolute with circular disc
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cv2.filter2D(dst, -1, disc, dst)  # do we need this convolution???

        # invert dst (because the backprojection chooses what we DON'T want)
        dst = 255 - dst

        return dst
        ## end HISTOGRAM BACKPROJECTION

    # this is what gets called when an image is recieved
    def image_callback(self, image):

        # use this to record start time for each frame
        #start_time = time.time()

        #### direct conversion from ROS Image to CV2 ####
        np_arr = np.fromstring(image.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        # assuming our region of interest is only in bottom half of image
        horizon = img.shape[0] / 2
        roi = img[horizon:, :]  # refer to how Python uses colons in lists

        # first remove grass (backprojection)
        # then blur out smaller dots (low-pass filter using median blur)
        # finally, apply a Gabor filter to find edge

        # run backprojection to remove grass
        backprojection_image =\
            self.get_backprojection_mask(roi, self.training_file_name)

        # threshold the backprojection to only grab the more probable ones
        ret, thresh = cv2.threshold(backprojection_image,
                                    self.backprojection_threshold,
                                    0,
                                    cv2.THRESH_TOZERO)

        # no need to convert to grayscale because
        # cv2.threshold already does that

        gray_roi = cv2.medianBlur(thresh, self.blur_size)

        # apply Gabor filter
        gabor_kernel = cv2.getGaborKernel((self.gabor_ksize, self.gabor_ksize),
                                          self.gabor_sigma,
                                          self.gabor_theta * math.pi / 180,
                                          self.gabor_lambd,
                                          self.gabor_gamma)
        final_image = cv2.filter2D(gray_roi, -1, gabor_kernel)
        
        #### Create CompressedImage to publish ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            final_image)[1]).tostring()

        # publishes image message with line pixels in it
        self.line_image_pub.publish(final_image_message)

    ## end image_callback()

    def reconfigure_callback(self, config, level):

        # TODO check if the keys exist in the config dictionary or else error

        self.blur_size = config['blur_size']
        self.hue_low = config['hue_low']
        self.hue_high = config['hue_high']
        self.saturation_low = config['saturation_low']
        self.saturation_high = config['saturation_high']
        self.value_low = config['value_low']
        self.value_high = config['value_high']
        self.backprojection_threshold = config['backprojection_threshold']
        self.training_file_name = config['training_file_name']

        # gabor filter parameters
        self.gabor_ksize = config['gabor_ksize']
        self.gabor_sigma = config['gabor_sigma']
        self.gabor_theta = config['gabor_theta']
        self.gabor_lambd = config['gabor_lambd']
        self.gabor_gamma = config['gabor_gamma']

        self.validate_parameters()

        return config

    # makes sure the parameters are valid and don't crash the
    # openCV calls. Changes them to valid values if invalid.
    def validate_parameters(self):
        
        # these parameters need validation:

        # blur_size can be an odd number only
        if self.blur_size % 2 == 0:
            self.blur_size -= 1

        # hue, saturation, and value parameters cannot have
        # larger or equal low limits than high limits
        if self.hue_low >= self.hue_high:
            self.hue_low = self.hue_high - 1
        if self.saturation_low >= self.saturation_high:
            self.saturation_low = self.saturation_high - 1
        if self.value_low >= self.value_high:
            self.value_low = self.value_high - 1

        # gabor filter parameters don't need validation

def main(args):
    # create a line_detection object
    ld = line_detection()

    # start the line_detector node and start listening
    rospy.init_node('generic_linedetection')
    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, ld.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
