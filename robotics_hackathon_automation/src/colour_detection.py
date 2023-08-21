#!/usr/bin/env python3
#You need to name this node "color_detector"
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node("color_detector")


from sensor_msgs.msg import Image


# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
