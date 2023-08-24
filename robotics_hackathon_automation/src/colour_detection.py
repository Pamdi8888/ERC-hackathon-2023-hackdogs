#!/usr/bin/env python3
#You need to name this node "color_detector"
import rospy
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node("colour_detector")

publisher = rospy.Publisher("/task_status", String, queue_size=10)

from sensor_msgs.msg import Image
import numpy as np


def show_image(im):
    cv2.imshow("Image Window", im)
    cv2.waitKey(3)

bridge = CvBridge()

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError:
        # rospy.logerr("CvBridge Error: {0}".format(e))
        print('error')
    lower_red = np.array([0, 0, 0], dtype = "uint8") 

    upper_red= np.array([255, 50, 255], dtype = "uint8")
    mask = cv2.inRange(cv_image, lower_red, upper_red)
    detected_output = cv2.bitwise_and(cv_image, cv_image, mask =  mask) 
    #
    # cv2.imshow("red color detection", detected_output) 
    avg_color_per_row = np.average(detected_output, axis=1)
    avg_color = np.average(avg_color_per_row, axis=0)
    if avg_color[0] > 1.5:
        publisher.publish("Iron Extraction")
    if avg_color[2] > 1:
        publisher.publish("Zinc Extraction")

    # cv2.waitKey(3)
    # Show the converted image
    # show_image(cv_image)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
print("hi")
sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
