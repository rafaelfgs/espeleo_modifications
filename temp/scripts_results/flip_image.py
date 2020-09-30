#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


bridge = CvBridge()
is_published = [False,]


def callback_img(data):
    
    global is_published
    is_published[0] = True
    
    encoding = data.encoding
    
    flip = bridge.imgmsg_to_cv2(data, desired_encoding=encoding)
    img = cv2.flip(flip,0)
    
    msg = bridge.cv2_to_imgmsg(img, encoding=encoding)
    msg.header = data.header
    
    pub = rospy.Publisher("/image_raw", Image, queue_size=10)
    pub.publish(msg)


def main_function():
    
    rospy.init_node("flip_image_node", anonymous=True)
    rospy.Subscriber("/image_raw_flipped", Image, callback_img)
    
    rate = rospy.Rate(10)
    
    while not all(is_published) and not rospy.is_shutdown():
        rate.sleep()
    
    rospy.loginfo("Flipping Image...")
    
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass