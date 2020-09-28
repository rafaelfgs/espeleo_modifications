#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import CompressedImage, Image, PointCloud2

def callback(msg):
    global is_published
    is_published = True
    if hasattr(msg,'data'): msg.data = []
    if hasattr(msg,'fields'): msg.fields = []
    print msg, '\n'

def main_function():

    rospy.init_node('print_info_node', anonymous=True)
    if sys.argv[2] == 'Image':
        rospy.Subscriber(sys.argv[1], Image, callback)
    if sys.argv[2] == 'PointCloud2':
        rospy.Subscriber(sys.argv[1], PointCloud2, callback)
    if sys.argv[2] == 'CompressedImage':
        rospy.Subscriber(sys.argv[1], CompressedImage, callback)

    is_published = False
    rate = rospy.Rate(1000)

    print "Waiting a publishing on %s" % sys.argv[1]
    while not is_published and not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
