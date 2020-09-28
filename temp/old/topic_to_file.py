#!/usr/bin/env python

import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


message = Image()


def callback(data):
    global message
    message = data


def topic_to_file():

    topic_name = sys.argv[1]
    file_name = sys.argv[2]
    count_max = int(sys.argv[3])

    rospy.init_node('topic_to_file_node', anonymous=True)
    rospy.Subscriber(topic_name, Image, callback)
    rate = rospy.Rate(30)

    sys.stdout.write("\nWaiting a publishing on %s" % topic_name)
    while message.header.seq == 0 and not rospy.is_shutdown():
       rate.sleep()
    sys.stdout.write(" Ok!")

    bridge = CvBridge()

    count = 1

    while count <= count_max and not rospy.is_shutdown():

        image = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
    
        image_flat = [x for sets in list(image) for x in sets]

        sys.stdout.write("\nWriting in file %s  " % file_name)
        with open(file_name, "w") as file_var:
            for x in range(0,len(image_flat)):
                file_var.write("%f\t" % image_flat[x])
        sys.stdout.write("Ok!\n\n")

        count += 1

        rate.sleep()


if __name__ == '__main__':
    try:
        topic_to_file()
    except rospy.ROSInterruptException:
        pass
