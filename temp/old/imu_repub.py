#!/usr/bin/env python

import rospy
from copy import copy
from sensor_msgs.msg import Imu

def callback(data, frame_id):
    msg = copy(data)
    msg.header.frame_id = frame_id
    pub = rospy.Publisher("/imu/data2", Imu, queue_size=10)
    pub.publish(msg)

if __name__ == "__main__":

    rospy.init_node("imu_repub_node", anonymous=True)
    frame_id = rospy.get_param("~frame_id", "imu_link")
    
    rospy.Subscriber("/imu/data", Imu, callback, frame_id)
    rospy.loginfo("Republishing Imu topic...")
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()