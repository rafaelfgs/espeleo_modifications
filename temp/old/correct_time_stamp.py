#!/usr/bin/env python

import rospy
import numpy
import sys
from copy import copy
from sensor_msgs.msg import Imu

curr_time = rospy.Time.from_sec(0.0)

def callback_old(data):
    global curr_time
    curr_time = data.header.stamp

def callback_new(data):
    pub = rospy.Publisher("/filter/imu/data_old", Imu, queue_size=1)
    data_old = copy(data)
    data_old.header.stamp = curr_time
    pub.publish(data_old)

def main_function():
    
    rospy.init_node("correct_time_stamp_node", anonymous=True)
    rospy.Subscriber("/d435i/gyro/sample", Imu, callback_old)
    rospy.Subscriber("/filter/imu/data", Imu, callback_new)
    rate = rospy.Rate(1000)
    
    sys.stdout.write('\nWaiting for publishing...\n')
    while curr_time == rospy.Time.from_sec(0.0):
        pass
    
    sys.stdout.write('\nCorrecting time stamp of Filter IMU\n')
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
