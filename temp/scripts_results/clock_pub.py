#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock

if __name__ == "__main__":

    rospy.init_node("clock_node", anonymous=True)
    pub = rospy.Publisher("/clock", Clock, queue_size=10)

    rate = rospy.Rate(1000)
    
    while not rospy.is_shutdown():
        pub.publish(rospy.Time.now())
        rate.sleep()