#!/usr/bin/env python

import rospy
from copy import copy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


topic_in = rospy.get_param("topic_in", "/espeleo_pose_ekf/pose_combined")
topic_out = rospy.get_param("topic_out", "/espeleo_pose_ekf/odom_combined")


odom = Odometry()
odom.header.frame_id = rospy.get_param("frame_id", "wheel_ekf_init")
odom.child_frame_id = rospy.get_param("child_frame_id", "wheel_ekf_odom")


def callback(data):
    
    odom.header.seq = copy(data.header.seq)
    odom.header.stamp = copy(data.header.stamp)
    odom.pose = copy(data.pose)
    
    pub = rospy.Publisher(topic_out, Odometry, queue_size=10)
    pub.publish(odom)


def main_function():

    rospy.init_node("pose_to_odom_node", anonymous=True)
    rospy.Subscriber(topic_in, PoseWithCovarianceStamped, callback)
    
    rospy.loginfo("Republishing " + topic_in + " as " + topic_out)
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass