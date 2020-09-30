#!/usr/bin/env python

import rospy
from math import sin
from copy import copy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


topic_name = rospy.get_param("topic_out", "/espeleo_pose_ekf/odom_combined")


x = [0,0]
y = [0,0]
z = [0,0]
roll  = [0,0]
pitch = [0,0]
yaw   = [0,0]


def callback(data):
    
    global x, y, z, roll, pitch, yaw
    
    euler = euler_from_quaternion([data.pose.pose.orientation.x,
                                   data.pose.pose.orientation.y,
                                   data.pose.pose.orientation.z,
                                   data.pose.pose.orientation.w])
    
    x = [x[1], copy(data.pose.pose.position.x)]
    y = [y[1], copy(data.pose.pose.position.y)]
    
    roll  = [roll[1],  euler[0]]
    pitch = [pitch[1], euler[1]]
    yaw   = [yaw[1],   euler[2]]
    
    dz = ( (x[1]-x[0])**2 + (y[1]-y[0])**2 )**0.5 * sin( (-pitch[0]-pitch[1])/2 )
    z = [z[1], z[1]+dz]
    
    odom = Odometry()
    odom.header = data.header
    odom.child_frame_id = data.child_frame_id
    odom.pose.pose.position.x = x[1]
    odom.pose.pose.position.y = y[1]
    odom.pose.pose.position.z = z[1]
    odom.pose.pose.orientation.x = copy(data.pose.pose.orientation.x)
    odom.pose.pose.orientation.y = copy(data.pose.pose.orientation.y)
    odom.pose.pose.orientation.z = copy(data.pose.pose.orientation.z)
    odom.pose.pose.orientation.w = copy(data.pose.pose.orientation.w)
    odom.pose.covariance = data.pose.covariance
    odom.twist.twist.linear.x = x[1] - x[0]
    odom.twist.twist.linear.y = y[1] - y[0]
    odom.twist.twist.linear.z = z[1] - z[0]
    odom.twist.twist.angular.x = roll[1]  - roll[0]
    odom.twist.twist.angular.y = pitch[1] - pitch[0]
    odom.twist.twist.angular.z = yaw[1]   - yaw[0]
    odom.twist.covariance = data.twist.covariance
    
    pub = rospy.Publisher(topic_name + "_3D", Odometry, queue_size=10)
    pub.publish(odom)


def main_function():

    rospy.init_node("pitch_to_z_axis_node", anonymous=True)
    rospy.Subscriber(topic_name, Odometry, callback)
    
    rospy.loginfo("Republishing " + topic_name + "_3D")
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass