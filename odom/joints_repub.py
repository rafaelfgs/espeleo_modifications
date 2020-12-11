#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist


class repub:
    
    
    def __init__(self):
        
        rospy.init_node("joints_node", anonymous=True)
        time_topic = rospy.get_param("~time_topic", "/clock")
        rospy.Subscriber("/cmd_vel", Twist, self.callback_vel)
        rospy.Subscriber("/clock", Clock, self.callback_time)
        
        self.ispub = False
        self.time = 0.0
        self.velocity_right = 0.0
        self.velocity_left = 0.0
        self.forward_orientation = True
        
        self.motor_publisher1 = rospy.Publisher("/device1/get_joint_state", JointState, queue_size=1)
        self.motor_publisher2 = rospy.Publisher("/device2/get_joint_state", JointState, queue_size=1)
        self.motor_publisher3 = rospy.Publisher("/device3/get_joint_state", JointState, queue_size=1)
        self.motor_publisher4 = rospy.Publisher("/device4/get_joint_state", JointState, queue_size=1)
        self.motor_publisher5 = rospy.Publisher("/device5/get_joint_state", JointState, queue_size=1)
        self.motor_publisher6 = rospy.Publisher("/device6/get_joint_state", JointState, queue_size=1)
    
    
    def callback_time(self, data):
        
        self.time = data.clock
        self.ispub = True
    
    
    def callback_vel(self, data):
    
        if self.forward_orientation:
            v = data.linear.x * -1
        else:
            v = data.linear.x
        
        w = data.angular.z
        r = 0.15
        l = 0.30
        
        speed_right = (v/r + w*(l/r)) * 14.5
        speed_left = (v/r - w*(l/r)) * 14.5
        
        self.velocity_right = -speed_right*100
        self.velocity_left = speed_left*100
    
    
    def run(self):
        
        rate = rospy.Rate(100)
        while not self.ispub:
            rate.sleep()
        
        while not rospy.is_shutdown():
            
            state_right = JointState()
            state_right.header.stamp = self.time
            state_right.velocity = [self.velocity_right]
            
            state_left = JointState()
            state_left.header.stamp = self.time
            state_left.velocity = [self.velocity_left]
            
            self.motor_publisher1.publish(state_right)
            self.motor_publisher2.publish(state_right)
            self.motor_publisher3.publish(state_right)
            self.motor_publisher4.publish(state_left)
            self.motor_publisher5.publish(state_left)
            self.motor_publisher6.publish(state_left)
            
            rate.sleep()
    

if __name__ == "__main__":
    try:
        node = repub()
        node.run()
    except rospy.ROSInterruptException:
        pass
