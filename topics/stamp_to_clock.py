#!/usr/bin/env python
import rospy
import sys
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, PointCloud, PointCloud2


class StampClock:
    
    
    def __init__(self):
        
        rospy.init_node("stamp_to_now_node", anonymous=True)
        topics = filter(None, rospy.get_param("~topics", "").split(" "))
        
        self.bool = False
        rospy.Subscriber("/clock", Clock, self.callback_clock)

        while not self.bool and not rospy.is_shutdown():
            rospy.sleep(0.01)
        
        for k in range(len(topics)/2):
            
            self.topic = (topics[2*k+1], eval(topics[2*k]))
            rospy.Subscriber(self.topic[0], self.topic[1], self.callback_topic)
            rospy.loginfo("Correcting %s", self.topic[0])
    
    
    def callback_clock(self, data):
        self.stamp = data.clock
        self.bool = True
    
    
    def callback_topic(self, data):
        data.header.stamp = self.stamp
        pub = rospy.Publisher("/corrected" + self.topic[0], self.topic[1], queue_size=10)
        pub.publish(data)


if __name__ == "__main__":
    
    StampClock()
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
