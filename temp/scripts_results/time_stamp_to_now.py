#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, PointCloud, PointCloud2
from tf2_msgs.msg import TFMessage


class StampNow:
    
    
    def __init__(self):
        
        rospy.init_node("time_stamp_to_now_node", anonymous=True)
        
        self.topics = filter(None, rospy.get_param("~topics", "").split(" "))
        
        for k in range(len(self.topics)/2):
            topic = (self.topics[2*k+1], eval(self.topics[2*k]))
            rospy.Subscriber(topic[0], topic[1], self.callback, topic)
            rospy.loginfo("Republishing %s", topic[0])
        
    
    
    def callback(self, data, topic):
        
        if topic[1] == TFMessage:
            if abs(data.transforms[0].header.stamp.to_sec() - rospy.Time.now().to_sec()) > 2.0:
                data.transforms[0].header.stamp = rospy.Time.now()
                pub = rospy.Publisher(topic[0], topic[1], queue_size=10)
                pub.publish(data)
        else:
            data.header.stamp = rospy.Time.now()
            pub = rospy.Publisher("/corrected" + topic[0], topic[1], queue_size=10)
            pub.publish(data)


if __name__ == "__main__":
    
    StampNow()
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()