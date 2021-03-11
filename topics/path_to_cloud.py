#!/usr/bin/python

import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import ChannelFloat32, PointCloud


class Path2Cloud:
    
    
    def __init__(self):
        
        rospy.init_node("tf_to_odom_node", anonymous=True)
        
        rospy.Subscriber("/rtabmap/mapPath", Path, self.callback)
        self.pub = rospy.Publisher("/cloud_path", PointCloud, queue_size=10)
    
    
    def callback(self, path):
        
        rospy.loginfo("Republishing Path as PointCloud...")
        
        self.cloud = PointCloud()
        self.cloud.header.frame_id = "rtab_init"
        self.cloud.points = [x.pose.position for x in path.poses]
        self.cloud.channels = [ChannelFloat32()]
        self.cloud.channels[0].name = "intensity"
        self.cloud.channels[0].values = [1.0 for x in path.poses]
        
        self.pub.publish(self.cloud)


if __name__ == "__main__":
    
    Path2Cloud()

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()