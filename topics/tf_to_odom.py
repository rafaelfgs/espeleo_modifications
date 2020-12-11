#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage



class CloudAggregator:
    
    
    
    def __init__(self):
        
        rospy.init_node("tf_to_odom_node", anonymous=True)
        
        self.map = rospy.get_param("~map", "base_init")
        self.odom = rospy.get_param("~odom", "truth_link")
        
        self.topic = rospy.get_param("~topic", "/ground_truth")
        self.freq = float(rospy.get_param("~freq", "1.0"))
        
        self.stamp = rospy.Time.from_sec(0.0)
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        
        rospy.Subscriber("/tf", TFMessage, self.callback)
        pub = rospy.Publisher(self.topic, Odometry, queue_size=10)
        
        rospy.loginfo("Publishing Odometry in %s at %.1fHz from %s to %s", self.topic, self.freq, self.map, self.odom)
        
        k = 0
        rate = rospy.Rate(self.freq)
        
        while not rospy.is_shutdown():
            
            msg = Odometry()
            msg.header.seq = k
            msg.header.stamp = self.stamp
            msg.header.frame_id = self.map
            msg.child_frame_id = self.odom
            msg.pose.pose.position.x = self.position[0]
            msg.pose.pose.position.y = self.position[1]
            msg.pose.pose.position.z = self.position[2]
            msg.pose.pose.orientation.x = self.orientation[0]
            msg.pose.pose.orientation.y = self.orientation[1]
            msg.pose.pose.orientation.z = self.orientation[2]
            msg.pose.pose.orientation.w = self.orientation[3]
            
            pub.publish(msg)
            
            rate.sleep()
    
    
    
    def callback(self, msg):
        
        if msg.transforms[0].header.frame_id == self.map and msg.transforms[0].child_frame_id == self.odom:
            
            self.stamp = msg.transforms[0].header.stamp
            self.position = [msg.transforms[0].transform.translation.x,
                             msg.transforms[0].transform.translation.y,
                             msg.transforms[0].transform.translation.z]
            self.orientation = [msg.transforms[0].transform.rotation.x,
                                msg.transforms[0].transform.rotation.y,
                                msg.transforms[0].transform.rotation.z,
                                msg.transforms[0].transform.rotation.w]



if __name__ == "__main__":
    CloudAggregator()