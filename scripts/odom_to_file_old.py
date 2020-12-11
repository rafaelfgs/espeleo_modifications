#!/usr/bin/python

import rospy
import os
from nav_msgs.msg import Odometry


class Odom2File:
    
    
    def __init__(self):
        
        rospy.init_node("odom_to_path_node", anonymous=True)
        
        self.path = ["/home/rafael/Dropbox/UFMG/Results/2011_JINT/average/odom/truth.txt",
                     "/home/rafael/Dropbox/UFMG/Results/2011_JINT/average/odom/wheel.txt",
                     "/home/rafael/Dropbox/UFMG/Results/2011_JINT/average/odom/ekf.txt",
                     "/home/rafael/Dropbox/UFMG/Results/2011_JINT/average/odom/rtab.txt",
                     "/home/rafael/Dropbox/UFMG/Results/2011_JINT/average/odom/lego.txt"]
        
        self.topic = ["/ground_truth",
                      "/odom_new",
                      "/ekf/odom",
                      "/rtabmap/odom_init",
                      "/integrated_to_init2"]
        
        self.file = [None for x in self.topic]
        self.t0   = [ 0.0 for x in self.topic]
        self.num  = [   0 for x in self.topic]
        
        for k in range(len(self.topic)):
            if os.path.exists(self.path[k]): os.remove(self.path[k])
            rospy.Subscriber(self.topic[k], Odometry, self.callback, k)
    
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            rate.sleep()
        
        for k in range(len(self.topic)):
            self.file[k].close()
    
    
    def callback(self, data, k):
        
        self.num[k] += 1
        
        if self.t0[k] == 0.0:
            
            self.t0[k] = data.header.stamp.to_sec()
            
            self.file[k] = open(self.path[k],"w")
            self.file[k].write("n t px py pz qx qy qz qw")
            
            rospy.loginfo("Saving %s to File", self.topic[k])
        
        self.file[k].write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (self.num[k],
                                                                                     data.header.stamp.to_sec() - self.t0[k],
                                                                                     data.pose.pose.position.x,
                                                                                     data.pose.pose.position.y,
                                                                                     data.pose.pose.position.z,
                                                                                     data.pose.pose.orientation.x,
                                                                                     data.pose.pose.orientation.y,
                                                                                     data.pose.pose.orientation.z,
                                                                                     data.pose.pose.orientation.w))


if __name__ == "__main__":
    
    Odom2File()