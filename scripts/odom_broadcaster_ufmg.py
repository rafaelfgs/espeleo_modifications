#!/usr/bin/env python

import rospy
import yaml
import tf
from rosgraph_msgs.msg import Clock


class tf_broadcaster:
    
    def __init__(self):
        
        rospy.init_node("odom_broadcaster_node", anonymous=True)
        
        self.load_yaml()
        self.define_tfs()
        
        rospy.Subscriber("/clock", Clock, self.callback)
        
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            rate.sleep()
    
    
    def load_yaml(self):
        
        yaml_file = rospy.get_param("~yaml_file", "")
        f = open(yaml_file)
        self.param = yaml.load(f, Loader=yaml.FullLoader)
    
    
    def define_tfs(self):
        
        self.tfs = [[(0,0,0), (0,0,0,1), "chassis_init", "odom_init"],
                    [(0,0,0), (0,0,0,1), "odom_pose",    "odom_chassis"],
                    ["chassis_base",     "odom_chassis", "odom_base"]]
        
        
        for k in range(len(self.tfs)):
            
            if isinstance(self.tfs[k][0], str):
                
                xyz = tuple(float(x) for x in self.param[self.tfs[k][0]+"_xyz"].split(' '))
                rpy = tuple(float(x) for x in self.param[self.tfs[k][0]+"_rpy"].split(' '))
                quat = tuple(tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'rxyz'))
                parent = self.tfs[k][1]
                child = self.tfs[k][2]
                
                self.tfs[k] = [xyz, quat, parent, child]
    
    
    def callback(self, data):
        
        for k in range(len(self.tfs)):
            msg = tf.TransformBroadcaster()
            msg.sendTransform(self.tfs[k][0], self.tfs[k][1], data.clock, self.tfs[k][3], self.tfs[k][2])


if __name__ == "__main__":
    tf_broadcaster()