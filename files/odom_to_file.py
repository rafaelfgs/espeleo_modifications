#!/usr/bin/python

import rospy
import os
from copy import copy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class Odom2File:
    
    
    def __init__(self):
        
        rospy.init_node("odom_to_path_node", anonymous=True)
        
        self.path = "/home/rafael/Dropbox/UFMG/Results/2011_JINT/average/odom/"
        
        self.param = [["/ground_truth",        Odometry, (0.00, 0.0, 0.00, 0.0, 0.0, 0.0, 1.0), "truth.txt"],
                      ["/odom_new",            Odometry, (0.00, 0.0, 0.00, 0.0, 0.0, 0.0, 1.0), "wheel.txt"],
                      ["/ekf/odom",            Odometry, (0.00, 0.0, 0.00, 0.0, 0.0, 0.0, 1.0), "ekf.txt"],
                      ["/integrated_to_init2", Odometry, (0.00, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0), "lego.txt"],
                      ["/rtabmap/odom",        Odometry, (0.27, 0.0, 0.07, 0.0, 0.0, 0.0, 1.0), "rtab_odom.txt"],
                      ["/rtabmap/mapPath",     Path,     (0.27, 0.0, 0.07, 0.0, 0.0, 0.0, 1.0), "rtab_path.txt"]]
        
        self.file = [None for x in self.param]
        self.t0   = [ 0.0 for x in self.param]
        self.num  = [   0 for x in self.param]
        self.data = [  [] for x in self.param]
        self.msg  = [  [] for x in self.param]
        self.stop = False
        
        for k in range(len(self.param)):
            rospy.Subscriber(self.param[k][0], self.param[k][1], self.callback, k)
        
        raw_input("Press enter to stop subscribing and save the data (Crtl+C enter to shut down):\n")
        
        self.savefile()
    
    
    def callback(self, msg, k):
        
        if not self.stop:
            
            if self.num[k] == 0:
                self.t0[k] = msg.header.stamp.to_sec()
                print("Reading data from %s" % self.param[k][0])
            
            if self.param[k][1] == Odometry:
                self.msg[k] += [PoseStamped()]
                self.msg[k][self.num[k]].header = copy(msg.header)
                self.msg[k][self.num[k]].pose = copy(msg.pose.pose)
                self.num[k] += 1
            
            elif self.param[k][1] == Path:
                self.msg[k] = copy(msg.poses)
                for n in [self.num[k]+x for x in range(len(msg.poses)-self.num[k])]:
                    self.msg[k][n].header = copy(msg.header)
                self.num[k] = len(msg.poses)
    
    
    def savefile(self):
        
        self.stop = True
        k = 0
        
        while k < len(self.data):
            
            f = self.path + self.param[k][3]
            
            if os.path.exists(f):
                os.remove(f)
            
            self.file[k] = open(f,"w")
            self.file[k].write("n,t,px,py,pz,qx,qy,qz,qw")
            
            if self.num[k] > 0:
                
                print("Saving %s to %s" % (self.param[k][0], self.param[k][3]))
                
                p0 = self.param[k][2][:3]
                q0 = self.param[k][2][3:]
                
                rate = rospy.Rate(1e4)
                n = 0
                
                while not rospy.is_shutdown() and n < self.num[k]:
                    
                    t = self.msg[k][n].header.stamp.to_sec()
                    
                    p1 = (self.msg[k][n].pose.position.x,
                          self.msg[k][n].pose.position.y,
                          self.msg[k][n].pose.position.z)
                    
                    q1 = (self.msg[k][n].pose.orientation.x,
                          self.msg[k][n].pose.orientation.y,
                          self.msg[k][n].pose.orientation.z,
                          self.msg[k][n].pose.orientation.w)
                    
                    q2 = qq_mult(q0, qq_mult(q1, q_conjugate(q0)))
                    p3 = pp_sum(qp_mult(q0, p1), p0)
                    p2 = pp_sum(qp_mult(q2, p_conjugate(p0)), p3)
                    
                    s = "\n%d,%014.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f" % ((n+1,) + (t,) + p2 + q2)
                    self.file[k].write(s)
                    
                    n += 1
                    rate.sleep()
                
            else:
                self.file[k].write("n,t,px,py,pz,qx,qy,qz,qw")
                
            self.file[k].close()
            k += 1


def pp_sum(p1, p2):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x = x1 + x2
    y = y1 + y2
    z = z1 + z2
    return x, y, z

def p_conjugate(p):
    x, y, z = p
    return -x, -y, -z

def qq_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w

def q_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w

def qp_mult(q1, p1):
    q2 = p1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]

if __name__ == "__main__":
    Odom2File()