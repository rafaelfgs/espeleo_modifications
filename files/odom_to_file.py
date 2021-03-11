#!/usr/bin/python

import rospy
import time
import sys
import os
from copy import copy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class Odom2File:
    
    
    def __init__(self):
        
        rospy.init_node("odom_to_path_node", anonymous=True)
        
        self.path = "/mnt/WD500/UFMG/DISSERTACAO/results/veloso2/odom_reg5/"
        
        self.param = [["/ground_truth",        Odometry, (0.00, 0.0, 0.00, 0.0, 0.0, 0.0, 1.0), "truth.txt"],
                      ["/odom",                Odometry, (0.00, 0.0, 0.00, 0.0, 0.0, 0.0, 1.0), "wheel.txt"],
                      ["/ekf/odom",            Odometry, (0.00, 0.0, 0.00, 0.0, 0.0, 0.0, 1.0), "ekf.txt"],
                      ["/t265/odom/sample",    Odometry, (0.00, 0.0, 0.00, 0.0, 0.0, 0.0, 1.0), "t265.txt"],
                      ["/integrated_to_init2", Odometry, (0.00, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0), "lego.txt"],
                      ["/rtabmap/odom",        Odometry, (0.27, 0.0, 0.07, 0.0, 0.0, 0.0, 1.0), "rtab_odom.txt"],
                      ["/rtabmap/mapPath",     Path,     (0.27, 0.0, 0.07, 0.0, 0.0, 0.0, 1.0), "rtab_path.txt"]]
        
        self.file  = [None for x in self.param]
        self.num   = [   0 for x in self.param]
        self.stamp = [  [] for x in self.param]
        self.msg   = [  [] for x in self.param]
        self.t0    = 0.0
        self.stop  = False
        
        for k in range(len(self.param)):
            rospy.Subscriber(self.param[k][0], self.param[k][1], self.callback, k)
        
        raw_input("Press enter to stop subscribing and save the data (Crtl+C enter to shut down):\n")
        
        if not rospy.is_shutdown():
            self.savefile()
    
    
    def callback(self, msg, k):
        
        if not self.stop:
            
            if self.t0 == 0:
                self.t0 = msg.header.stamp.to_sec()
            
            if self.num[k] == 0:
                sys.stdout.write("\nReading data from %s" % self.param[k][0])
                sys.stdout.flush()
            
            if self.param[k][1] == Odometry:
                self.stamp[k] += [msg.header.stamp.to_sec() - self.t0]
                self.msg[k] += [PoseStamped()]
                self.msg[k][-1].pose = copy(msg.pose.pose)
                self.num[k] += 1
            
            elif self.param[k][1] == Path:
                if len(self.msg[k]) < len(msg.poses):
                    self.stamp[k] += [msg.header.stamp.to_sec() - self.t0]
                else:
                    self.stamp[k][-1] = msg.header.stamp.to_sec()
                self.msg[k] = copy(msg.poses)
                self.num[k] = len(msg.poses)
    
    
    def savefile(self):
        
        self.stop = True
        
        if not os.path.exists(self.path):
            os.makedirs(self.path)
            
        a = []
        for k in range(len(self.param)):
            a += [os.path.exists(self.path + self.param[k][3])]
        if any(a):
            if raw_input("\nOutput files exist, replace them? (y/n): ") == "y":
                for k in range(len(self.param)):
                    if a[k]: os.remove(self.path + self.param[k][3])
            elif raw_input("\nSave them in another folder? (y/n): ") == "y":
                self.path = raw_input("")
            else:
                sys.stdout.write("\nNo changes made.\n\n")
                sys.stdout.flush()
                sys.exit()
        
        for k in range(len(self.param)):
            
            self.file[k] = open(self.path + self.param[k][3],"w")
            self.file[k].write("n,t,px,py,pz,qx,qy,qz,qw")
            
            sys.stdout.write("\nSaving %s to %s" % (self.param[k][0], self.param[k][3]))
            sys.stdout.flush()
            
            p0 = self.param[k][2][:3]
            q0 = self.param[k][2][3:]
            
            if self.num[k] == 0: self.file[k].write("\n1,0,0,0,0,0,0,0,1")
            
            n = 0
            while n < self.num[k]:
                
                t = self.stamp[k][n]
                
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
                
                s = "\n%d,%.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f" % ((n+1,) + (t,) + p2 + q2)
                self.file[k].write(s)
                
                n += 1
                time.sleep(1e-6)
                
            self.file[k].close()
        
        sys.stdout.write("\n\n")
        sys.stdout.flush()
        sys.exit()
                

def pp_sum(p1, p2):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    return x1 + x2, y1 + y2, z1 + z2

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