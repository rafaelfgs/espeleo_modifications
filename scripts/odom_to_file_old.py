#!/usr/bin/python

import rospy
import os
from copy import copy
from nav_msgs.msg import Odometry



truth_file = "/home/rafael/Dropbox/UFMG/Artigos/lars2020_2/matlab/truth.txt"
rtab_file  = "/home/rafael/Dropbox/UFMG/Artigos/lars2020_2/matlab/rtabmap.txt"
ekf_file   = "/home/rafael/Dropbox/UFMG/Artigos/lars2020_2/matlab/ekf.txt"
lego_file  = "/home/rafael/Dropbox/UFMG/Artigos/lars2020_2/matlab/lego.txt"



t0_truth,  t0_rtab,  t0_ekf  = 0.0, 0.0, 0.0
num_truth, num_rtab, num_ekf = 0,   0,   0



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

def v_conjugate(v):
    x, y, z, = v
    return -x, -y, -z

def qv_mult(q1, v1):
    q2 = v1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]



def callback_truth(data):
    
    global num_truth, t0_truth
    
    num_truth += 1
    if t0_truth == 0.0:
        t0_truth = data.header.stamp.to_sec()
    
    f_truth.write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (num_truth,
                                                                            data.header.stamp.to_sec() - t0_truth,
                                                                            copy(data.pose.pose.position.x),
                                                                            copy(data.pose.pose.position.y),
                                                                            copy(data.pose.pose.position.z),
                                                                            copy(data.pose.pose.orientation.x),
                                                                            copy(data.pose.pose.orientation.y),
                                                                            copy(data.pose.pose.orientation.z),
                                                                            copy(data.pose.pose.orientation.w)))

def callback_rtab(data):
    
    global num_rtab, t0_rtab
    
    num_rtab += 1
    if t0_rtab == 0.0:
        t0_rtab = data.header.stamp.to_sec()
    
    f_rtab.write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (num_rtab,
                                                                           data.header.stamp.to_sec() - t0_rtab,
                                                                           copy(data.pose.pose.position.x),
                                                                           copy(data.pose.pose.position.y),
                                                                           copy(data.pose.pose.position.z),
                                                                           copy(data.pose.pose.orientation.x),
                                                                           copy(data.pose.pose.orientation.y),
                                                                           copy(data.pose.pose.orientation.z),
                                                                           copy(data.pose.pose.orientation.w)))

def callback_ekf(data):
    
    global num_ekf, t0_ekf
    
    num_ekf += 1
    if t0_ekf == 0.0:
        t0_ekf = data.header.stamp.to_sec()
    
    f_ekf.write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (num_ekf,
                                                                          data.header.stamp.to_sec() - t0_ekf,
                                                                          copy(data.pose.pose.position.x),
                                                                          copy(data.pose.pose.position.y),
                                                                          copy(data.pose.pose.position.z),
                                                                          copy(data.pose.pose.orientation.x),
                                                                          copy(data.pose.pose.orientation.y),
                                                                          copy(data.pose.pose.orientation.z),
                                                                          copy(data.pose.pose.orientation.w)))



def main_function():
    
    global f_truth, f_rtab, f_ekf
    
    rospy.init_node("odom_to_file_node", anonymous=True)
    
    if os.path.exists(truth_file): os.remove(truth_file)
    if os.path.exists(rtab_file):  os.remove(rtab_file)
    if os.path.exists(ekf_file):   os.remove(ekf_file)
    
    
    f_truth = open(truth_file,"w")
    f_rtab  = open(rtab_file,"w")
    f_ekf   = open(ekf_file,"w")
    
    f_truth.write("n t px py pz qx qy qz qw")
    f_rtab.write("n t px py pz qx qy qz qw")
    f_ekf.write("n t px py pz qx qy qz qw")
    
    rospy.Subscriber("/truth/odom",   Odometry, callback_truth)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_rtab)
    rospy.Subscriber("/ekf/odom",     Odometry, callback_ekf)
    
    rospy.loginfo("Saving Odometry to Files...")
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
        
    f_truth.close()
    f_rtab.close()
    f_ekf.close()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
