#!/usr/bin/python
import rospy
import sys
import os
from copy import copy
from nav_msgs.msg import Path



file_name = rospy.get_param("~file_name", "path.txt")
tf_sensor = rospy.get_param("~tf_sensor", "0.07 0 0.25 -0.5 0.5 -0.5 0.5")

tf_sensor = tuple(map(float,tf_sensor.split(" ")))
t0 = 0.0
num = 0
t = []



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



def callback(data):
    
    global t0, num, t, p
    
    if t0 == 0.0:
        t0 = data.header.stamp.to_sec()
    
    if len(data.poses) > num:
        num = len(data.poses)
        t += [data.header.stamp.to_sec() - t0]
        p = data.poses



def save_poses():
    
    for k in range(len(p)):
        
        v_sensor = (p[k].pose.position.x, p[k].pose.position.y, p[k].pose.position.z)
        q_sensor = (p[k].pose.orientation.x, p[k].pose.orientation.y, p[k].pose.orientation.z, p[k].pose.orientation.w)
        
        v_base = qv_mult(q_sensor, qv_mult(q_conjugate(tf_sensor[3:]), v_conjugate(tf_sensor[:3])))
        v_base = qv_mult(tf_sensor[3:], (v_sensor[0]+v_base[0], v_sensor[1]+v_base[1], v_sensor[2]+v_base[2]))
        v_base = (v_base[0]+tf_sensor[0], v_base[1]+tf_sensor[1], v_base[2]+tf_sensor[2])
        q_base = qq_mult(tf_sensor[3:], qq_mult(q_sensor, q_conjugate(tf_sensor[3:])))
        
        f.write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (k+1, t[k], copy(v_base[0]),
                                                                          copy(v_base[1]), copy(v_base[2]),
                                                                          copy(q_base[0]), copy(q_base[1]),
                                                                          copy(q_base[2]), copy(q_base[3])))



def main_function():
    
    global f
    
    if os.path.exists(file_name):
        if raw_input("\nFile already exists. Remove it? (y/n): ") == "y":
            os.remove(file_name)
    
    else:
        
        f = open(file_name,"w")
        f.write("n t px py pz qx qy qz qw")
        
        rospy.init_node("odom_to_file", anonymous=True)
        rospy.Subscriber("/path", Path, callback)
        
        sys.stdout.write("Saving Path to File...\n")
        sys.stdout.flush()
        
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            
        save_poses()
        f.close()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass