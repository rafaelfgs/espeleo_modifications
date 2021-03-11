#!/usr/bin/python

import rospy
import sys
from math import pi
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def main_function():
    global raw_reg, raw_rtab, tf_reg, tf_rtab, data_reg, data_rtab
    
    rospy.init_node("file_to_path_node", anonymous=True)
    
    frame  = rospy.get_param("~frame", "base_init")
    freq   = float(rospy.get_param("~freq", "1.0"))
    
    folder = rospy.get_param("~folder", "cave2")
    
    if folder == "cave2":
        tf_reg  = rospy.get_param("~tf", "-0.728520 -1.022489 -0.591337 -0.0067226 0.0021616 0.008038 0.9999428")
        tf_rtab = rospy.get_param("~tf", "0.171135 0.731557 -0.011839 -0.0014095 0.0076989 -0.015365 0.9998513")
    
    elif folder == "cave3":
        tf_reg  = rospy.get_param("~tf", "-0.050356 0.257231 -0.010462 -0.0011115 0.0012195 -0.0049853 0.9999862")
        tf_rtab = rospy.get_param("~tf", "0.044481 -0.017968 0.077054 0.0004048 0.0001445 -0.0024175 0.999997")
        
    elif folder == "veloso1":
        tf_reg  = rospy.get_param("~tf", "0.017721 0.234458 0.303212 0.0268289 0.0005092 0.001487 0.9996388")
        tf_rtab = rospy.get_param("~tf", "0.017516 0.094458 0.430594 0.0031193 0.0218101 0.0021255 0.999755")
        
    elif folder == "veloso2":
        tf_reg  = rospy.get_param("~tf", "-0.449136 0.095166 0.020098 -0.0287285 -0.0005158 0.0179488 0.999426")
        tf_rtab = rospy.get_param("~tf", "-0.096254 0.383585 -0.114160 0.0019417 -0.0077941 -0.02566 0.9996385")
    
    tf_reg  = tuple([float(x) for x in tf_reg.split(" ")[:-1]])  + (float(tf_reg.split(" ")[-1]),)
    tf_rtab = tuple([float(x) for x in tf_rtab.split(" ")[:-1]]) + (float(tf_rtab.split(" ")[-1]),)
    
    file_reg  = open("/mnt/WD500/UFMG/DISSERTACAO/results/" + folder + "/reg_map.txt",  "r")
    file_rtab = open("/mnt/WD500/UFMG/DISSERTACAO/results/" + folder + "/rtab_map.txt", "r")
    
    raw_reg  = [tuple([float(y) for y in x.split(',')[2:]]) for x in file_reg.read().splitlines()[1:]]
    raw_rtab = [tuple([float(y) for y in x.split(',')[2:]]) for x in file_rtab.read().splitlines()[1:]]
    
    data_reg  = [pp_sum(qp_mult(tf_reg[3:], x[:3]),tf_reg[:3])  + qq_mult(tf_reg[3:], x[3:]) for x in raw_reg]
    data_rtab = [pp_sum(qp_mult(tf_rtab[3:],x[:3]),tf_rtab[:3]) + qq_mult(tf_rtab[3:],x[3:]) for x in raw_rtab]
    
    msg_reg  = Path()
    msg_rtab = Path()
    
    msg_reg.poses  = [list_to_posestamped(x) for x in data_reg]
    msg_rtab.poses = [list_to_posestamped(x) for x in data_rtab]
    
    pub_reg  = rospy.Publisher("/reg/path",  Path, queue_size=10)
    pub_rtab = rospy.Publisher("/rtab/path", Path, queue_size=10)
    
    sys.stdout.write("Publishing reg and rtab in %s\n" % folder)
    sys.stdout.flush()
    
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        
        msg_reg.header.seq += 1
        msg_reg.header.stamp = rospy.Time.now()
        msg_reg.header.frame_id = frame
        
        msg_rtab.header.seq += 1
        msg_rtab.header.stamp = rospy.Time.now()
        msg_rtab.header.frame_id = frame
        
        pub_reg.publish(msg_reg)
        pub_rtab.publish(msg_rtab)
        
        rate.sleep()


def rpy_to_quat(data):
    r = pi/180 * data[0]
    p = pi/180 * data[1]
    y = pi/180 * data[2]
    return tuple(quaternion_from_euler(r,p,y))
    
    
    
def list_to_posestamped(data):
    msg = PoseStamped()
    msg.pose.position.x = data[0]
    msg.pose.position.y = data[1]
    msg.pose.position.z = data[2]
    msg.pose.orientation.x = data[3]
    msg.pose.orientation.y = data[4]
    msg.pose.orientation.z = data[5]
    msg.pose.orientation.w = data[6]
    return msg


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

def pp_sum(p1, p2):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    return x1 + x2, y1 + y2, z1 + z2

def p_conjugate(p):
    x, y, z = p
    return -x, -y, -z


if __name__ == "__main__":
    main_function()