#!/usr/bin/python

import rospy
import sys
from math import pi
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def main_function():
    global raw, tf, data
    
    rospy.init_node("file_to_path_node", anonymous=True)
    
    frame  = rospy.get_param("~frame", "base_init")
    freq   = float(rospy.get_param("~freq", "1.0"))
    
    path = open(rospy.get_param("~path", ""),  "r")

    tf = rospy.get_param("~tf", "0.0 0.0 0.0 0.0 0.0 0.0 1.0")
    
    tf = tuple([float(x) for x in tf.split(" ")[:-1]]) + (float(tf.split(" ")[-1]),)
    
    raw = [tuple([float(y) for y in x.split(',')[2:]]) for x in path.read().splitlines()[1:]]
    
    data = [pp_sum(qp_mult(tf[3:],x[:3]),tf[:3]) + qq_mult(tf[3:],x[3:]) for x in raw]
    
    msg = Path()
    
    msg.poses = [list_to_posestamped(x) for x in data]
    
    pub = rospy.Publisher("/path", Path, queue_size=10)
    
    sys.stdout.write("Publishing\n")
    sys.stdout.flush()
    
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        
        pub.publish(msg)
        
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
