#!/usr/bin/env python

import rospy
from copy import copy
import tf
from math import pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped



freq = 12
is_published = [False,]



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

def qv_mult(q1, v1):
    q2 = v1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]



def callback_tth(data):
    
    global tfs, is_published
    
    v = qv_mult(tuple(q_conjugate(tfs[wld_row][3:7])), (data.pose.position.x - tfs[wld_row][0], 
                                                        data.pose.position.y - tfs[wld_row][1], 
                                                        data.pose.position.z - tfs[wld_row][2])) 
    q = qq_mult(tuple(q_conjugate(tfs[wld_row][3:7])), (data.pose.orientation.x, data.pose.orientation.y, 
                                                        data.pose.orientation.z, data.pose.orientation.w))
    tfs[tth_row][:7] = list(v) + list(q)
    
    odom = Odometry()
    odom.header.seq = copy(data.header.seq)
    odom.header.stamp = copy(data.header.stamp)
    odom.header.frame_id = "base_init"
    odom.child_frame_id = "truth_link"
    odom.pose.pose.position.x = v[0]
    odom.pose.pose.position.y = v[1]
    odom.pose.pose.position.z = v[2]
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    pub = rospy.Publisher("/truth/odom", Odometry, queue_size=10)
    pub.publish(odom)
    
    is_published[0] = True



def main_function():
    
    global tfs, wld_row, tth_row

    rospy.init_node("tower_tfs_node", anonymous=True)
    
    tf_arg = rospy.get_param("~tf_world", "0 0 0 0 0 0")
    tf_world = [float(x) for x in tf_arg.split(" ")] + [0.0,]
    
    tfs = (  #px     #py     #pz     #ax     #ay     #az     #--      #frame_id     #child_id
            tf_world +                                             [       "world",    "base_init"],
           [+0.000, +0.000, +0.150,     +0,     +0,     +0,     +0,    "base_init", "chassis_init"],  
           [+0.000, +0.000, +0.000,     +0,     +0,     +0,     +0, "chassis_init",   "truth_link"],
           [+0.073, +0.000, +0.178,     +0,    -15,     +0,     +0, "chassis_init",    "rtab_init"],
           [+0.000, +0.000, +0.227,     +0,     +0,     +0,     +0, "chassis_init",    "lego_init"],
           [+0.000, +0.000, +0.000,     +0,     +0,     +0,     +0, "chassis_init",     "ekf_init"],
           [+0.000, +0.000, +0.000,     +0,     +0,     +0,     +0, "chassis_init",    "odom_init"],
           [-0.073, +0.000, -0.178,     +0,    +15,     +0,     +0,    "rtab_pose",    "rtab_link"],
           [+0.000, +0.000, +0.077,     +0,     +0,     +0,     +0,    "rtab_link",     "rtab_imu"],
           [+0.000, +0.000, +0.000,     +0,     +0,     +0,     +0,     "ekf_pose",     "ekf_link"],
           [+0.000, +0.000, +0.077,     +0,     +0,     +0,     +0,     "ekf_link",      "ekf_imu"],
           [+0.073, +0.000, +0.178,     +0,    +75,    -90,     +0,    "rtab_link", "d435_color_optical_frame"],
           [+0.073, +0.000, +0.178,     +0,    +75,    -90,     +0,    "rtab_link", "d435_depth_optical_frame"],
           )
    
    wld_row, tth_row = 0, 0
    for k in range(len(tfs)):
        tfs[k][3:7] = tf.transformations.quaternion_from_euler(pi/180*tfs[k][3],pi/180*tfs[k][4],pi/180*tfs[k][5],'rxyz')
        if tfs[k][7] == "world"        and tfs[k][8] == "base_init":
            wld_row = k
        if tfs[k][7] == "chassis_init" and tfs[k][8] == "truth_link":
            tth_row = k
    
    rospy.Subscriber("/ground_truth", PoseStamped, callback_tth)
    
    rate = rospy.Rate(freq)
    while not all(is_published) and not rospy.is_shutdown():
        rate.sleep()
    
    rospy.loginfo("Publishing TFs at %dHz..." % freq)
    
    odom = [tf.TransformBroadcaster(),] * len(tfs)
    
    while not rospy.is_shutdown():
        
        for k in range(len(tfs)):
            t = rospy.Time.now()
            odom[k].sendTransform(tuple(tfs[k][0:3]), tuple(tfs[k][3:7]), t, tfs[k][8], tfs[k][7])
        rate.sleep()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass