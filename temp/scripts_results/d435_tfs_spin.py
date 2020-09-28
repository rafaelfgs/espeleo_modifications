#!/usr/bin/env python

import rospy
import sys
import tf
from math import pi
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Quaternion, Transform



freq = 12
is_published = [False, False, False]

tfs = (  #px     #py     #pz     #ax     #ay     #az     #--      #frame_id     #child_id
       [+0.000, -0.000, +0.000,     +0,     +0,     +0,     +0,      "world",  "base_init"],
       [+0.000, -0.000, +0.000,     +0,     +0,     +0,     +0,  "base_init", "truth_link"],
       [+0.500, +0.000, +0.480,     +0,     +0,     +0,     +0,  "base_init", "tower_init"],
       [+0.000, -0.000, +0.000,    -90,    +90,     +0,     +0, "tower_init",   "dyn_init"],
       [+0.015, -0.019, +0.082,    +90,     +0,     +0,     +0,   "dyn_init",  "d435_init"],
       [+0.000, +0.000, +0.000,    -90,    +90,     +0,     +0,  "d435_init",  "odom_init"],
       [+0.000, +0.000, +0.000,    +90,     +0,    +90,     +0,  "odom_pose",  "d435_link"],
       [-0.015, -0.082, -0.019,    -90,     +0,     +0,     +0,  "d435_link",   "dyn_link"],
       [-0.000, +0.000, -0.000,     +0,     +0,     +0,     +0,   "dyn_link", "tower_link"],
       [-0.500, +0.000, -0.480,     +0,     +0,     +0,     +0, "tower_link",  "odom_link"],
       [+0.000, +0.045, +0.045,     +0,    +90,     +0,     +0, "tower_link",   "imu_link"],
       [+0.000, -0.000, +0.000,    -90,    +90,     +0,     +0,  "d435_link", "d435_color_optical_frame"],
       [+0.000, -0.000, +0.000,    -90,    +90,     +0,     +0,  "d435_link", "d435_depth_optical_frame"],
       )

int_row, tth_row, dyn_row = 0, 0, 0
for k in range(len(tfs)):
    tfs[k][3:7] = tf.transformations.quaternion_from_euler(pi/180*tfs[k][3],pi/180*tfs[k][4],pi/180*tfs[k][5],'rxyz')
    if tfs[k][7] == "world"     and tfs[k][8] == "base_init":
        int_row = k
    if tfs[k][7] == "dyn_link"  and tfs[k][8] == "tower_link":
        dyn_row = k
    if tfs[k][7] == "base_init" and tfs[k][8] == "truth_link":
        tth_row = k



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
    
    

def callback_clock(data):
    
    global diff_time
    diff_time = rospy.Time.now().to_sec() - data.clock.to_sec()
    is_published[0] = True



def callback_tth(data):
    
    global tfs
    
    v = qv_mult(tuple(q_conjugate(tfs[int_row][3:7])), (data.translation.x - tfs[int_row][0], 
                                                        data.translation.y - tfs[int_row][1], 
                                                        data.translation.z - tfs[int_row][2])) 
    q = qq_mult(tuple(q_conjugate(tfs[int_row][3:7])), (data.rotation.x, data.rotation.y, 
                                                        data.rotation.z, data.rotation.w))
    tfs[tth_row][:7] = list(v) + list(q)
    is_published[1] = True


def callback_dyn(data):
    
    global tfs
    tfs[dyn_row][3:7] = [data.y, data.x, -data.z, data.w]
    is_published[2] = True



def main_function():
    
    global is_published, tfs

    rospy.init_node("d435_tfs_node", anonymous=True)

    tf0 = [float(x) for x in rospy.get_param("~tf0", "0.0 0.0 0.0 0.0").split(" ")]
    tfs[int_row][0:3] = [tf0[0], tf0[1], tf0[2]]
    tfs[int_row][3:7] = tf.transformations.quaternion_from_euler(0.0, 0.0, pi/180*tf0[3], 'rxyz')

    rospy.Subscriber("/clock", Clock, callback_clock)
    rospy.Subscriber("/ground_truth", Transform, callback_tth)
    rospy.Subscriber("/dynamixel_quat", Quaternion, callback_dyn)

    rate = rospy.Rate(freq)
    while not all(is_published) and not rospy.is_shutdown():
        rate.sleep()
    
    sys.stdout.write("Publishing TFs at " + str(freq) + "Hz...\n")
    sys.stdout.flush()
        
    odom = [tf.TransformBroadcaster(),] * len(tfs)
    
    while not rospy.is_shutdown():
        
        for k in range(len(tfs)):
            t = rospy.Time.from_sec(rospy.Time.now().to_sec() - diff_time)
            odom[k].sendTransform(tuple(tfs[k][0:3]), tuple(tfs[k][3:7]), t, tfs[k][8], tfs[k][7])
        rate.sleep()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass