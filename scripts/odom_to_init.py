#!/usr/bin/python

import rospy
import tf
from math import pi
from nav_msgs.msg import Odometry



tf_truth = [+0.000, +0.000, +0.000,     +0,     +0,     +0,     +0]
tf_rtab  = [+0.073, +0.000, +0.178,     +0,     +0,     +0,     +0]
tf_os1   = [+0.000, +0.000, +0.227,     +0,     +0,     +0,     +0]
tf_ekf   = [+0.000, +0.000, +0.000,     +0,     +0,     +0,     +0]

tf_truth[3:7] = tf.transformations.quaternion_from_euler(pi/180*tf_truth[3], pi/180*tf_truth[4], pi/180*tf_truth[5], 'rxyz')
tf_rtab[3:7]  = tf.transformations.quaternion_from_euler(pi/180*tf_rtab[3],  pi/180*tf_rtab[4],  pi/180*tf_rtab[5],  'rxyz')
tf_os1[3:7]   = tf.transformations.quaternion_from_euler(pi/180*tf_os1[3],   pi/180*tf_os1[4],   pi/180*tf_os1[5],   'rxyz')
tf_ekf[3:7]   = tf.transformations.quaternion_from_euler(pi/180*tf_ekf[3],   pi/180*tf_ekf[4],   pi/180*tf_ekf[5],   'rxyz')



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
        
    odom = Odometry()
    
    odom.header.seq = data.header.seq
    odom.header.stamp = data.header.stamp
    odom.header.frame_id = "base_init"
    odom.child_frame_id = "truth_link"
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, qv_mult(q_conjugate(tf_truth[3:]), v_conjugate(tf_truth[:3])))
    v_base = qv_mult(tf_truth[3:], (v_sensor[0]+v_base[0], v_sensor[1]+v_base[1], v_sensor[2]+v_base[2]))
    v_base = (v_base[0]+tf_truth[0], v_base[1]+tf_truth[1], v_base[2]+tf_truth[2])
    q_base = qq_mult(tf_truth[3:], qq_mult(q_sensor, q_conjugate(tf_truth[3:])))
    
    odom.pose.pose.position.x = v_base[0]
    odom.pose.pose.position.y = v_base[1]
    odom.pose.pose.position.z = v_base[2]
    
    odom.pose.pose.orientation.x = q_base[0]
    odom.pose.pose.orientation.y = q_base[1]
    odom.pose.pose.orientation.z = q_base[2]
    odom.pose.pose.orientation.w = q_base[3]
    
    pub = rospy.Publisher("/truth/odom_init", Odometry, queue_size=10)
    pub.publish(odom)



def callback_rtab(data):
    
    odom = Odometry()
    
    odom.header.seq = data.header.seq
    odom.header.stamp = data.header.stamp
    odom.header.frame_id = "base_init"
    odom.child_frame_id = "rtab_link"
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, qv_mult(q_conjugate(tf_rtab[3:]), v_conjugate(tf_rtab[:3])))
    v_base = qv_mult(tf_rtab[3:], (v_sensor[0]+v_base[0], v_sensor[1]+v_base[1], v_sensor[2]+v_base[2]))
    v_base = (v_base[0]+tf_rtab[0], v_base[1]+tf_rtab[1], v_base[2]+tf_rtab[2])
    q_base = qq_mult(tf_rtab[3:], qq_mult(q_sensor, q_conjugate(tf_rtab[3:])))
    
    odom.pose.pose.position.x = v_base[0]
    odom.pose.pose.position.y = v_base[1]
    odom.pose.pose.position.z = v_base[2]
    
    odom.pose.pose.orientation.x = q_base[0]
    odom.pose.pose.orientation.y = q_base[1]
    odom.pose.pose.orientation.z = q_base[2]
    odom.pose.pose.orientation.w = q_base[3]
    
    pub = rospy.Publisher("/rtabmap/odom_init", Odometry, queue_size=10)
    pub.publish(odom)



def callback_os1(data):
    
    odom = Odometry()
    
    odom.header.seq = data.header.seq
    odom.header.stamp = data.header.stamp
    odom.header.frame_id = "base_init"
    odom.child_frame_id = "lego_link"
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, qv_mult(q_conjugate(tf_os1[3:]), v_conjugate(tf_os1[:3])))
    v_base = qv_mult(tf_os1[3:], (v_sensor[0]+v_base[0], v_sensor[1]+v_base[1], v_sensor[2]+v_base[2]))
    v_base = (v_base[0]+tf_os1[0], v_base[1]+tf_os1[1], v_base[2]+tf_os1[2])
    q_base = qq_mult(tf_os1[3:], qq_mult(q_sensor, q_conjugate(tf_os1[3:])))
    
    odom.pose.pose.position.x = v_base[0]
    odom.pose.pose.position.y = v_base[1]
    odom.pose.pose.position.z = v_base[2]
    
    odom.pose.pose.orientation.x = q_base[0]
    odom.pose.pose.orientation.y = q_base[1]
    odom.pose.pose.orientation.z = q_base[2]
    odom.pose.pose.orientation.w = q_base[3]
    
    pub = rospy.Publisher("/lego/odom_init", Odometry, queue_size=10)
    pub.publish(odom)



def callback_ekf(data):
    
    odom = Odometry()
    
    odom.header.seq = data.header.seq
    odom.header.stamp = data.header.stamp
    odom.header.frame_id = "base_init"
    odom.child_frame_id = "ekf_link"
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, qv_mult(q_conjugate(tf_ekf[3:]), v_conjugate(tf_ekf[:3])))
    v_base = qv_mult(tf_ekf[3:], (v_sensor[0]+v_base[0], v_sensor[1]+v_base[1], v_sensor[2]+v_base[2]))
    v_base = (v_base[0]+tf_ekf[0], v_base[1]+tf_ekf[1], v_base[2]+tf_ekf[2])
    q_base = qq_mult(tf_ekf[3:], qq_mult(q_sensor, q_conjugate(tf_ekf[3:])))
    
    odom.pose.pose.position.x = v_base[0]
    odom.pose.pose.position.y = v_base[1]
    odom.pose.pose.position.z = v_base[2]
    
    odom.pose.pose.orientation.x = q_base[0]
    odom.pose.pose.orientation.y = q_base[1]
    odom.pose.pose.orientation.z = q_base[2]
    odom.pose.pose.orientation.w = q_base[3]
    
    pub = rospy.Publisher("/ekf/odom_init", Odometry, queue_size=10)
    pub.publish(odom)



def main_function():
    
    global tf_world
    
    rospy.init_node("odom_to_init_node", anonymous=True)
    
    tf_arg = rospy.get_param("~tf_world", "0 0 0 0 0 0")
    tf_world = [float(x) for x in tf_arg.split(" ")] + [0.0,]
    tf_world[3:7] = tf.transformations.quaternion_from_euler(pi/180*tf_world[3], pi/180*tf_world[4], pi/180*tf_world[5], 'rxyz')
    
    rospy.Subscriber("/truth/odom", Odometry, callback_truth)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_rtab)
    rospy.Subscriber("/os1/odom", Odometry, callback_os1)
    rospy.Subscriber("/ekf/odom", Odometry, callback_ekf)
    
    rospy.loginfo("Republishing Odometry to Initial Frame...")
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass