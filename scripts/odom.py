#!/usr/bin/env python

"""
This node receives the motors telemetry from the espeleorobo and calculates the odometry based on the wheels velocity
It subscribes to the wheels velocities in ros_eposmcd/motor1 to motor6, published by ros_eposmcd
It publishes the odometry to odom topic
It can calculate the odometry using differential or skidsteering kinematic models,
just change the flag skid_steer to 1 if you want skid steer or to 0 if you want differential
The parameters used both for robot and skidsteer come from Eduardo Cota master thesis
https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
"""

from math import sin, cos, pi

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState


#from bib_espeleo_differential import espeleo_differential

class odometry:
    def __init__(self):

        # Kinematic model
        self.skid_steer = 0

        # Robot Pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.Dth = 0.0

        # Motor velocities
        self.motor_velocity1 = 0
        self.motor_velocity2 = 0
        self.motor_velocity3 = 0
        self.motor_velocity4 = 0
        self.motor_velocity5 = 0
        self.motor_velocity6 = 0
        
        # Skidsteer parameters
        self.alpha_skid = 0.9838227539528335
        self.ycir_skid = 0.3045030420948333

        # Wheel radius for TF (base_init > chassis_init)
        self.wheel_radius = 0.145
        self.robot_width_external = 0.48

        self.reduction_planetary = 111
        self.reduction_synchronizer = 50.0/26.0
        self.time_counter_aux = 0

        self.ros_init()

    def ros_init(self):

        rospy.init_node('wheel_odometry_publisher', anonymous=True)
        rospy.loginfo("Computing Wheel Odometry")
        
	      # Robot mode
        self.side_mode = int(rospy.get_param("~side_mode", "0"))
        self.wheels_mode = 6

        # Times used to integrate velocity to pose
        self.current_time = 0.0
        self.last_time = 0.0

        # Create subscribers that receives the wheels velocities
        self.subscriber_motor1 = rospy.Subscriber("/device1/get_joint_state", JointState, self.motor1_callback)
        self.subscriber_motor2 = rospy.Subscriber("/device2/get_joint_state", JointState, self.motor2_callback)
        self.subscriber_motor3 = rospy.Subscriber("/device3/get_joint_state", JointState, self.motor3_callback)
        self.subscriber_motor4 = rospy.Subscriber("/device4/get_joint_state", JointState, self.motor4_callback)
        self.subscriber_motor5 = rospy.Subscriber("/device5/get_joint_state", JointState, self.motor5_callback)
        self.subscriber_motor6 = rospy.Subscriber("/device6/get_joint_state", JointState, self.motor6_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.vel_pub = rospy.Publisher("robot_vel", Twist, queue_size=1)

        # Tf broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    # Motor velocity callbacks
    def motor1_callback(self, message):
        self.motor_velocity1 = message.velocity[0]

    def motor2_callback(self, message):
        self.motor_velocity2 = message.velocity[0]

    def motor3_callback(self, message):
        self.motor_velocity3 = message.velocity[0]

    def motor4_callback(self, message):
        self.motor_velocity4 = message.velocity[0]

    def motor5_callback(self, message):
        self.motor_velocity5 = message.velocity[0]

    def motor6_callback(self, message):
        self.motor_velocity6 = message.velocity[0]
        self.current_time = message.header.stamp.secs + message.header.stamp.nsecs*0.000000001

        #self.odometry_calculation()
        if self.last_time > 0.0:
            self.odometry_calculation()
        else:
            self.last_time = self.current_time

    def odometry_calculation(self):

        # velocities of each side of the robot, the average of the wheels velocities in RPM
        velocity_left_rpm = ((self.motor_velocity1 + self.motor_velocity2 + self.motor_velocity3))/(self.reduction_planetary * self.reduction_synchronizer * 3)
        # print "Left", velocity_left_rpm
        velocity_right_rpm = ((self.motor_velocity4 + self.motor_velocity5 + self.motor_velocity6))/(self.reduction_planetary * self.reduction_synchronizer * 3)
        # print "Right", velocity_right_rpm

        # Changed RPM to m/s constant value from  0.10471675688 to 0.10471975511965977 -> (2*pi)/60
        # RPM to m/s, and multiplying by the wheel_radius -> (2*pi*radius)/60
        velocity_right = - (0.10471975511965977) * velocity_right_rpm * self.wheel_radius   
        velocity_left = (0.10471975511965977) * velocity_left_rpm * self.wheel_radius

        if self.skid_steer:

            # Linear velocity
            v_robot = (velocity_right + velocity_left) * (self.alpha_skid / 2)

            # Angular velocity
            w_robot = ((velocity_right - velocity_left) * (self.alpha_skid / (2 * self.ycir_skid)))
            
            # Change linear velocity according to side mode
            if self.side_mode == 1:
                v_robot = -v_robot
            
            # Velocity in the XY plane
            vx_robot = v_robot * cos(self.th)
            vy_robot = v_robot * sin(self.th)

            if self.time_counter_aux == 0:
                self.last_time = self.current_time
                self.time_counter_aux = 1

            # Calculating odometry
            dt = self.current_time - self.last_time
            delta_x = vx_robot * dt
            delta_y = vy_robot * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y

            self.th += delta_th


        else:
            
            L = self.robot_width_external/2.0
        
            # Linear velocity
            v_robot = (velocity_right + velocity_left) / 2.0
            # Angular velocity
            w_robot = (velocity_right - velocity_left) / L
            
            # Change linear velocity according to side mode
            if self.side_mode == 1:
                v_robot = -v_robot
            
            # Velocity in the XY plane
            vx_robot = v_robot * cos(self.th)
            vy_robot = v_robot * sin(self.th)

            if self.time_counter_aux == 0:
                self.last_time = self.current_time
                self.time_counter_aux = 1

            # Calculating odometry
            dt = self.current_time - self.last_time
            delta_x = vx_robot * dt
            delta_y = vy_robot * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th


        vel_robot = Twist()
        vel_robot.linear.x = v_robot
        vel_robot.angular.z = w_robot

        # Since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # Transform from base_init to chassis_init
        self.odom_broadcaster.sendTransform(
            (0., 0., self.wheel_radius),
            (0., 0., 0., 1.),
            rospy.Time.from_sec(self.current_time), # rospy.Time.now()
            "chassis_init", # base_link
            "base_init", # odom
        )
        
        # First, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            rospy.Time.from_sec(self.current_time), # rospy.Time.now()
            "wheel_odom", # base_link
            "chassis_init", # odom
        )
        
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(self.current_time) # rospy.Time.now()
        odom.header.frame_id = "chassis_init" # odom

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "wheel_odom" # base_link
        odom.twist.twist = Twist(Vector3(v_robot, 0, 0), Vector3(0, 0, w_robot))
        
        k = 0.01 + abs(odom.twist.twist.angular.z)
        odom.pose.covariance = [      k,     0.0,     0.0,     0.0,     0.0,     0.0,
                                    0.0,       k,     0.0,     0.0,     0.0,     0.0,
                                    0.0,     0.0,     1.0,     0.0,     0.0,     0.0,
                                    0.0,     0.0,     0.0,     1.0,     0.0,     0.0,
                                    0.0,     0.0,     0.0,     0.0,     1.0,     0.0,
                                    0.0,     0.0,     0.0,     0.0,     0.0, 100.0*k]

        # publish the message
        self.odom_pub.publish(odom)
        self.vel_pub.publish(vel_robot)

        self.last_time = self.current_time


if __name__ == '__main__':
    #espeleo = espeleo_differential()
    odometry_obj = odometry()
