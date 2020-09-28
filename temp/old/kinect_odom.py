#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry



def callback(data):

  new_data = Odometry()

  new_data.header = data.header
  new_data.child_frame_id = "/odom"

  new_data.pose.pose.position.x =  data.pose.pose.position.z
  new_data.pose.pose.position.y = -data.pose.pose.position.x
  new_data.pose.pose.position.z = -data.pose.pose.position.y

  new_data.pose.pose.orientation.x =  data.pose.pose.orientation.z
  new_data.pose.pose.orientation.y = -data.pose.pose.orientation.x
  new_data.pose.pose.orientation.z = -data.pose.pose.orientation.y
  new_data.pose.pose.orientation.w =  data.pose.pose.orientation.w

  new_data.pose.covariance =  data.pose.covariance

  new_data.twist.twist.linear.x =  data.twist.twist.linear.z
  new_data.twist.twist.linear.y = -data.twist.twist.linear.x
  new_data.twist.twist.linear.z = -data.twist.twist.linear.y

  new_data.twist.twist.angular.x =  data.twist.twist.angular.z
  new_data.twist.twist.angular.y = -data.twist.twist.angular.x
  new_data.twist.twist.angular.z = -data.twist.twist.angular.y

  new_data.twist.covariance =  data.twist.covariance

  pub.publish(new_data)



def corrector():

  global pub

  rospy.init_node('odometry_axes_corrector', anonymous=True)

  rospy.Subscriber('/kinect_odometer/odometry', Odometry, callback)

  pub = rospy.Publisher('/vo', Odometry, queue_size=10)

  rate = rospy.Rate(30)

  while not rospy.is_shutdown():
    rate.sleep()



if __name__ == '__main__':

  try:
    corrector()
  except rospy.ROSInterruptException:
    pass
