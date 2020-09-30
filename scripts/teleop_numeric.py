#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import sys, select, termios, tty


moveBindings = {
                '1':(-1, +1),
                '2':(-1, +0),
                '3':(-1, -1),
                '4':(+0, +1),
                '6':(+0, -1),
                '7':(+1, +1),
                '8':(+1, +0),
                '9':(+1, -1),
               }


speedBindings = {
                 'q':(1.25, 1.25),
                 'a':(0.80, 0.80),
                 'w':(1.25, 1.00),
                 's':(0.80, 1.00),
                 'e':(1.00, 1.25),
                 'd':(1.00, 0.80),
                }


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "\nlinear velocity: %s\nangular velocity: %s" % (speed,turn)


if __name__=="__main__":
    
    settings = termios.tcgetattr(sys.stdin)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_numeric_node')
    
    init_vel = input("\nType the value of initial velocity: ")
    
    dyn_pub = rospy.Publisher('/dynamixel_bool', Bool, queue_size=1)
    dyn_gain = False
    
    speed = rospy.get_param("~speed", float(init_vel))
    turn = rospy.get_param("~turn", float(init_vel))
    lin = 0
    ang = 0
    
    try:
        
        print(vels(speed,turn))
        
        while(True):
            
            key = getKey()
            
            if key in moveBindings.keys():
                lin = moveBindings[key][0]
                ang = moveBindings[key][1]
            
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
            
            elif key == "0":
                dyn_gain = not dyn_gain
                dyn_pub.publish(dyn_gain)
            
            else:
                lin = 0
                ang = 0
                if (key == '\x03'):
                    break
            
            twist = Twist()
            twist.linear.x = lin*speed; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang*turn
            pub.publish(twist)
    
    except Exception as e:
        print(e)
    
    finally:
        
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
