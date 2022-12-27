#!/usr/bin/env python3

import rospy
import sys, select, os
import tty, termios
from geometry_msgs.msg import Twist


LIN_VEL_STEP = 0.2
ANG_VEL_STEP = 1.0

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":

    rospy.init_node('ball_teleop_key')
    pub = rospy.Publisher('/gazebo/ball_moving_info', Twist, queue_size=10)

    msg = """
=============================
Moving the ball around:
 'A' Z 'E'
Q    S    D

X to stop all motion !    
Spacebar to jump !

CTRL-C to quit
=============================
"""

    print(msg)

    settings = termios.tcgetattr(sys.stdin)
    linear_vel_x = 0
    linear_vel_y = 0
    linear_vel_z = 0
    angular_vel = 0
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'z' :
            linear_vel_x += - LIN_VEL_STEP
        elif key == 'q' :
            linear_vel_y += - LIN_VEL_STEP
        elif key == 's' :
            linear_vel_x += LIN_VEL_STEP
        elif key == 'd' :
            linear_vel_y += LIN_VEL_STEP
        elif key == 'a' :
            angular_vel += ANG_VEL_STEP
        elif key == 'e' :
            angular_vel += ANG_VEL_STEP
        elif key == ' ' :
            linear_vel_z = 1.0
        elif key == 'x' : # Press x to stop
            print("Ball stopped moving !")
            linear_vel_x = 0.0
            linear_vel_y = 0.0
            linear_vel_z = 0.0
            angular_vel  = 0.0
        else:
            if key == '\x03' : # CTRL-C
                break

        twist = Twist()

        twist.linear.x = linear_vel_x
        twist.linear.y = linear_vel_y
        twist.linear.z = linear_vel_z

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel

        pub.publish(twist)
