#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
from turtlebot3_ball_following.msg import ImageInfo

def test_move(image_info):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Test: %s, %s", image_info.ball_center_x, image_info.ball_center_y)
    rate = rospy.Rate(10)

    vel = Twist()
    seuil_gauche = 640/2 - 50
    seuil_droit = 640/2 + 50
    vitesse_lineaire = 0.2

    if image_info.ball_found == True: 

        if image_info.ball_center_x < seuil_gauche:
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0.1
        
        if image_info.ball_center_x > seuil_droit:
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = - 0.1

        vel.linear.x = 0.2
        vel.linear.y = 0
        vel.linear.z = 0

    else :
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0.2

    rospy.loginfo("Linear : %f, Angular : %f", vitesse_lineaire, vel.angular.z)

    pub.publish(vel)
    rate.sleep()

if __name__ == "__main__":
    rospy.init_node("test_move", anonymous=True)
    rospy.Subscriber("/image_processing/info", ImageInfo, test_move)

    rospy.spin()

