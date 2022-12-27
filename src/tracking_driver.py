#!/usr/bin/env python3

import rospy

from constants import CENTER_X_CAMERA
from turtlebot3_ball_following.msg import ImageInfo
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

class Tracking_driver:

    GOAL_CENTER = CENTER_X_CAMERA
    MAX_ROTATION = 0.5

    GOAL_RADIUS = 120
    MAX_SPEED = 0.8

    STOP = Twist()

    COMMAND = False

    LAST_POS_X_BALL = 0

    def __init__(self):

        rospy.init_node("tracking_driver", anonymous=True)

        self.service = rospy.Service("/tracking_driver/command", SetBool, self.handler_command)

        self.sub = rospy.Subscriber("/image_processing/info", ImageInfo, self.handler_image_info)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.spin()

    def handler_image_info(self, data):

        if not self.COMMAND:
            self.pub.publish(self.STOP)
            return

        #rospy.loginfo(rospy.get_caller_id() + "\n" + str(data))
        if data.ball_found:
            current_radius = data.ball_radius
            current_pos_x = data.ball_center_x

            self.LAST_POS_X_BALL = current_pos_x

            error_radius = self.GOAL_RADIUS - current_radius
            error_pos = self.GOAL_CENTER - current_pos_x

            speed = Twist()

            if error_radius < 15:
                speed.angular.z = self.MAX_ROTATION * (error_pos / self.GOAL_CENTER)
                self.pub.publish(speed)
                return

            speed = Twist()
            speed.linear.x = self.MAX_SPEED * ((error_radius / self.GOAL_RADIUS) / 1.6)
            speed.angular.z = self.MAX_ROTATION * (error_pos / self.GOAL_CENTER)

            self.pub.publish(speed)
        else:
            angular_speed = 0.5

            rotate = Twist()
            rotate.angular.z = angular_speed if self.LAST_POS_X_BALL < CENTER_X_CAMERA else -angular_speed

            self.pub.publish(rotate)

    def handler_command(self, req):
        self.COMMAND = req.data
        return SetBoolResponse(True, "good")
        
if __name__ == "__main__":
    Tracking_driver()