#!/usr/bin/env python3

import rospy

from turtlebot3_ball_following.msg import ImageInfo
from geometry_msgs.msg import Twist

class Ball_tracking_v0:

    HEIGHT, WIDTH = 480, 640
    CENTER_X_CAMERA = int(WIDTH / 2)
    GOAL_CENTER = CENTER_X_CAMERA
    MAX_ROTATION = 0.5

    GOAL_RADIUS = 120
    MAX_SPEED = 0.8

    STOP = Twist()

    def __init__(self):

        rospy.init_node("ball_tracking_v0", anonymous=True)
        self.sub = rospy.Subscriber("/image_processing/info", ImageInfo, self.callback_image_info)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.spin()

        pass

    def callback_image_info(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "\n" + str(data))
        if data.ball_found:
            current_radius = data.ball_radius
            current_pos_x = data.ball_center_x

            error_radius = self.GOAL_RADIUS - current_radius
            error_pos = self.GOAL_CENTER - current_pos_x

            if error_radius < 15:
                self.pub.publish(self.STOP)
                return

            speed = Twist()
            speed.linear.x = self.MAX_SPEED * ((error_radius / self.GOAL_RADIUS) / 1.6)
            speed.angular.z = self.MAX_ROTATION * (error_pos / self.GOAL_CENTER)

            self.pub.publish(speed)
        else:
            rotate = Twist()
            rotate.angular.z = 0.5

            self.pub.publish(rotate)
        
if __name__ == "__main__":
    Ball_tracking_v0()