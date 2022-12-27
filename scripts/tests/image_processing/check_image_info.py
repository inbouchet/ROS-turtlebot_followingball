#!/usr/bin/env python3

import rospy
from turtlebot3_ball_following.msg import ImageInfo

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "\n" + str(data))

if __name__ == "__main__":
    rospy.init_node("check_image_data", anonymous=True)
    rospy.Subscriber("/image_processing/info", ImageInfo, callback)

    rospy.spin()
