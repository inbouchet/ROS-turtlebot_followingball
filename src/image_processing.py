#!/usr/bin/env python3

import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from turtlebot3_ball_following.msg import ImageInfo
from constants import LOWER_YELLOW, HIGHER_YELLOW

class ImageProcessing:

    bridge = CvBridge()

    def __init__(self):

        rospy.init_node("camera_driver", anonymous=True)

        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_listener_callback)
        self.pub = rospy.Publisher("/image_processing/info", ImageInfo, queue_size=10)

        self.rate = rospy.Rate(75) # 75hz

        rospy.spin()

        self.unregister()

    def image_listener_callback(self, image_data):
        try:
            frame = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_frame, LOWER_YELLOW, HIGHER_YELLOW)

        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

        image_info = ImageInfo()

        if len(contours) == 0:
            rospy.loginfo(rospy.get_caller_id() + " I don't see yellow ball (contours)")
        else:
            circles = []
            for c in contours:
                ((x, y), radius) = cv.minEnclosingCircle(c)
                circles.append([int(x), int(y), int(radius)])

            if len(circles) == 0:
                rospy.loginfo(rospy.get_caller_id() + " I don't see yellow ball (minEnclosingCircle)")
        
            for [x, y, radius] in circles:
                if mask[y, x]:
                    image_info.ball_found = True
                    image_info.ball_center_x = x
                    image_info.ball_center_y = y
                    image_info.ball_radius = radius

                    rospy.loginfo(rospy.get_caller_id() + " Publishing image information :\n" + str(image_info))

                    #TODO Gerer le cas où on a plusieurs boules
                    break

        self.pub.publish(image_info)

        self.rate.sleep()

    def unregister(self):
        self.sub.unregister()
        self.pub.unregister()

if __name__ == "__main__":
    ImageProcessing()
