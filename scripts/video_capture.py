#!/usr/bin/env python3

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraDriver:

    bridge = CvBridge()

    def __init__(self):
        rospy.init_node("take_image", anonymous=True)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_handler)

        rospy.spin()

    def camera_handler(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv.imshow("Image window", cv_image)
        
        if cv.waitKey(1) & 0xFF == ord("q"):
            cv.destroyAllWindows()
            self.sub.unregister()
            rospy.signal_shutdown("end")

if __name__ == "__main__":
    CameraDriver()
