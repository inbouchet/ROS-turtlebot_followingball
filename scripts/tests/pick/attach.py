#!/usr/bin/env python

import rospy
from turtlebot3_ball_following.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "link5"
    req.model_name_2 = "yellow_ball"
    req.link_name_2 = "link"

    attach_srv.call(req)
