#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np

from time import sleep
from constants import WIDTH_CAMERA, WIDTH_ROBOT, LENGTH_ROBOT, CENTER_X_CAMERA
from turtlebot3_ball_following.msg import ImageInfo
from turtlebot3_ball_following.srv import Attach, AttachRequest, ManipulationCommand, ManipulationCommandResponse

class Manipulation_driver:

    INIT_ARM = [0, 0, 0, 0]
    PICK_BALL_ARM  = [0, 1.050, -0.3, 0]
    HOME_BALL_ARM = [0, -0.9, -0.2, 0]

    OPEN_GRIPPER = [0.019, 0.019]

    def __init__(self):
        rospy.init_node("manipulation_driver", anonymous=True)
        
        self.service = rospy.Service("/manipulation_driver/command", ManipulationCommand, self.handler_run_command)

        rospy.spin()

    def handler_run_command(self, req):
        command = req.command

        status = False

        if command == "init":
            self.__run_gripper(self.OPEN_GRIPPER)
            self.__run_arm(self.INIT_ARM)

            status = True
        elif command == "pick":
            self.__run_gripper(self.OPEN_GRIPPER)
            self.__run_arm(self.PICK_BALL_ARM)

            sleep(2)

            self.__attach()

            status = True
        elif command == "detach":
            self.__detach()

            status = True
        elif command == "podium":
            self.__run_gripper(self.OPEN_GRIPPER)
            self.__run_arm(self.HOME_BALL_ARM)

            status = True

        return ManipulationCommandResponse(status)
        
    def __run_arm(self, joints):
        arm_group = moveit_commander.MoveGroupCommander("arm")
        
        arm_joints = joints
        
        arm_group.go(arm_joints, wait=True)
        arm_group.stop()

    def __run_gripper(self, joints):
        gripper_group = moveit_commander.MoveGroupCommander("gripper")
        gripper_joints = joints

        gripper_group.go(gripper_joints, wait=True)
        gripper_group.stop()

    def __attach(self):
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()

        rospy.loginfo("Attaching yellow ball and arm")
        req = AttachRequest()
        req.model_name_1 = "robot"
        req.link_name_1 = "link5"
        req.model_name_2 = "yellow_ball"
        req.link_name_2 = "link"

        attach_srv.call(req)

    def __detach(self):
        detact_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        detact_srv.wait_for_service()

        rospy.loginfo("Detaching yellow ball and arm")
        req = AttachRequest()
        req.model_name_1 = "robot"
        req.link_name_1 = "link5"
        req.model_name_2 = "yellow_ball"
        req.link_name_2 = "link"

        detact_srv.call(req)

    """def __get_joints0_arm(self):

        image_info = rospy.wait_for_message("/image_processing/info", ImageInfo)
        BALL_CENTER_X = image_info.ball_center_x

        diff_camera = BALL_CENTER_X - CENTER_X_CAMERA

        oppose = (diff_camera * WIDTH_ROBOT) / WIDTH_CAMERA
        oppose -= 0.03

        offset_tangent = LENGTH_ROBOT / 2 + 0.03
        tangent = LENGTH_ROBOT / 2 + offset_tangent

        joints0 = np.arctan2(oppose, tangent)

        rospy.loginfo(rospy.get_caller_id() + "\n" + str(oppose) + " - " + str(tangent) + "\n------------------------")

        rospy.loginfo(rospy.get_caller_id() + "\n" + str(BALL_CENTER_X) + " - " + str(CENTER_X_CAMERA) + " - " + str(diff_camera) + " - " + str(joints0))

        return -joints0 / 2"""


if __name__ == "__main__":
    Manipulation_driver()