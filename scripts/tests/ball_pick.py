#!/usr/bin/env python3

import rospy
import moveit_commander

class Ball_pick:

    INIT_ARM = [0, 0, 0, 0]
    PICK_BALL_ARM  = [0, 1.050, -0.3, 0]
    HOME_BALL_ARM = [0, -0.9, -0.2, 0]

    OPEN_GRIPPER = [0.019, 0.019]

    def __init__(self):
        rospy.init_node("ball_pick", anonymous=True)

        self.__run_gripper(self.OPEN_GRIPPER)

        self.run_pick_ball()

    def run_pick_ball(self):
        self.__run_gripper(self.OPEN_GRIPPER)
        self.__run_arm(self.HOME_BALL_ARM)

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

        
if __name__ == "__main__":
    Ball_pick()