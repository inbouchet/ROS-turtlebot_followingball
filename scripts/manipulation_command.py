#!/usr/bin/env python3

import rospy
import argparse

from turtlebot3_ball_following.srv import ManipulationCommand

COMMANDS = ["pick", "detach", "podium", "init"]

def manipulation_command(command):
    rospy.wait_for_service('/manipulation_driver/command')

    run_command = rospy.ServiceProxy('/manipulation_driver/command', ManipulationCommand)
    result = run_command(command)
    
    return result.ok

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("command", type=str, help="send command to manipulation driver (command: pick, podium) (default: stop)")

    args = parser.parse_args()

    command = args.command

    if command in COMMANDS:
        success = manipulation_command(command)

        print(success)
    else:
        print("Wrong command")