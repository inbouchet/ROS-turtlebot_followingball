#!/usr/bin/env python3

import rospy
import argparse

from std_srvs.srv import SetBool

def tracking_command(command):
    rospy.wait_for_service('/tracking_driver/command')

    run_command = rospy.ServiceProxy('/tracking_driver/command', SetBool)
    result = run_command(command)
    
    return result.success

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("command", type=str, help="send command to tracking driver (command: run, stop) (default: stop)")

    args = parser.parse_args()

    command_name = args.command

    command = True if command_name == "run" else False
    success = tracking_command(command)

    print(success)