# Turtlebot3 ball following

Project ASPIC in collaboration with Reau Romain & Tahiri Mohamed.

## Plugins

[Link attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher)

## Installation 

- [Setup TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

- [Setup manipulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator)

## Quick Start

```bash
roslaunch turtlebot3_ball_following simulator_with_arm.launch
roslaunch turtlebot3_ball_following robot_controller.launch
```

Yellow ball teleop
```bash
roslaunch turtlebot3_ball_following ball_teleop.launch
```

Tracking command
```bash
rosrun turtlebot3_ball_following tracking_command.py run
rosrun turtlebot3_ball_following tracking_command.py stop
```

Manipulation command 
```bash
rosrun turtlebot3_ball_following manipulation_command.py init
rosrun turtlebot3_ball_following manipulation_command.py pick
rosrun turtlebot3_ball_following manipulation_command.py detach
rosrun turtlebot3_ball_following manipulation_command.py podium
```