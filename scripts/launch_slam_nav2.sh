#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true

