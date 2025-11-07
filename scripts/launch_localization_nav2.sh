#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py
ros2 launch turtlebot4_navigation localization.launch.py map:=/root/maps/warehouse.yaml
ros2 launch turtlebot4_navigation nav2.launch.py

