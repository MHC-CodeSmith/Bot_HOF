#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
ros2 launch turtlebot4_navigation localization.launch.py map:=/root/maps/warehouse.yaml
ros2 launch turtlebot4_navigation nav2.launch.py
