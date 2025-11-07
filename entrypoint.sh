#!/bin/bash
set -e

# Source do ambiente ROS
source /opt/ros/humble/setup.bash
source /root/turtlebot4_ws/install/setup.bash

# Permitir acesso gráfico (Gazebo, RViz)
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

exec "$@"
