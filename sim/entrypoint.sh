#!/bin/bash
set -e

# Source do ambiente ROS
source /opt/ros/jazzy/setup.bash
if [ -f /root/turtlebot4_ws/install/setup.bash ]; then
  source /root/turtlebot4_ws/install/setup.bash
fi

# Permitir acesso gráfico (Gazebo, RViz)
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

exec "$@"
