#!/usr/bin/env bash
# Lança o TurtleBot4 no mundo do laboratório
# Uso: ./scripts/launch_lab_world.sh [slam] [nav2] [x] [y] [yaw]
# Exemplo: ./scripts/launch_lab_world.sh true true 0.0 0.0 0.0

set -e
source /opt/ros/humble/setup.bash

# Garantir RMW correto
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SLAM=${1:-true}
NAV2=${2:-true}
X_POS=${3:-0.0}
Y_POS=${4:-0.0}
YAW=${5:-0.0}

echo "[*] Lançando TurtleBot4 no mundo do laboratório..."
echo "    RMW: $RMW_IMPLEMENTATION"
echo "    SLAM: $SLAM"
echo "    Nav2: $NAV2"
echo "    Posição inicial: x=$X_POS, y=$Y_POS, yaw=$YAW"
echo ""

if [ "$NAV2" = "true" ]; then
  echo "[*] Iniciando com Nav2 (usando params customizados)..."
  ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
    world:=my_lab \
    rviz:=true \
    slam:=$SLAM \
    nav2:=$NAV2 \
    x:=$X_POS \
    y:=$Y_POS \
    yaw:=$YAW \
    use_sim_time:=true
else
  echo "[*] Iniciando sem Nav2 (apenas sim + SLAM)..."
  ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
    world:=my_lab \
    rviz:=true \
    slam:=$SLAM \
    nav2:=false \
    x:=$X_POS \
    y:=$Y_POS \
    yaw:=$YAW \
    use_sim_time:=true
fi

