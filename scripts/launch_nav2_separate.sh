#!/usr/bin/env bash
# Lança o Nav2 separadamente (depois que a sim já está rodando)
# Use este script se o Nav2 não subir junto com a sim
# Uso: ./scripts/launch_nav2_separate.sh

set -e
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[*] Aguardando 5 segundos para garantir que TF/scan estejam estáveis..."
sleep 5

echo "[*] Verificando TF e scan antes de iniciar Nav2..."
if ! timeout 2 ros2 run tf2_ros tf2_echo odom base_link > /dev/null 2>&1; then
    echo "[!] ERRO: Transform odom -> base_link não disponível!"
    echo "    Execute primeiro: ./scripts/launch_lab_world.sh true false"
    exit 1
fi

if ! timeout 2 ros2 topic echo -n 1 /scan > /dev/null 2>&1; then
    echo "[!] ERRO: Tópico /scan não está publicando!"
    echo "    Execute primeiro: ./scripts/launch_lab_world.sh true false"
    exit 1
fi

echo "[*] TF e scan OK. Iniciando Nav2..."
echo ""

# Tentar usar o launch file do turtlebot4_navigation
if ros2 pkg list | grep -q turtlebot4_navigation; then
    echo "[*] Usando turtlebot4_navigation..."
    ros2 launch turtlebot4_navigation nav2.launch.py \
      use_sim_time:=true \
      params_file:=/root/params/tb4_nav2_params.yaml 2>&1 || {
        echo "[!] Falha ao usar params_file, tentando sem..."
        ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true
    }
else
    echo "[*] turtlebot4_navigation não encontrado, usando nav2_bringup..."
    ros2 launch nav2_bringup bringup_launch.py \
      use_sim_time:=true \
      params_file:=/root/params/tb4_nav2_params.yaml 2>&1 || {
        echo "[!] Falha ao usar params_file, tentando sem..."
        ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true
    }
fi

