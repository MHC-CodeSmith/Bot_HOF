#!/usr/bin/env bash
# ============================================================
# Script para rodar o TurtleBot4 no mundo do laboratório
# Versão estável - RMW FastDDS (rmw_fastrtps_cpp)
# ============================================================

set -euo pipefail

IMAGE="turtlebot4:jazzy"
NAME="tb4_sim"

# Parâmetros opcionais
SLAM=${1:-true}
NAV2=${2:-true}
X_POS=${3:-0.0}
Y_POS=${4:-0.0}
YAW=${5:-0.0}

echo "=========================================="
echo "  TurtleBot4 - Mundo do Laboratório"
echo "=========================================="
echo "SLAM: $SLAM"
echo "Nav2: $NAV2"
echo "Posição inicial: x=$X_POS, y=$Y_POS, yaw=$YAW"
echo "=========================================="

# Verificar se o mundo existe
if [ ! -f "worlds/my_lab.sdf" ]; then
    echo "[!] Erro: worlds/my_lab.sdf não encontrado!"
    echo "    Gere o mundo primeiro com:"
    echo "    python3 scripts/gen_world.py config/lab.yaml worlds/my_lab.sdf"
    exit 1
fi

# Verificar se a imagem existe
if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${IMAGE}$"; then
    echo "[!] Erro: Imagem $IMAGE não encontrada!"
    echo "    Faça o build primeiro com:"
    echo "    docker build --no-cache -t $IMAGE ."
    exit 1
fi

# Libera acesso X11
xhost +local:root 1>/dev/null 2>&1 || true
cleanup() { xhost -local:root 1>/dev/null 2>&1 || true; }
trap cleanup EXIT

# Remove contêiner antigo
if docker ps -aq --filter "name=^/${NAME}$" | grep -q .; then
    echo "[*] Removendo contêiner antigo..."
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
fi

# Escolha/auto-detecção de GPU
GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1; then
    GPU_ARGS=(--gpus all
              -e NVIDIA_VISIBLE_DEVICES=all
              -e NVIDIA_DRIVER_CAPABILITIES=all,graphics,utility,compute)
    echo "[*] Usando GPU: NVIDIA"
elif [ -d /dev/dri ]; then
    GPU_ARGS=(--device /dev/dri:/dev/dri)
    echo "[*] Usando GPU: Intel/AMD (DRI)"
else
    GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
    echo "[*] Sem GPU dedicada (software rendering)"
fi

echo "[*] Iniciando contêiner..."
echo "[*] Para sair: Ctrl+C ou feche o terminal"
echo ""

# Executa o contêiner e lança o mundo
docker run --rm -it \
  --name "${NAME}" \
  --net host \
  --ipc host \
  --shm-size=2g \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR="/tmp/runtime-root" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.gz:/root/.gz \
  -v $PWD/maps:/root/maps \
  -v $PWD/worlds:/root/worlds \
  -v $PWD/params:/root/params \
  -v $PWD/worlds:/opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds \
  -e GZ_SIM_RESOURCE_PATH=/root/worlds:/root/.gz \
  -e IGN_GAZEBO_RESOURCE_PATH=/root/worlds:/root/.gz \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  "${GPU_ARGS[@]}" \
  "${IMAGE}" bash -i -c "
    source /opt/ros/jazzy/setup.bash
    unset RMW_IMPLEMENTATION
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    echo '[*] Lançando TurtleBot4 no mundo do laboratório...'
    echo '[*] RMW: \$RMW_IMPLEMENTATION'
    echo '[*] SLAM: $SLAM, Nav2: $NAV2'
    echo '[*] Posição inicial: x=$X_POS, y=$Y_POS, yaw=$YAW'
    echo ''

    if [ '$NAV2' = 'true' ]; then
      echo '[*] Iniciando com Nav2...'
      echo '[*] Nota: Se Nav2 falhar, inicie sem Nav2 primeiro e depois:'
      echo '         ./scripts/launch_nav2_separate.sh'
      env RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
        world:=my_lab rviz:=true slam:=$SLAM nav2:=$NAV2 \
        x:=$X_POS y:=$Y_POS yaw:=$YAW use_sim_time:=true
    else
      echo '[*] Iniciando sem Nav2 (apenas sim + SLAM)...'
      echo '[*] Para iniciar Nav2 depois, execute:'
      echo '    ./scripts/launch_nav2_separate.sh'
      env RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
        world:=my_lab rviz:=true slam:=$SLAM nav2:=false \
        x:=$X_POS y:=$Y_POS yaw:=$YAW use_sim_time:=true
    fi
    exec bash
  "
