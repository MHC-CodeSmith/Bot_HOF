#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(dirname "$(realpath "$0")")
TB4_DIR=$(dirname "$SCRIPT_DIR")

IMAGE="turtlebot4:jazzy"
NAME="tb4_sim"

# ============================
# Check da imagem
# ============================
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[ERRO] A imagem Docker '$IMAGE' não existe."
  echo "       Rode:  cd $SCRIPT_DIR && docker build -t turtlebot4:jazzy ."
  exit 1
fi

# ============================
# X11 ACCESS (safe)
# ============================
xhost +local:root 1>/dev/null 2>&1 || true
cleanup() { xhost -local:root 1>/dev/null 2>&1 || true; }
trap cleanup EXIT

# ============================
# Remove container antigo
# ============================
if docker ps -aq --filter "name=^/${NAME}$" | grep -q .; then
  docker rm -f "${NAME}" >/dev/null 2>&1 || true
fi

# ============================
# GPU AUTO-DETECTION
# ============================
GPU_ARGS=()

if command -v nvidia-smi >/dev/null 2>&1; then
  echo "[run] GPU detectada: NVIDIA"
  GPU_ARGS=(
    --gpus all
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all,compute,utility,graphics
  )
elif [ -d /dev/dri ]; then
  echo "[run] GPU detectada: Intel/AMD (DRI)"
  GPU_ARGS=(
    --device /dev/dri:/dev/dri
  )
else
  echo "[run] Sem GPU dedicada — renderização por software."
  GPU_ARGS=(
    -e LIBGL_ALWAYS_SOFTWARE=1
  )
fi

# ============================
# CLOCK SYNC
# ============================
CLOCK_ARGS=(
  --privileged
  -v /etc/localtime:/etc/localtime:ro
  -v /etc/timezone:/etc/timezone:ro
)

# ============================
# DOCKER RUN (versão blindada)
# ============================
# Nota: ROS_DISCOVERY_SERVER=";10.42.0.172:11811" define o Server ID 1
# (o ponto e vírgula inicial pula o ID 0)
docker run --rm -it \
  --name "${NAME}" \
  --network host \
  --ipc host \
  --shm-size=2g \
  \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_super_client.xml \
  -v "$TB4_DIR/fastdds_super_client.xml:/root/fastdds_super_client.xml:ro" \
  \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  \
  -v "$TB4_DIR/maps:/root/maps" \
  -v "$SCRIPT_DIR/worlds:/root/worlds" \
  -v "$SCRIPT_DIR/worlds:/opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds" \
  \
  -e GZ_SIM_RESOURCE_PATH=/root/worlds:/root/.gz \
  -e IGN_GAZEBO_RESOURCE_PATH=/root/worlds:/root/.gz \
  \
  "${GPU_ARGS[@]}" \
  "${CLOCK_ARGS[@]}" \
  "$IMAGE" \
  bash
