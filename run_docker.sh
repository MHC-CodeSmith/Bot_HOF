#!/usr/bin/env bash
set -euo pipefail

IMAGE="turtlebot4:humble"
NAME="tb4_sim"

# --- Libera acesso X11 e garante que será revogado ao sair ---
xhost +local:root 1>/dev/null 2>&1 || true
cleanup() { xhost -local:root 1>/dev/null 2>&1 || true; }
trap cleanup EXIT

# --- Remove contêiner antigo com mesmo nome (se existir) ---
if docker ps -aq --filter "name=^/${NAME}$" | grep -q .; then
  docker rm -f "${NAME}" >/dev/null 2>&1 || true
fi

# --- Escolha/auto-detecção de GPU ---
GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1; then
  # NVIDIA
  GPU_ARGS=(--gpus all
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=all,graphics,utility,compute)
  echo "[run] Usando GPU: NVIDIA"
elif [ -d /dev/dri ]; then
  # Intel/AMD
  GPU_ARGS=(--device /dev/dri:/dev/dri)
  echo "[run] Usando GPU: Intel/AMD (DRI)"
else
  # Sem GPU – renderização por software (lento)
  GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
  echo "[run] Sem GPU dedicada (software rendering)."
fi

# --- Execução ---
docker run --rm -it \
  --name "${NAME}" \
  --net host \
  --ipc host \
  --shm-size=2g \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.ignition:/root/.ignition \
  -v $PWD/maps:/root/maps \
  -e IGN_FUEL_CACHE_PATH=/root/.ignition/fuel \
  "${GPU_ARGS[@]}" \
  "${IMAGE}" bash
