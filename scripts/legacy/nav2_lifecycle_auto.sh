#!/usr/bin/env bash
set -euo pipefail

NODES=(
  /controller_server
  /planner_server
  /behavior_server
  /smoother_server
  /bt_navigator
  /waypoint_follower
)

# Uso:
#   ./nav2_lifecycle_auto.sh            # tenta levar tudo para ACTIVE
#   ./nav2_lifecycle_auto.sh refresh    # força DEACTIVATE (se ativo) e volta para ACTIVE
MODE="${1:-}"

log() { echo -e "[nav2-life] $*"; }

node_exists() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

get_state() {
  # Saída típica: "Current state: active"
  ros2 lifecycle get "$1" 2>/dev/null | awk -F': ' '/Current state/ {print $2}'
}

do_set() {
  local node="$1" transition="$2"
  log "  $node <= $transition"
  ros2 lifecycle set "$node" "$transition" >/dev/null
}

bring_to_active() {
  local node="$1"
  if ! node_exists "$node"; then
    log "  $node: (não existe / não está visível no ROS graph) - pulando"
    return 0
  fi

  local state
  state="$(get_state "$node" || true)"

  if [[ -z "${state}" ]]; then
    log "  $node: não consegui ler estado (talvez não seja lifecycle node?)"
    return 0
  fi

  # Modo refresh: se estiver ACTIVE, desativa antes
  if [[ "$MODE" == "refresh" && "$state" == "active" ]]; then
    do_set "$node" deactivate
    state="$(get_state "$node")"
  fi

  # Caminho até ACTIVE depende do estado atual
  case "$state" in
    unconfigured)
      do_set "$node" configure
      state="$(get_state "$node")"
      ;;&
    inactive)
      do_set "$node" activate
      ;;
    active)
      log "  $node: já está ACTIVE"
      ;;
    finalized)
      log "  $node: está FINALIZED (precisa relançar o nó / bringup)."
      ;;
    *)
      log "  $node: estado desconhecido '$state' (vou tentar levar a inactive->active se possível)"
      # tentativa conservadora: se der, ativa; se não, deixa quieto
      ros2 lifecycle set "$node" activate >/dev/null 2>&1 || true
      ;;
  esac
}

log "Modo: ${MODE:-normal} | Alvo: ACTIVE"
for n in "${NODES[@]}"; do
  log "Processando $n"
  bring_to_active "$n"
done

log ""
log "Resumo final:"
for n in "${NODES[@]}"; do
  if node_exists "$n"; then
    st="$(get_state "$n" || true)"
    printf "  %-20s : %s\n" "$n" "${st:-?}"
  else
    printf "  %-20s : (não existe)\n" "$n"
  fi
done

