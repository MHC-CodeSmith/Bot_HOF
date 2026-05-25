#!/usr/bin/env bash

# run_benchmarks_batch.sh
# Uso: ./scripts/run_benchmarks_batch.sh <NUMERO_DE_EXECUCOES> <PREFIXO_OPCIONAL>
# Exemplo: ./scripts/run_benchmarks_batch.sh 10 test_day1

NUM_RUNS=${1:-5}
PREFIX=${2:-"benchmark"}
ROUTINE=${3:-"delivery"}

# Validar rotina admissível
if [[ "$ROUTINE" != "delivery" && "$ROUTINE" != "failure" && "$ROUTINE" != "restock" ]]; then
    echo "Erro: A rotina deve ser 'delivery', 'failure' ou 'restock'."
    echo "Exemplo: ./scripts/run_benchmarks_batch.sh 10 test_day1 restock"
    exit 1
fi

# Tratamento para capturar Ctrl+C (SIGINT) e abortar TODO o script, não apenas o loop atual
trap 'echo -e "\n[!] Operação cancelada pelo usuário (Ctrl+C). Abortando bateria de testes..."; kill 0; exit 1' SIGINT


BAG_DIR="benchmark_bags"

echo "=========================================================="
echo "  TurtleBot4 - Orquestrador de Benchmark de $ROUTINE"
echo "  Iniciando bateria de $NUM_RUNS execuções..."
echo "  Os bags serão salvos na pasta: $BAG_DIR"
echo "=========================================================="

mkdir -p "$BAG_DIR"

for (( i=1; i<=NUM_RUNS; i++ ))
do
    RUN_ID=$(printf "%02d" $i)
    BAG_NAME="${BAG_DIR}/${PREFIX}_${ROUTINE}_run_${RUN_ID}"
    
    echo ""
    echo "----------------------------------------------------------"
    echo "[Run $RUN_ID/$NUM_RUNS] Preparando execução de $ROUTINE..."
    echo "----------------------------------------------------------"

    # 1. Iniciar gravação do ROS Bag em background salvando apenas os tópicos cruciais
    echo "[Run $RUN_ID] Iniciando captura do ROS Bag em $BAG_NAME..."
    ros2 bag record -o "$BAG_NAME" \
        /odom \
        /amcl_pose \
        /tf \
        /tf_static \
        /goal_pose \
        /product_class \
        /rosout > /dev/null 2>&1 &
    
    BAG_PID=$!
    
    # Dar um tempinho pro bag inicializar 
    sleep 3

    # 2. Disparar a Missão específica
    echo "[Run $RUN_ID] Disparando Trigger de Missão (/start_$ROUTINE)..."
    timeout 10 ros2 service call /start_$ROUTINE std_srvs/srv/Trigger "{}" > /dev/null 2>&1

    # 3. Aguardar a Missão acabar:
    # A missão se encerra quando o robô volta para o DOCK e a action de docking finaliza.
    # Vamos ficar ouvindo o rosout para caçar o aviso final do "=== Delivery: iniciando ===" fechando ciclo
    # Uma approach robusta: escutar /dock_status (se disponível) ou apenas fazer grep por um tempo razoável.
    
    echo "[Run $RUN_ID] Robô em movimento. Monitorando chegada... (Timeout 5 min)"
    
    # Monitoramento via Tópico suprimindo stderr do rclpy p/ evitar BrokenPipe
    # Usamos bash -c para garantir piping correto e match ampliado para sucessos ou falhas.
    timeout 300 bash -c "stdbuf -oL -eL ros2 topic echo /rosout 2>/dev/null | grep -m 1 -E -q 'Dock bem-sucedido\!|Falha ao ir para|Dock falhou'"
    
    STATUS_CODE=$?
    if [ $STATUS_CODE -eq 0 ]; then
        echo "[Run $RUN_ID] ✅ Fim da missão detectado (Sucesso ou Aborto)!"
    elif [ $STATUS_CODE -eq 124 ]; then
         echo "[Run $RUN_ID] ❌ TIMEOUT na missão. Passaram-se 5 minutos."
    else
         echo "[Run $RUN_ID] ⚠️ A missão terminou por interrupção ou Rviz."
    fi

    # 4. Encerrar a gravação do rosbag gracefully (ignorando se já tiver morrido)
    echo "[Run $RUN_ID] Encerrando ROS Bag (PID $BAG_PID)..."
    kill -INT $BAG_PID 2>/dev/null || true
    
    # Aguarda o processo morrer para não corromper o .db3
    wait $BAG_PID 2>/dev/null || true
    echo "[Run $RUN_ID] Bag salvo: $BAG_NAME"
    
    # Uma pequena pausa para o DWB planner zerar custo e buffers antes de reiniciar
    echo "[Run $RUN_ID] Resfriando Nav2 por 10 segundos..."
    sleep 10
done

echo "=========================================================="
echo "🎉 Bateria Completa! $NUM_RUNS runs realizadas."
echo "Use rodadas de plot no python para ver as trajetórias."
echo "=========================================================="
