#!/usr/bin/env bash
# Script de diagnóstico para verificar TF e scan antes de iniciar Nav2

set -e

echo "=========================================="
echo "  Diagnóstico: TF e LaserScan"
echo "=========================================="
echo ""

# Verificar tópicos
echo "[*] Verificando tópicos do laser..."
if ros2 topic list | grep -qE '/scan$|/scan_2d$'; then
    echo "    ✓ Tópico /scan encontrado"
    ros2 topic info /scan
else
    echo "    ✗ Tópico /scan NÃO encontrado!"
    echo "    Tópicos disponíveis:"
    ros2 topic list | grep scan || echo "    (nenhum tópico scan encontrado)"
fi

echo ""
echo "[*] Verificando mensagens do laser..."
if timeout 2 ros2 topic echo -n 1 /scan > /dev/null 2>&1; then
    echo "    ✓ LaserScan publicando mensagens"
    ros2 topic echo -n 1 /scan | grep -E "frame_id|header" | head -3
else
    echo "    ✗ LaserScan NÃO está publicando!"
fi

echo ""
echo "[*] Verificando TF (odom -> base_link)..."
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link > /dev/null 2>&1; then
    echo "    ✓ Transform odom -> base_link OK"
    ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -5
else
    echo "    ✗ Transform odom -> base_link FALHOU (timeout)"
fi

echo ""
echo "[*] Verificando TF (map -> base_link)..."
if timeout 2 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1; then
    echo "    ✓ Transform map -> base_link OK"
    ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -5
else
    echo "    ⚠ Transform map -> base_link não disponível (normal se SLAM ainda não iniciou)"
fi

echo ""
echo "[*] Listando todas as transforms disponíveis..."
ros2 run tf2_ros tf2_monitor 2>&1 | head -10 || echo "    (tf2_monitor não disponível)"

echo ""
echo "=========================================="
echo "  Diagnóstico completo"
echo "=========================================="


