#!/bin/bash
set -e

ROBOT_IP="192.168.171.138"
ROBOT_USER="ubuntu"
PC_IP=$(ip route get 192.168.171.138 | sed -n 's/.*src \([0-9.]*\).*/\1/p')
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPORT_FILE="${REPORT_FILE:-${HOME}/.gemini/antigravity/brain/b7b70953-9bad-4f46-8a19-ef9e15823f94/network_profiling_report.md}"
mkdir -p "$(dirname "$REPORT_FILE")"

echo "# Network Communication Drill-Down Report" > "$REPORT_FILE"
echo "Date: $(date)" >> "$REPORT_FILE"
echo "" >> "$REPORT_FILE"
echo "PC IP: **$PC_IP**" >> "$REPORT_FILE"
echo "Robot IP: **$ROBOT_IP**" >> "$REPORT_FILE"
echo "" >> "$REPORT_FILE"

echo "## 1. Raw IP Communication (ICMP Ping)" >> "$REPORT_FILE"
echo "### PC -> Robot" >> "$REPORT_FILE"
echo '```text' >> "$REPORT_FILE"
ping -c 5 $ROBOT_IP >> "$REPORT_FILE" 2>&1 || true
echo '```' >> "$REPORT_FILE"

echo "### Robot -> PC" >> "$REPORT_FILE"
echo '```text' >> "$REPORT_FILE"
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "ping -c 5 $PC_IP" >> "$REPORT_FILE" 2>&1 || echo "SSH failed" >> "$REPORT_FILE"
echo '```' >> "$REPORT_FILE"

echo "## 2. ROS 2 Transport Instrumentation (Fast DDS RTT & Drop Rate)" >> "$REPORT_FILE"
echo "Copying instrumentation script to robot..."
scp -o BatchMode=yes "${SCRIPT_DIR}/network_instrumentation.py" $ROBOT_USER@$ROBOT_IP:/tmp/network_instrumentation.py
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "chmod +x /tmp/network_instrumentation.py"

echo "Starting Echo Server on Robot..."
ssh -o BatchMode=yes -f $ROBOT_USER@$ROBOT_IP '/bin/bash -c "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DISCOVERY_SERVER=192.168.171.138:11811 && nohup python3 /tmp/network_instrumentation.py --mode server > /tmp/ros_server.log 2>&1 &"'
sleep 5 # wait for server discovery

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.171.138:11811"

echo "### Test A: Small Payload (100 Bytes) - 10 Seconds" >> "$REPORT_FILE"
echo '```text' >> "$REPORT_FILE"
timeout 12 python3 "${SCRIPT_DIR}/network_instrumentation.py" --mode client --size 100 > /tmp/client_small.log 2>&1 &
PID_SMALL=$!
sleep 10
kill -INT $PID_SMALL || true
sleep 2
cat /tmp/client_small.log >> "$REPORT_FILE"
echo '```' >> "$REPORT_FILE"

echo "### Test B: Large Payload (1,000,000 Bytes ~1MB) - 10 Seconds" >> "$REPORT_FILE"
echo '```text' >> "$REPORT_FILE"
timeout 12 python3 "${SCRIPT_DIR}/network_instrumentation.py" --mode client --size 1000000 > /tmp/client_large.log 2>&1 &
PID_LARGE=$!
sleep 10
kill -INT $PID_LARGE || true
sleep 2
cat /tmp/client_large.log >> "$REPORT_FILE"
echo '```' >> "$REPORT_FILE"

echo "Cleaning up Robot Server..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "pkill -f network_instrumentation.py" || true
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "cat /tmp/ros_server.log" > /tmp/ros_server_dump.log

echo "### Remote Server Logs Extract (Echoes)" >> "$REPORT_FILE"
echo '```text' >> "$REPORT_FILE"
grep -m 15 "Echoed" /tmp/ros_server_dump.log >> "$REPORT_FILE" || echo "No echoes logged" >> "$REPORT_FILE"
echo '```' >> "$REPORT_FILE"

echo "## 3. Network Interfaces Data" >> "$REPORT_FILE"
echo '```text' >> "$REPORT_FILE"
ip addr >> "$REPORT_FILE"
echo '```' >> "$REPORT_FILE"

echo "Profiling successfully finished."
