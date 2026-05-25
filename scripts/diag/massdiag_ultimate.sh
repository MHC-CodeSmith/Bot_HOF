#!/bin/bash
set -e

ROBOT_IP="192.168.171.138"
ROBOT_USER="ubuntu"
PC_IP=$(ip route get 192.168.171.138 | sed -n 's/.*src \([0-9.]*\).*/\1/p')
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

LOCAL_OUT="${LOCAL_OUT:-${HOME}/Downloads/massdiag_robot_turtlebot4_${TIMESTAMP}}"
REMOTE_OUT="/tmp/massdiag_${TIMESTAMP}"

# Create dual structure
mkdir -p "$LOCAL_OUT/ROBOT_DATA"
mkdir -p "$LOCAL_OUT/PC_DATA"

echo "======================================================"
echo "    ULTIMATE MASS DIAGNOSTIC SCRIPT (PC & ROBOT)      "
echo "======================================================"
echo "Targeting Robot: $ROBOT_IP (User: $ROBOT_USER)"
echo "Targeting PC: $PC_IP"
echo "Output Directory: $LOCAL_OUT"
echo ""

# Validate SSH
if ! ssh -o ConnectTimeout=3 -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "echo 'SSH OK'" > /dev/null 2>&1; then
    echo "ERROR: Could not SSH into $ROBOT_IP without password. Diagnostics aborted."
    exit 1
fi

echo "Creating remote directory..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "mkdir -p $REMOTE_OUT"

# ==========================================
# 1. SUMMARY
# ==========================================
echo "1/8: Generating 00_summary.txt (PC & Robot)..."
# Robot Summary
cat << 'EOF' > /tmp/gen_summary_robot.sh
echo "=== Mass Diagnostic Summary (ROBOT) ==="
echo "Date: $(date)"
echo "Hostname: $(hostname)"
echo "Uptime: $(uptime -p)"
echo "CPU Architecture: $(uname -m)"
echo "OS Version: $(cat /etc/os-release | grep PRETTY_NAME | cut -d '=' -f 2 | tr -d '\"')"
echo ""
echo "=== ROS 2 Environments ==="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-Not Set}"
echo "ROS_DISCOVERY_SERVER: ${ROS_DISCOVERY_SERVER:-Not Set}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-Not Set}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-Not Set}"
echo ""
echo "=== IP Addresses ==="
ip addr show | grep 'inet ' | awk '{print $NF, $2}' | column -t
EOF
scp -o BatchMode=yes /tmp/gen_summary_robot.sh $ROBOT_USER@$ROBOT_IP:/tmp/
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "bash /tmp/gen_summary_robot.sh > $REMOTE_OUT/00_summary.txt"

# PC Summary
cat << 'EOF' > /tmp/gen_summary_pc.sh
echo "=== Mass Diagnostic Summary (PC) ==="
echo "Date: $(date)"
echo "Hostname: $(hostname)"
echo "Uptime: $(uptime -p)"
echo "CPU Architecture: $(uname -m)"
echo "OS Version: $(cat /etc/os-release | grep PRETTY_NAME | cut -d '=' -f 2 | tr -d '\"')"
echo ""
echo "=== ROS 2 Environments ==="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-Not Set}"
echo "ROS_DISCOVERY_SERVER: ${ROS_DISCOVERY_SERVER:-Not Set}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-Not Set}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-Not Set}"
echo ""
echo "=== IP Addresses ==="
ip addr show | grep 'inet ' | awk '{print $NF, $2}' | column -t
EOF
bash /tmp/gen_summary_pc.sh > "$LOCAL_OUT/PC_DATA/00_summary.txt"

# ==========================================
# 2. NETWORK
# ==========================================
echo "2/8: Generating 01_network.txt (PC & Robot)..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "echo '--- ip route ---' > $REMOTE_OUT/01_network.txt && ip route >> $REMOTE_OUT/01_network.txt && echo '--- ip neigh ---' >> $REMOTE_OUT/01_network.txt && ip neigh >> $REMOTE_OUT/01_network.txt && echo '--- ifconfig ---' >> $REMOTE_OUT/01_network.txt && ifconfig -a >> $REMOTE_OUT/01_network.txt 2>/dev/null || true"

echo '--- ip route ---' > "$LOCAL_OUT/PC_DATA/01_network.txt"
ip route >> "$LOCAL_OUT/PC_DATA/01_network.txt"
echo '--- ip neigh ---' >> "$LOCAL_OUT/PC_DATA/01_network.txt"
ip neigh >> "$LOCAL_OUT/PC_DATA/01_network.txt"
echo '--- ifconfig ---' >> "$LOCAL_OUT/PC_DATA/01_network.txt"
ifconfig -a >> "$LOCAL_OUT/PC_DATA/01_network.txt" 2>/dev/null || true

# ==========================================
# 3. WIFI
# ==========================================
echo "3/8: Generating 02_wifi.txt (PC & Robot)..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "echo '--- iwconfig ---' > $REMOTE_OUT/02_wifi.txt && iwconfig >> $REMOTE_OUT/02_wifi.txt 2>/dev/null || true && echo '--- iw dev link ---' >> $REMOTE_OUT/02_wifi.txt && iw dev wlan0 link >> $REMOTE_OUT/02_wifi.txt 2>/dev/null || iw dev wlo1 link >> $REMOTE_OUT/02_wifi.txt 2>/dev/null || true && echo '--- nmcli d wifi ---' >> $REMOTE_OUT/02_wifi.txt && nmcli d wifi >> $REMOTE_OUT/02_wifi.txt 2>/dev/null || true"

echo '--- iwconfig ---' > "$LOCAL_OUT/PC_DATA/02_wifi.txt"
iwconfig >> "$LOCAL_OUT/PC_DATA/02_wifi.txt" 2>/dev/null || true
echo '--- iw dev link ---' >> "$LOCAL_OUT/PC_DATA/02_wifi.txt"
iw dev wlan0 link >> "$LOCAL_OUT/PC_DATA/02_wifi.txt" 2>/dev/null || iw dev wlo1 link >> "$LOCAL_OUT/PC_DATA/02_wifi.txt" 2>/dev/null || true
echo '--- nmcli d wifi ---' >> "$LOCAL_OUT/PC_DATA/02_wifi.txt"
nmcli d wifi >> "$LOCAL_OUT/PC_DATA/02_wifi.txt" 2>/dev/null || true

# ==========================================
# 4. FIREWALL
# ==========================================
echo "4/8: Generating 03_firewall.txt (PC & Robot)..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "echo '--- ufw status ---' > $REMOTE_OUT/03_firewall.txt && sudo ufw status verbose >> $REMOTE_OUT/03_firewall.txt 2>/dev/null || echo 'UFW not installed or permission denied' >> $REMOTE_OUT/03_firewall.txt && echo '--- iptables ---' >> $REMOTE_OUT/03_firewall.txt && sudo iptables -L -n >> $REMOTE_OUT/03_firewall.txt 2>/dev/null || echo 'Iptables permission denied' >> $REMOTE_OUT/03_firewall.txt"

echo '--- ufw status ---' > "$LOCAL_OUT/PC_DATA/03_firewall.txt"
sudo ufw status verbose >> "$LOCAL_OUT/PC_DATA/03_firewall.txt" 2>/dev/null || echo 'UFW not installed or permission denied' >> "$LOCAL_OUT/PC_DATA/03_firewall.txt"
echo '--- iptables ---' >> "$LOCAL_OUT/PC_DATA/03_firewall.txt"
sudo iptables -L -n >> "$LOCAL_OUT/PC_DATA/03_firewall.txt" 2>/dev/null || echo 'Iptables permission denied' >> "$LOCAL_OUT/PC_DATA/03_firewall.txt"

# ==========================================
# 5. SOCKETS & PROCESSES
# ==========================================
echo "5/8: Generating 04_sockets_processes.txt (PC & Robot)..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "echo '--- ss -ulnp (UDP) ---' > $REMOTE_OUT/04_sockets_processes.txt && sudo ss -ulnp >> $REMOTE_OUT/04_sockets_processes.txt 2>/dev/null || true && echo '--- ss -tlnp (TCP) ---' >> $REMOTE_OUT/04_sockets_processes.txt && sudo ss -tlnp >> $REMOTE_OUT/04_sockets_processes.txt 2>/dev/null || true && echo '--- ROS 2 Processes ---' >> $REMOTE_OUT/04_sockets_processes.txt && ps aux | grep -i ros >> $REMOTE_OUT/04_sockets_processes.txt 2>/dev/null || true"

echo '--- ss -ulnp (UDP) ---' > "$LOCAL_OUT/PC_DATA/04_sockets_processes.txt"
sudo ss -ulnp >> "$LOCAL_OUT/PC_DATA/04_sockets_processes.txt" 2>/dev/null || true
echo '--- ss -tlnp (TCP) ---' >> "$LOCAL_OUT/PC_DATA/04_sockets_processes.txt"
sudo ss -tlnp >> "$LOCAL_OUT/PC_DATA/04_sockets_processes.txt" 2>/dev/null || true
echo '--- ROS 2 Processes ---' >> "$LOCAL_OUT/PC_DATA/04_sockets_processes.txt"
ps aux | grep -i ros >> "$LOCAL_OUT/PC_DATA/04_sockets_processes.txt" 2>/dev/null || true

# ==========================================
# 6. CONNECTIVITY (Bidirectional)
# ==========================================
echo "6/8: Generating 05_connectivity.txt (PC & Robot)..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "echo '--- ping PC ($PC_IP) ---' > $REMOTE_OUT/05_connectivity.txt && ping -c 10 $PC_IP >> $REMOTE_OUT/05_connectivity.txt 2>&1 || true && echo '--- ping Gateway/Router ---' >> $REMOTE_OUT/05_connectivity.txt && ping -c 10 \$(ip route | grep default | awk '{print \$3}') >> $REMOTE_OUT/05_connectivity.txt 2>&1 || true"

echo "--- ping Robot ($ROBOT_IP) ---" > "$LOCAL_OUT/PC_DATA/05_connectivity.txt"
ping -c 10 $ROBOT_IP >> "$LOCAL_OUT/PC_DATA/05_connectivity.txt" 2>&1 || true
echo '--- ping Gateway/Router ---' >> "$LOCAL_OUT/PC_DATA/05_connectivity.txt"
ping -c 10 $(ip route | grep default | awk '{print $3}' | head -n1) >> "$LOCAL_OUT/PC_DATA/05_connectivity.txt" 2>&1 || true

# ==========================================
# 7. MULTICAST & TCPDUMP (Dual Sided)
# ==========================================
echo "7/8: Multicast tests and Dual TCPDump Captures (Wait ~15s)..."
ROBOT_WIFI_IF=$(ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "ip route | grep default | awk '{print \$5}' | head -n1" || echo "wlan0")
PC_WIFI_IF=$(ip route | grep default | awk '{print $5}' | head -n1)

# Robot Packet Captures
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "sudo tcpdump -i $ROBOT_WIFI_IF udp portrange 7400-7600 -w $REMOTE_OUT/udp_7400_7600_robot.pcap -G 12 -W 1 >/dev/null 2>&1 &"
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "sudo tcpdump -i $ROBOT_WIFI_IF udp port 11811 -w $REMOTE_OUT/udp_11811_robot.pcap -G 12 -W 1 >/dev/null 2>&1 &"
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "sudo tcpdump -i $ROBOT_WIFI_IF dst 239.255.0.1 -w $REMOTE_OUT/udp_multicast_robot.pcap -G 12 -W 1 >/dev/null 2>&1 &"

# PC Packet Captures
sudo tcpdump -i $PC_WIFI_IF udp portrange 7400-7600 -w "$LOCAL_OUT/PC_DATA/udp_7400_7600_pc.pcap" -G 12 -W 1 >/dev/null 2>&1 &
sudo tcpdump -i $PC_WIFI_IF udp port 11811 -w "$LOCAL_OUT/PC_DATA/udp_11811_pc.pcap" -G 12 -W 1 >/dev/null 2>&1 &
sudo tcpdump -i $PC_WIFI_IF dst 239.255.0.1 -w "$LOCAL_OUT/PC_DATA/udp_multicast_pc.pcap" -G 12 -W 1 >/dev/null 2>&1 &

# Try ROS 2 Multicast Receive on BOTH
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "source /opt/ros/jazzy/setup.bash && timeout 10 ros2 multicast receive > $REMOTE_OUT/06_ros2_multicast_receive.txt 2>&1 &"
timeout 10 /bin/bash -c "source /opt/ros/jazzy/setup.bash && ros2 multicast receive > \"$LOCAL_OUT/PC_DATA/06_ros2_multicast_receive.txt\" 2>&1" &
sleep 4

# Send Multicast from BOTH
echo "     Sending UDP Multicast crossovers..."
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "source /opt/ros/jazzy/setup.bash && ros2 multicast send > $REMOTE_OUT/07_ip_multicast_send.txt 2>&1 || true"
/bin/bash -c "source /opt/ros/jazzy/setup.bash && ros2 multicast send > \"$LOCAL_OUT/PC_DATA/07_ip_multicast_send.txt\" 2>&1 || true"

sleep 12 # wait for tcpdump to neatly close via timeout (-G 12)

# ==========================================
# 8. FINALIZE & SYNC
# ==========================================
echo "8/8: Syncing data from Robot and generating final summaries..."
# Fix permissions for pcaps so we can download them
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "sudo chmod 666 $REMOTE_OUT/*.pcap 2>/dev/null || true"
sudo chmod 666 "$LOCAL_OUT/PC_DATA/"*.pcap 2>/dev/null || true

ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "echo '--- PCAP File Sizes ---' > $REMOTE_OUT/08_tcpdump_info.txt && ls -lh $REMOTE_OUT/*.pcap >> $REMOTE_OUT/08_tcpdump_info.txt 2>/dev/null || true"

echo '--- PCAP File Sizes ---' > "$LOCAL_OUT/PC_DATA/08_tcpdump_info.txt"
ls -lh "$LOCAL_OUT/PC_DATA/"*.pcap >> "$LOCAL_OUT/PC_DATA/08_tcpdump_info.txt" 2>/dev/null || true

echo "Downloading Robot logs to $LOCAL_OUT/ROBOT_DATA..."
scp -o BatchMode=yes -r $ROBOT_USER@$ROBOT_IP:$REMOTE_OUT/* "$LOCAL_OUT/ROBOT_DATA/"
ssh -o BatchMode=yes $ROBOT_USER@$ROBOT_IP "rm -rf $REMOTE_OUT"

echo "======================================================"
echo " Mass Diagnostic successfully gathered!               "
echo " You can view the contents via your file explorer at: "
echo " $LOCAL_OUT "
echo "======================================================"
