#!/bin/bash
set -e

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOCAL_OUT="/home/mhc/Downloads/massdiag_pc_only_${TIMESTAMP}"
mkdir -p "$LOCAL_OUT"

echo "======================================================"
echo "    MASS DIAGNOSTIC SCRIPT (PC ONLY)                  "
echo "======================================================"
echo "Output Directory: $LOCAL_OUT"
echo ""

echo "1/7: Generating 00_summary.txt..."
cat << 'EOF' > /tmp/gen_summary_pc.sh
echo "=== Mass Diagnostic Summary (PC) ==="
echo "Date: $(date)"
echo "Hostname: $(hostname)"
echo "Uptime: $(uptime -p)"
echo "CPU Architecture: $(uname -m)"
echo "OS Version: $(cat /etc/os-release | grep PRETTY_NAME | cut -d '=' -f 2 | tr -d '\"')"
echo ""
echo "=== IP Addresses ==="
ip addr show | grep 'inet ' | awk '{print $NF, $2}' | column -t
EOF
bash /tmp/gen_summary_pc.sh > "$LOCAL_OUT/00_summary.txt"

echo "2/7: Generating 01_network.txt..."
echo '--- ip route ---' > "$LOCAL_OUT/01_network.txt"
ip route >> "$LOCAL_OUT/01_network.txt"
echo '--- ip neigh ---' >> "$LOCAL_OUT/01_network.txt"
ip neigh >> "$LOCAL_OUT/01_network.txt"
echo '--- ifconfig ---' >> "$LOCAL_OUT/01_network.txt"
ifconfig -a >> "$LOCAL_OUT/01_network.txt" 2>/dev/null || true

echo "3/7: Generating 02_wifi.txt..."
echo '--- iwconfig ---' > "$LOCAL_OUT/02_wifi.txt"
iwconfig >> "$LOCAL_OUT/02_wifi.txt" 2>/dev/null || true
echo '--- iw dev link ---' > "$LOCAL_OUT/02_wifi.txt"
iw dev wlan0 link >> "$LOCAL_OUT/02_wifi.txt" 2>/dev/null || iw dev wlo1 link >> "$LOCAL_OUT/02_wifi.txt" 2>/dev/null || true
echo '--- nmcli d wifi ---' >> "$LOCAL_OUT/02_wifi.txt"
nmcli d wifi >> "$LOCAL_OUT/02_wifi.txt" 2>/dev/null || true

echo "4/7: Generating 03_firewall.txt..."
echo '--- ufw status ---' > "$LOCAL_OUT/03_firewall.txt"
sudo ufw status verbose >> "$LOCAL_OUT/03_firewall.txt" 2>/dev/null || echo 'UFW not installed or permission denied' >> "$LOCAL_OUT/03_firewall.txt"
echo '--- iptables ---' >> "$LOCAL_OUT/03_firewall.txt"
sudo iptables -L -n >> "$LOCAL_OUT/03_firewall.txt" 2>/dev/null || echo 'Iptables permission denied' >> "$LOCAL_OUT/03_firewall.txt"

echo "5/7: Generating 04_sockets_processes.txt..."
echo '--- ss -ulnp (UDP) ---' > "$LOCAL_OUT/04_sockets_processes.txt"
sudo ss -ulnp >> "$LOCAL_OUT/04_sockets_processes.txt" 2>/dev/null || true
echo '--- ss -tlnp (TCP) ---' >> "$LOCAL_OUT/04_sockets_processes.txt"
sudo ss -tlnp >> "$LOCAL_OUT/04_sockets_processes.txt" 2>/dev/null || true

echo "6/7: Generating 05_connectivity.txt..."
echo '--- ping Gateway/Router ---' > "$LOCAL_OUT/05_connectivity.txt"
ping -c 10 $(ip route | grep default | awk '{print $3}' | head -n1) >> "$LOCAL_OUT/05_connectivity.txt" 2>&1 || true

echo "7/7: Multicast tests and TCPDump Captures (Wait ~15s)..."
PC_WIFI_IF=$(ip route | grep default | awk '{print $5}' | head -n1)

# PC Packet Captures
sudo tcpdump -i $PC_WIFI_IF udp portrange 7400-7600 -w "$LOCAL_OUT/udp_7400_7600_pc.pcap" -G 12 -W 1 >/dev/null 2>&1 &
sudo tcpdump -i $PC_WIFI_IF udp port 11811 -w "$LOCAL_OUT/udp_11811_pc.pcap" -G 12 -W 1 >/dev/null 2>&1 &
sudo tcpdump -i $PC_WIFI_IF dst 239.255.0.1 -w "$LOCAL_OUT/udp_multicast_pc.pcap" -G 12 -W 1 >/dev/null 2>&1 &

sleep 12 # wait for tcpdump to neatly close via timeout (-G 12)

# Fix permissions
sudo chmod 666 "$LOCAL_OUT/"*.pcap 2>/dev/null || true

echo '--- PCAP File Sizes ---' > "$LOCAL_OUT/08_tcpdump_info.txt"
ls -lh "$LOCAL_OUT/"*.pcap >> "$LOCAL_OUT/08_tcpdump_info.txt" 2>/dev/null || true

echo "======================================================"
echo " Mass Diagnostic (PC ONLY) successfully gathered!     "
echo " You can view the contents via your file explorer at: "
echo " $LOCAL_OUT "
echo "======================================================"
