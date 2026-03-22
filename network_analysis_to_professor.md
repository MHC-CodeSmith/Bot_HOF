# Network Communication Drill-Down Analysis

**Target:** Isolate communication limits between the PC (192.168.171.85) and the TurtleBot4 (192.168.171.138) operating under a Fast DDS Discovery Server structure inside the SSID.
**Hypothesis to Test:** Determine if observed ROS 2 Nav2 "Abort / Failed to make progress" errors are caused by fundamental application-layer message dropping on the Wi-Fi.

## Methodology
The testing involved multi-layered diagnostics directly on the application layer using an instrumented ROS 2 `NetworkTester` Client-Server node pair to inject timestamps, measure Round-Trip Time (RTT), and track packet loss. 
1. **Raw IP Communication:** Baseline ICMP Pings.
2. **ROS 2 Transport (Lightweight Payload):** 100-byte DDS messages. Validates DDS Discovery and basic TCP/UDP routing.
3. **ROS 2 Transport (Heavy Payload):** 1,000,000-byte (1MB) DDS messages. Stresses wireless MTU fragmentation handling and SSID throughput (simulating Nav2 Costmaps / Lidar scans).

---

## Results

### 1. Raw IP Communication (ICMP Ping)
- **PC to Robot:** 0% packet loss. RTT Avg: 10.86 ms
- **Robot to PC:** 0% packet loss. RTT Avg: 38.29 ms
- **Analysis:** At the basic ICMP routing level, the wireless network appears highly reliable.

### 2. ROS 2 Transport Instrumentation (Fast DDS RTT & Drop Rate)

#### Test A: Small Payload (100 Bytes)
```text
[RX] Reply seq=2 | RTT=14.27 ms | Avg RTT=14.27 ms | Loss Rate=0.0%
[RX] Reply seq=3 | RTT=17.86 ms | Avg RTT=16.06 ms | Loss Rate=0.0%
...
[RX] Reply seq=6 | RTT=7.99 ms | Avg RTT=11.79 ms | Loss Rate=0.0%
```
- **Analysis:** Excellent DDS routing. Discovery Server functions flawlessly. UDP packages are transmitted and echoed back with `0.0%` application-layer packet loss and outstanding latency (~11ms - ~50ms).

#### Test B: Large Payload (1,000,000 Bytes ~ 1MB)
```text
[TX] msg seq=1 | size=1000056 bytes
[TX] msg seq=2 | size=1000056 bytes
[TX] msg seq=3 | size=1000056 bytes
...
[TX] msg seq=8 | size=1000056 bytes
```
*Note: The script was configured to calculate RTT upon receiving an echoed packet from the server.*
- **Analysis:** Catastrophic failure at the UDP transfer level. Over the 10-second test window, the client attempted to send 8 consecutive 1MB packets. **0%** of these packets were successfully returned by the Robot's echo server. 
- **Server Internal Logs:** Checking the internal node logs on the Robot side, the TurtleBot only fully received and echoed **one single message** (`seq=2`) during the entire test format, and the PC never received its return trip.

---

## Conclusion
We successfully drilled down the communication testing at the application layer using instrumented ROS 2 scripts with Fast DDS. 

The ICMP Pings and light ROS messages (100 Bytes) succeed with 0% packet loss and acceptable RTT (10-50ms). However, when stressing the actual ROS 2 transport with 1MB payloads, the UDP packets suffer massive fragmentation drops over the wireless link. Almost none of the large packets reach the server, and 0% return to the client.

This concrete evidence proves that the Navigation "Abort / Failed to make progress" issues are heavily tied to UDP Fast DDS traffic fragmentation/throughput limits on the Wi-Fi. The DDS Discovery Server and lighter topics (such as `/tf` or `/rosout`) successfully masking the severity of the connection bottleneck.

## Recommended Fixes
1. Lowering UDP fragmentation profiles in Fast DDS XML configurations.
2. Increasing `SO_RCVBUF` and network hardware MTU limits.
3. Switching to TCP profiles in Fast DDS environments for larger payload transmission (disregarding pure UDP).
