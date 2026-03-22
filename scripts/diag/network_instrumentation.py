#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
import argparse
import sys

class NetworkTester(Node):
    def __init__(self, mode, payload_size):
        super().__init__('network_tester_' + mode)
        self.mode = mode
        self.payload_size = payload_size
        self.seq = 0
        
        if mode == 'client':
            self.publisher_ = self.create_publisher(String, '/network_ping', 10)
            self.subscription = self.create_subscription(String, '/network_pong', self.listener_callback, 10)
            self.timer = self.create_timer(1.0, self.timer_callback) # 1 Hz
            self.get_logger().info(f'Client started. Sending {self.payload_size} bytes payload.')
        elif mode == 'server':
            self.publisher_ = self.create_publisher(String, '/network_pong', 10)
            self.subscription = self.create_subscription(String, '/network_ping', self.listener_callback, 10)
            self.get_logger().info('Server started. Waiting for pings from client...')
            
        self.latencies = []
        self.last_seq_received = 0
        self.packets_lost = 0

    def timer_callback(self):
        self.seq += 1
        msg = String()
        data = {
            'seq': self.seq,
            'tx_time': time.time(),
            'payload': 'X' * self.payload_size
        }
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info(f'[TX] msg seq={self.seq} | size={len(msg.data)} bytes')

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON received')
            return
            
        if self.mode == 'server':
            # Server just echoes back the exact same data immediately (but updates server_rx_time)
            data['server_rx_time'] = time.time()
            reply = String()
            reply.data = json.dumps(data)
            self.publisher_.publish(reply)
            self.get_logger().info(f'[RX -> TX] Echoed seq={data["seq"]}')
            
        elif self.mode == 'client':
            # Client calculates the round trip time
            rx_time = time.time()
            tx_time = data['tx_time']
            seq = data['seq']
            
            # Detect drops
            if self.last_seq_received != 0 and seq > self.last_seq_received + 1:
                dropped = seq - self.last_seq_received - 1
                self.packets_lost += dropped
                self.get_logger().warn(f'PACKET DROP DETECTED! Lost {dropped} packets.')
            
            self.last_seq_received = seq
            
            rtt = (rx_time - tx_time) * 1000.0 # convert to ms
            
            self.latencies.append(rtt)
            avg_rtt = sum(self.latencies) / len(self.latencies)
            
            loss_rate = (self.packets_lost / seq) * 100.0 if seq > 0 else 0.0
            
            self.get_logger().info(
                f'[RX] Reply seq={seq} | RTT={rtt:.2f} ms | '
                f'Avg RTT={avg_rtt:.2f} ms | Loss Rate={loss_rate:.1f}%'
            )

def main(args=None):
    # We use sys.argv to allow ROS arguments to pass through, but handle our specific args first
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['client', 'server'], required=True, help='Run as client (sender) or server (echo)')
    parser.add_argument('--size', type=int, default=100, help='Payload size in bytes (client only)')
    
    parsed_args, unknown_args = parser.parse_known_args()

    # Pass the remaining unknown arguments to rclpy
    rclpy.init(args=[sys.argv[0]] + unknown_args)
    tester = NetworkTester(parsed_args.mode, parsed_args.size)
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
