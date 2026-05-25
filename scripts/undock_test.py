#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock

class UndockClient(Node):
    def __init__(self):
        super().__init__('undock_client_test')
        self.client = ActionClient(self, Undock, '/undock')

    def run(self):
        self.get_logger().info('Waiting for /undock...')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Server not available')
            return

        self.get_logger().info('Sending goal...')
        goal_msg = Undock.Goal()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected or no handle')
            return

        self.get_logger().info('Goal accepted, waiting result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)

        result = result_future.result()
        self.get_logger().info(f'Result: {result}')

def main():
    rclpy.init()
    node = UndockClient()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()