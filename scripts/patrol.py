#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String
from std_srvs.srv import Trigger

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

from irobot_create_msgs.action import Dock, Undock


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float  # radians


def yaw_to_quat_z_w(yaw: float):
    """2D yaw -> quaternion (z,w)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        # ===== Params =====
        # Waypoints are loaded from the provided params file (waypoints.yaml)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("pickup_station", [-0.68, -2.55, 0.0])
        self.declare_parameter("client_red", [0.80, -1.20, 0.0])
        self.declare_parameter("client_blue", [0.80, -2.20, 0.0])
        self.declare_parameter("predock", [-0.56, 0.0, 0.0])
        self.declare_parameter("dock_tries", 2)
        self.declare_parameter("vision_timeout_s", 10.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.waypoints: Dict[str, Waypoint] = {
            "pickup_station": self._wp_from_param("pickup_station"),
            "client_red": self._wp_from_param("client_red"),
            "client_blue": self._wp_from_param("client_blue"),
            "predock": self._wp_from_param("predock"),
        }
        self.dock_tries = int(self.get_parameter("dock_tries").value)
        self.vision_timeout = float(self.get_parameter("vision_timeout_s").value)

        # ===== Action clients =====
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.dock_client = ActionClient(self, Dock, "/dock")
        self.undock_client = ActionClient(self, Undock, "/undock")

        # ===== Vision subscription =====
        # Vision node should publish: "red" or "blue" (or "unknown") to /product_class
        self.latest_product_class: Optional[str] = None
        self.create_subscription(String, "/product_class", self._on_product_class, 10)

        # ===== Simple trigger to test =====
        self.create_service(Trigger, "/start_delivery", self._srv_start_delivery)

        self.get_logger().info("MissionManager ready. Call: ros2 service call /start_delivery std_srvs/srv/Trigger {}")

    def _wp_from_param(self, name: str) -> Waypoint:
        v = self.get_parameter(name).value  # [x,y,yaw]
        return Waypoint(float(v[0]), float(v[1]), float(v[2]))

    def _on_product_class(self, msg: String):
        self.latest_product_class = msg.data.strip().lower()

    # ---------- Service callback ----------
    def _srv_start_delivery(self, req, resp):
        # Trigger mission in a timer to avoid blocking the service callback
        self.create_timer(0.01, self._start_once)
        resp.success = True
        resp.message = "Mission triggered."
        return resp

    def _start_once(self):
        # ensures it runs only once per trigger
        self.destroy_timer(self.timers[0])
        self.timers.clear()
        self._run_mission_async()

    def _run_mission_async(self):
        self.get_logger().info("=== Starting Mission: Pickup -> Classify -> Deliver -> Dock ===")

        # 1) Undock if needed
        self._undock_then(lambda ok: self._go_pickup() if ok else self.get_logger().error("Undock failed"))

    def _go_pickup(self):
        self.get_logger().info("Heading to pickup_station...")
        self._navigate_to("pickup_station", lambda ok: self._wait_vision_and_deliver() if ok else self.get_logger().error("Failed to reach pickup station"))

    def _wait_vision_and_deliver(self):
        self.get_logger().info("Waiting for product classification (/product_class)...")
        self.latest_product_class = None

        deadline = self.get_clock().now() + Duration(seconds=self.vision_timeout)

        def poll():
            if self.latest_product_class in ("red", "blue"):
                self.get_logger().info(f"Product detected: {self.latest_product_class}")
                self.destroy_timer(t)
                target = "client_red" if self.latest_product_class == "red" else "client_blue"
                self._navigate_to(target, lambda ok: self._return_and_dock() if ok else self.get_logger().error("Delivery failed"))
                return

            if self.get_clock().now() > deadline:
                self.get_logger().warn("Vision timeout. Proceeding with fallback (client_blue).")
                self.destroy_timer(t)
                self._navigate_to("client_blue", lambda ok: self._return_and_dock() if ok else self.get_logger().error("Delivery failed"))
                return

        t = self.create_timer(0.2, poll)

    def _return_and_dock(self):
        self.get_logger().info("Returning to predock...")
        self._navigate_to("predock", lambda ok: self._dock_with_retries(self.dock_tries) if ok else self.get_logger().error("Failed to return to predock"))

    # ---------- Nav2 ----------
    def _navigate_to(self, wp_name: str, done_cb):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action /navigate_to_pose not available.")
            done_cb(False)
            return

        wp = self.waypoints[wp_name]
        qz, qw = yaw_to_quat_z_w(wp.yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = Time().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)

        def _on_goal_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().error(f"Navigation goal rejected for: {wp_name}")
                done_cb(False)
                return
            self.get_logger().info(f"Navigation goal accepted for: {wp_name}")
            result_future = gh.get_result_async()
            result_future.add_done_callback(lambda rf: self._on_nav_result(rf, wp_name, done_cb))

        send_future.add_done_callback(_on_goal_sent)

    def _on_nav_result(self, fut, wp_name: str, done_cb):
        status = fut.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info(f"Reached {wp_name}.")
            done_cb(True)
        else:
            self.get_logger().error(f"Failed to reach {wp_name} with status: {status}")
            done_cb(False)

    # ---------- Dock / Undock ----------
    def _undock_then(self, done_cb):
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Action /undock not available. Proceeding without undock.")
            done_cb(True)
            return

        goal = Undock.Goal()
        send_future = self.undock_client.send_goal_async(goal)

        def _on_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("Undock goal rejected. Proceeding.")
                done_cb(True)
                return
            self.get_logger().info("Undock goal sent.")
            gh.get_result_async().add_done_callback(lambda _: done_cb(True))

        send_future.add_done_callback(_on_sent)

    def _dock_with_retries(self, tries_left: int):
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action /dock not available.")
            return

        self.get_logger().info(f"Attempting to dock... ({tries_left} attempts remaining)")
        goal = Dock.Goal()
        send_future = self.dock_client.send_goal_async(goal)

        def _on_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("Dock goal rejected.")
                if tries_left > 1:
                    self._dock_with_retries(tries_left - 1)
                return

            self.get_logger().info("Dock goal sent.")
            gh.get_result_async().add_done_callback(lambda fut_result: self._dock_done(fut_result, tries_left))

        send_future.add_done_callback(_on_sent)

    def _dock_done(self, fut_result, tries_left: int):
        status = fut_result.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info("Docking successful. ✅")
        else:
            self.get_logger().warn(f"Docking failed with status: {status}")
            if tries_left > 1:
                self._dock_with_retries(tries_left - 1)
            else:
                self.get_logger().error("Max docking retries reached.")

def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
