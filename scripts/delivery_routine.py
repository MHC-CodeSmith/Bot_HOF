#!/usr/bin/env python3
"""
delivery_routine.py
───────────────────
Rotina de Delivery:
  undock → pickup_point → (visão → red/blue) → delivery_red/blue → return_to_dock
"""
from std_msgs.msg import String
from rclpy.duration import Duration

from mission_base import MissionBase


class DeliveryRoutine(MissionBase):
    def __init__(self, node_name="delivery_routine"):
        super().__init__(node_name)

        self.declare_parameter("vision_timeout_s", 10.0)
        self.vision_timeout: float = float(
            self.get_parameter("vision_timeout_s").value
        )

        self.latest_product_class: str | None = None
        self.create_subscription(
            String, "/product_class", self._on_product_class, 10
        )

    # ── Callbacks ───────────────────────────────────────────────

    def _on_product_class(self, msg: String):
        self.latest_product_class = msg.data.strip().lower()

    # ── Entrada da rotina ────────────────────────────────────────

    def start(self):
        self.get_logger().info("=== Delivery: iniciando ===")
        self.undock(
            lambda ok: self._go_pickup()
            if ok
            else self.get_logger().error("Delivery: undock falhou")
        )

    # ── Passos da missão ─────────────────────────────────────────

    def _go_pickup(self):
        self.navigate_to(
            "pickup_point",
            lambda ok: self._wait_vision_and_deliver()
            if ok
            else self.get_logger().error("Delivery: falhou em pickup_point"),
        )

    def _wait_vision_and_deliver(self):
        self.get_logger().info("Aguardando classificação do produto (/product_class)...")
        self.latest_product_class = None

        deadline = self.get_clock().now() + Duration(seconds=self.vision_timeout)

        def _poll():
            cls = self.latest_product_class
            if cls in ("red", "blue"):
                self.get_logger().info(f"Produto detectado: {cls}")
                self.destroy_timer(_t)
                target = "delivery_red" if cls == "red" else "delivery_blue"
                self._go_deliver(target)
                return

            if self.get_clock().now() > deadline:
                self.get_logger().warn(
                    "Timeout de visão — usando delivery_red como fallback"
                )
                self.destroy_timer(_t)
                self._go_deliver("delivery_red")

        _t = self.create_timer(0.2, _poll)

    def _go_deliver(self, target: str):
        self.navigate_to(
            target,
            lambda ok: self.return_to_dock(lambda _: None)
            if ok
            else self.get_logger().error(f"Delivery: falhou em {target}"),
        )
