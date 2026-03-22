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
        self.red_count = 0
        self.blue_count = 0

        deadline = self.get_clock().now() + Duration(seconds=self.vision_timeout)

        def _poll():
            cls = self.latest_product_class
            if cls == "red":
                self.red_count += 1
            elif cls == "blue":
                self.blue_count += 1

            # Confirma após 5 leituras iguais (evita ruídos ou falsos positivos momentâneos)
            if self.red_count >= 5:
                self.get_logger().info("Produto CONFIRMADO: red")
                self.destroy_timer(_t)
                self._go_deliver("delivery_red")
                return
            elif self.blue_count >= 5:
                self.get_logger().info("Produto CONFIRMADO: blue")
                self.destroy_timer(_t)
                self._go_deliver("delivery_blue")
                return

            if self.get_clock().now() > deadline:
                self.get_logger().warn(
                    "Timeout: Produto não reconhecido com certeza. Retornando para dock."
                )
                self.destroy_timer(_t)
                self.return_to_dock(lambda _: None)

        _t = self.create_timer(0.2, _poll)

    def _go_deliver(self, target: str):
        self.navigate_to(
            target,
            lambda ok: self.return_to_dock(lambda _: None)
            if ok
            else self.get_logger().error(f"Delivery: falhou em {target}"),
        )
