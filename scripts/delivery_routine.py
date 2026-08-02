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
        if getattr(self, 'wait_vision_active', False):
            self.latest_product_class = msg.data.strip().lower()

    # ── Entrada da rotina ────────────────────────────────────────

    def start(self, on_complete=None):
        self._on_complete = on_complete
        self.get_logger().info("=== Delivery: iniciando ===")
        self.undock(
            lambda ok: self._go_pickup()
            if ok
            else self.finish_mission(False, "Delivery: undock falhou")
        )

    # ── Passos da missão ─────────────────────────────────────────

    def _go_pickup(self):
        self.navigate_to(
            "pickup_point",
            lambda ok: self._go_deliver("delivery_red")
            if ok
            else self.finish_mission(False, "Delivery: falhou em pickup_point"),
        )

    def _go_deliver(self, target: str):
        self.navigate_to(
            target,
            lambda ok: self.return_to_dock(lambda dock_ok: self.finish_mission(dock_ok))
            if ok
            else self.finish_mission(False, f"Delivery: falhou em {target}"),
        )
