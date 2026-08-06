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
        val = msg.data.strip().lower()
        self.get_logger().info(f"Tópico /product_class recebido no TurtleBot 4: '{val}'")
        self.latest_product_class = val

    # ── Entrada da rotina ────────────────────────────────────────

    def start(self, on_complete=None):
        self._on_complete = on_complete
        self.start_mission_watchdog(180.0)
        self.get_logger().info("=== Delivery: iniciando ===")
        self.undock(
            lambda ok: self._go_pickup()
            if ok
            else self.finish_mission(False, "Delivery: undock falhou")
        )

    # ── Passos da missão ─────────────────────────────────────────

    def _go_pickup(self):
        def _on_pickup_reached(ok: bool):
            if not ok:
                self.finish_mission(False, "Delivery: falhou em pickup_point")
                return
            
            # Determina o waypoint de destino dinamicamente com base na cor/classe da lata
            cls = (self.latest_product_class or "").lower()
            if "blue" in cls or "azul" in cls:
                target_wp = "delivery_blue"
            elif "red" in cls or "vermelh" in cls:
                target_wp = "delivery_red"
            else:
                target_wp = "delivery_red"  # fallback padrão

            self.get_logger().info(f"Classe detectada: '{cls}' -> Navegando para '{target_wp}'")
            self._go_deliver(target_wp)

        self.navigate_to("pickup_point", _on_pickup_reached)

    def _go_deliver(self, target: str):
        self.navigate_to(
            target,
            lambda ok: self.return_to_dock(lambda dock_ok: self.finish_mission(dock_ok))
            if ok
            else self.finish_mission(False, f"Delivery: falhou em {target}"),
        )
