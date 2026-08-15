#!/usr/bin/env python3
"""
delivery_routine.py
───────────────────
Rotina de Delivery:
  classe fresca → undock → pickup_point → handshake → delivery_red/blue → dock
"""
import time
from typing import Optional

from std_msgs.msg import Bool, String

from mission_base import MissionBase


def delivery_target_for_class(product_class: str) -> Optional[str]:
    """Traduz somente classes explicitamente reconhecidas em um destino."""
    normalized = product_class.strip().lower().replace("-", "_").replace(" ", "_")
    red_classes = {
        "red", "vermelho", "vermelha", "tin_valid_red", "tin_valid_red_square"
    }
    blue_classes = {
        "blue", "azul", "tin_valid_blue", "tin_valid_blue_square"
    }
    if normalized in red_classes:
        return "delivery_red"
    if normalized in blue_classes:
        return "delivery_blue"
    return None


def classification_is_fresh(
    received_at: Optional[float], now: float, max_age_s: float
) -> bool:
    return received_at is not None and 0.0 <= now - received_at <= max_age_s


class DeliveryRoutine(MissionBase):
    def __init__(self, node_name="delivery_routine"):
        super().__init__(node_name)

        self.declare_parameter("vision_timeout_s", 10.0)
        self.declare_parameter("product_class_max_age_s", 5.0)
        self.declare_parameter("handoff_timeout_s", 90.0)
        self.declare_parameter("pickup_announce_period_s", 0.5)
        self.vision_timeout: float = float(
            self.get_parameter("vision_timeout_s").value
        )
        self.product_class_max_age_s = float(
            self.get_parameter("product_class_max_age_s").value
        )
        self.handoff_timeout_s = float(self.get_parameter("handoff_timeout_s").value)
        self.pickup_announce_period_s = float(
            self.get_parameter("pickup_announce_period_s").value
        )
        invalid = [
            name
            for name, value in (
                ("vision_timeout_s", self.vision_timeout),
                ("product_class_max_age_s", self.product_class_max_age_s),
                ("handoff_timeout_s", self.handoff_timeout_s),
                ("pickup_announce_period_s", self.pickup_announce_period_s),
            )
            if value <= 0
        ]
        if invalid:
            raise ValueError(f"Parâmetros devem ser maiores que zero: {', '.join(invalid)}")

        self.latest_product_class: str | None = None
        self.latest_product_class_monotonic: Optional[float] = None
        self._mission_delivery_target: Optional[str] = None
        self._classification_timer = None
        self._handoff_timer = None
        self._waiting_for_item_release = False
        self._item_released_monotonic: Optional[float] = None
        self.create_subscription(
            String, "/product_class", self._on_product_class, 10
        )
        self.create_subscription(
            Bool, "/item_released_on_tb4", self._on_item_released, 10
        )
        self._pickup_pub = self.create_publisher(Bool, "/turtlebot_at_pickup", 10)

    # ── Callbacks ───────────────────────────────────────────────

    def _on_product_class(self, msg: String):
        val = msg.data.strip().lower()
        self.get_logger().info(f"Tópico /product_class recebido no TurtleBot 4: '{val}'")
        self.latest_product_class = val
        self.latest_product_class_monotonic = time.monotonic()

    def _on_item_released(self, msg: Bool):
        if msg.data and self._waiting_for_item_release:
            self._item_released_monotonic = time.monotonic()
            self.get_logger().info("Confirmação /item_released_on_tb4 recebida")

    # ── Entrada da rotina ────────────────────────────────────────

    def start(self, on_complete=None):
        self.begin_mission(on_complete, watchdog_timeout_s=180.0)
        self._cancel_delivery_timers()
        self._mission_delivery_target = None
        self._waiting_for_item_release = False
        self._item_released_monotonic = None
        self.get_logger().info("=== Delivery: iniciando ===")

        target = self._fresh_delivery_target()
        if target is not None:
            self._start_with_target(target)
            return

        self.get_logger().info(
            f"Aguardando classe válida em /product_class por até {self.vision_timeout:.1f}s; nenhum movimento será iniciado antes disso"
        )
        deadline = time.monotonic() + self.vision_timeout

        def _check_classification() -> None:
            target_now = self._fresh_delivery_target()
            if target_now is not None:
                self._cancel_timer("_classification_timer")
                self._start_with_target(target_now)
            elif time.monotonic() >= deadline:
                received = self.latest_product_class or "nenhuma"
                self._cancel_timer("_classification_timer")
                self.finish_mission(
                    False,
                    f"Delivery recusado: classe válida e fresca não recebida (última='{received}')",
                )

        self._classification_timer = self.create_timer(
            0.1, _check_classification, callback_group=self._cb_group
        )

    def _fresh_delivery_target(self) -> Optional[str]:
        if not classification_is_fresh(
            self.latest_product_class_monotonic,
            time.monotonic(),
            self.product_class_max_age_s,
        ):
            return None
        return delivery_target_for_class(self.latest_product_class or "")

    def _start_with_target(self, target: str) -> None:
        if self._finish_started:
            return
        self._mission_delivery_target = target
        self.get_logger().info(
            f"Classe válida e fresca confirmada; destino desta missão: '{target}'"
        )
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
            self._wait_for_item_release()

        self.navigate_to("pickup_point", _on_pickup_reached)

    def _wait_for_item_release(self) -> None:
        target = self._mission_delivery_target
        if target is None:
            self.finish_mission(False, "Delivery sem destino medido; navegação recusada")
            return

        self._waiting_for_item_release = True
        self._item_released_monotonic = None
        deadline = time.monotonic() + self.handoff_timeout_s
        next_announcement = {"at": 0.0}
        self.get_logger().info(
            f"Pickup alcançado; aguardando /item_released_on_tb4 por até {self.handoff_timeout_s:.1f}s"
        )

        def _check_handoff() -> None:
            now = time.monotonic()
            if now >= next_announcement["at"]:
                self._pickup_pub.publish(Bool(data=True))
                next_announcement["at"] = now + self.pickup_announce_period_s
            if self._item_released_monotonic is not None:
                self._waiting_for_item_release = False
                self._cancel_timer("_handoff_timer")
                self.get_logger().info(
                    f"Handoff confirmado; iniciando navegação para '{target}'"
                )
                self._go_deliver(target)
            elif now >= deadline:
                self._waiting_for_item_release = False
                self._cancel_timer("_handoff_timer")
                self.finish_mission(
                    False,
                    "Delivery: timeout aguardando confirmação de liberação da lata",
                )

        self._handoff_timer = self.create_timer(
            0.1, _check_handoff, callback_group=self._cb_group
        )

    def _cancel_timer(self, attribute: str) -> None:
        """Cancela um timer sem destruir sua entidade dentro de um callback.

        A rotina usa um executor multithread. ``destroy_timer`` enquanto o
        próprio timer (ou outro callback) ainda está na wait-set pode gerar
        ``rclpy._rclpy_pybind11.InvalidHandle`` e derrubar todo o Mission
        Manager. O nó mantém os timers cancelados até seu encerramento, quando
        o rclpy faz a destruição em um ponto seguro.
        """
        timer = getattr(self, attribute, None)
        setattr(self, attribute, None)
        if timer is not None:
            try:
                timer.cancel()
            except Exception:
                pass

    def _cancel_delivery_timers(self) -> None:
        self._cancel_timer("_classification_timer")
        self._cancel_timer("_handoff_timer")

    def finish_mission(self, success: bool, msg: str = ""):
        self._waiting_for_item_release = False
        self._cancel_delivery_timers()
        super().finish_mission(success, msg)

    def _go_deliver(self, target: str):
        self.navigate_to(
            target,
            lambda ok: self.return_to_dock(lambda dock_ok: self.finish_mission(dock_ok))
            if ok
            else self.finish_mission(False, f"Delivery: falhou em {target}"),
        )
