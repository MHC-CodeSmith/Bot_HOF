#!/usr/bin/env python3
"""
failure_routine.py
──────────────────
Rotina de Failure:
  undock → failure_pickup → failure_zone → return_to_dock
"""
from mission_base import MissionBase


class FailureRoutine(MissionBase):
    def __init__(self, node_name="failure_routine"):
        super().__init__(node_name)

    # ── Entrada da rotina ────────────────────────────────────────

    def start(self):
        self.get_logger().info("=== Failure: iniciando ===")
        self.undock(
            lambda ok: self._go_failure_pickup()
            if ok
            else self.get_logger().error("Failure: undock falhou")
        )

    # ── Passos da missão ─────────────────────────────────────────

    def _go_failure_pickup(self):
        self.navigate_to(
            "failure_pickup",
            lambda ok: self._go_failure_zone()
            if ok
            else self.get_logger().error("Failure: falhou em failure_pickup"),
        )

    def _go_failure_zone(self):
        self.navigate_to(
            "failure_zone",
            lambda ok: self.return_to_dock(lambda _: None)
            if ok
            else self.get_logger().error("Failure: falhou em failure_zone"),
        )
