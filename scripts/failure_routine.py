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

    def start(self, on_complete=None):
        self._on_complete = on_complete
        self.start_mission_watchdog(180.0)
        self.get_logger().info("=== Failure: iniciando ===")
        self.undock(
            lambda ok: self._go_failure_pickup()
            if ok
            else self.finish_mission(False, "Failure: undock falhou")
        )

    # ── Passos da missão ─────────────────────────────────────────

    def _go_failure_pickup(self):
        self.navigate_to(
            "failure_pickup",
            lambda ok: self._go_failure_zone()
            if ok
            else self.finish_mission(False, "Failure: falhou em failure_pickup"),
        )

    def _go_failure_zone(self):
        self.navigate_to(
            "failure_zone",
            lambda ok: self.return_to_dock(lambda dock_ok: self.finish_mission(dock_ok))
            if ok
            else self.finish_mission(False, "Failure: falhou em failure_zone"),
        )
