#!/usr/bin/env python3
"""
restock_routine.py
──────────────────
Rotina de Restock:
  undock → inventory_point → supply_point → return_to_dock
"""
from mission_base import MissionBase


class RestockRoutine(MissionBase):
    def __init__(self, node_name="restock_routine"):
        super().__init__(node_name)

    # ── Entrada da rotina ────────────────────────────────────────

    def start(self):
        self.get_logger().info("=== Restock: iniciando ===")
        self.undock(
            lambda ok: self._go_inventory()
            if ok
            else self.get_logger().error("Restock: undock falhou")
        )

    # ── Passos da missão ─────────────────────────────────────────

    def _go_inventory(self):
        self.navigate_to(
            "inventory_point",
            lambda ok: self._go_supply()
            if ok
            else self.get_logger().error("Restock: falhou em inventory_point"),
        )

    def _go_supply(self):
        self.navigate_to(
            "supply_point",
            lambda ok: self.return_to_dock(lambda _: None)
            if ok
            else self.get_logger().error("Restock: falhou em supply_point"),
        )
