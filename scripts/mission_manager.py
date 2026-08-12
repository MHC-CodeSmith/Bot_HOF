#!/usr/bin/env python3
"""
mission_manager.py
──────────────────
Nó principal do gerenciador de missões.

Serviços disponíveis:
  /start_delivery  →  std_srvs/Trigger
  /start_failure   →  std_srvs/Trigger
  /start_restock   →  std_srvs/Trigger

Uso:
  ros2 run <pkg> mission_manager.py \
    --ros-args --params-file params/waypoints.yaml

Ou diretamente:
  python3 scripts/mission_manager.py \
    --ros-args --params-file params/waypoints.yaml
"""
import sys
import os

# Garante que os outros módulos do mesmo diretório sejam importados
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from delivery_routine import DeliveryRoutine
from failure_routine import FailureRoutine
from restock_routine import RestockRoutine


class MissionManager(Node):
    """
    Nó gerenciador de missões.
    Recebe triggers via service e despacha para a rotina correspondente.
    """

    def __init__(self):
        super().__init__("mission_manager")

        # ── Callback Group Reentrante ──────────────────────────
        self._cb_group = ReentrantCallbackGroup()

        # ── Instancia as rotinas ─────────────────────────────────
        self.delivery = DeliveryRoutine("delivery_routine")
        self.failure = FailureRoutine("failure_routine")
        self.restock = RestockRoutine("restock_routine")

        # ── Flags ─────────────────────────────────────────────────
        self._busy = False
        self._active_routine = None
        self._nav2_ready = False  # bloqueia missões até Nav2 estar pronto

        # ── Services ─────────────────────────────────────────────
        self.create_service(Trigger, "/start_delivery", self._srv_delivery, callback_group=self._cb_group)
        self.create_service(Trigger, "/start_failure", self._srv_failure, callback_group=self._cb_group)
        self.create_service(Trigger, "/start_restock", self._srv_restock, callback_group=self._cb_group)
        self.create_service(Trigger, "/stop_mission", self._srv_stop, callback_group=self._cb_group)

        # ── Aguarda Nav2 ficar disponível ─────────────────────────
        self.get_logger().info("Aguardando Nav2 ficar disponível...")
        self._ready_timer = self.create_timer(2.0, self._check_nav2_ready, callback_group=self._cb_group)

        self.get_logger().info(
            "\n"
            "┌─────────────────────────────────────────────┐\n"
            "│         Mission Manager — TurtleBot4        │\n"
            "├─────────────────────────────────────────────┤\n"
            "│  /start_delivery  → rota entrega            │\n"
            "│  /start_failure   → rota falha              │\n"
            "│  /start_restock   → rota reabastecimento    │\n"
            "│  /stop_mission    → CANCELAR MISSÃO ATUAL   │\n"
            "└─────────────────────────────────────────────┘"
        )

    def _check_nav2_ready(self):
        """Verifica periodicamente se o Nav2 está disponível."""
        if self.delivery.nav_client.wait_for_server(timeout_sec=0.1):
            self._nav2_ready = True
            self.destroy_timer(self._ready_timer)
            self.get_logger().info("✅ Nav2 pronto — missões habilitadas!")

    # ─────────────────────────────────────────────────────────
    # Callbacks de service
    # ─────────────────────────────────────────────────────────

    def _srv_delivery(self, req, resp):
        return self._trigger_routine("delivery", self.delivery, resp)

    def _srv_failure(self, req, resp):
        return self._trigger_routine("failure", self.failure, resp)

    def _srv_restock(self, req, resp):
        return self._trigger_routine("restock", self.restock, resp)

    def _srv_stop(self, req, resp):
        self.get_logger().warn("🛑 Comando de CANCELAMENTO DE MISSÃO recebido (/stop_mission)!")
        if self._active_routine is not None:
            self._active_routine.cancel_current_mission()
            self._active_routine = None
        else:
            self.get_logger().info("ℹ️ Nenhuma missão em andamento no momento. Verificando estado do robô...")
            if self.delivery._is_docked:
                self.get_logger().info("✅ Robô já está acoplado na Dock Station. Cancelamento concluído sem movimento.")
            else:
                self.get_logger().warn("⚠️ Robô está fora da dock sem missão ativa. Retornando para predock_point & Dock Station...")
                self.delivery.return_to_dock(lambda ok: None)

        self._busy = False
        resp.success = True
        resp.message = "Cancelamento de missão processado com sucesso!"
        return resp

    # ─────────────────────────────────────────────────────────
    # Dispatcher
    # ─────────────────────────────────────────────────────────

    def _trigger_routine(self, name: str, routine, resp):
        if not self._nav2_ready:
            self.get_logger().warn("Nav2 ainda não pronto — certifique-se de iniciar a Localização e o Stack Nav2 no Dashboard.")
            resp.success = False
            resp.message = "Nav2 ainda não inicializado. Por favor, acione os botões '1. Iniciar Localização' e '2. Lançar Nav2 Stack' no painel."
            return resp

        if self._busy:
            self.get_logger().warn(
                f"Missão em andamento! Ignorando trigger '{name}'. "
                "Aguarde a missão terminar."
            )
            resp.success = False
            resp.message = "Missão em andamento. Tente novamente depois."
            return resp

        self.get_logger().info(f"Trigger recebido: {name}")
        self._busy = True
        self._active_routine = routine

        # Dispara a rotina usando um timer de disparo único
        # para não bloquear o callback do service
        _timer_ref = [None]

        def _on_mission_complete(success: bool):
            self.get_logger().info(f"Missão '{name}' reportou conclusão (Success: {success}). Liberando Manager.")
            self._busy = False
            self._active_routine = None

        def _once():
            _t = _timer_ref[0]
            if _t is not None:
                self.destroy_timer(_t)
                _timer_ref[0] = None
            routine.start(on_complete=_on_mission_complete)

        _timer_ref[0] = self.create_timer(0.01, _once, callback_group=self._cb_group)

        resp.success = True
        resp.message = f"Rotina '{name}' disparada."
        return resp


# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()

    manager = MissionManager()

    executor = MultiThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(manager.delivery)
    executor.add_node(manager.failure)
    executor.add_node(manager.restock)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        manager.delivery.destroy_node()
        manager.failure.destroy_node()
        manager.restock.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
