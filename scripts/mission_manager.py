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

        # ── Inicia a Câmera OAK-D e o Classificador Visual ────────
        self._classifier_proc = None
        self._start_camera_and_vision_nodes()

        # ── Instancia as rotinas ─────────────────────────────────
        self.delivery = DeliveryRoutine("delivery_routine")
        self.failure = FailureRoutine("failure_routine")
        self.restock = RestockRoutine("restock_routine")

        # ── Flags ─────────────────────────────────────────────────
        self._busy = False
        self._nav2_ready = False  # bloqueia missões até Nav2 estar pronto

        # ── Services ─────────────────────────────────────────────
        self.create_service(Trigger, "/start_delivery", self._srv_delivery)
        self.create_service(Trigger, "/start_failure", self._srv_failure)
        self.create_service(Trigger, "/start_restock", self._srv_restock)

        # ── Aguarda Nav2 ficar disponível ─────────────────────────
        self.get_logger().info("Aguardando Nav2 ficar disponível...")
        self._ready_timer = self.create_timer(2.0, self._check_nav2_ready)

        self.get_logger().info(
            "\n"
            "┌─────────────────────────────────────────────┐\n"
            "│         Mission Manager — TurtleBot4        │\n"
            "├─────────────────────────────────────────────┤\n"
            "│  /start_delivery  → rota entrega            │\n"
            "│  /start_failure   → rota falha              │\n"
            "│  /start_restock   → rota reabastecimento    │\n"
            "│  (aguardando Nav2 pronto antes de aceitar)  │\n"
            "└─────────────────────────────────────────────┘"
        )

    def _start_camera_and_vision_nodes(self):
        import subprocess
        import time
        
        self.get_logger().info("🚀 [AUTO-START] Iniciando a câmera OAK-D na Raspberry Pi (192.168.0.129) via SSH...")
        try:
            # Comando SSH em background usando nohup para iniciar o driver da câmera no robô físico
            ssh_cmd = [
                "ssh", "-o", "ConnectTimeout=3", "ubuntu@192.168.0.129",
                "nohup ros2 launch turtlebot4_bringup oakd.launch.py > /tmp/oakd.log 2>&1 &"
            ]
            subprocess.Popen(ssh_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info("✅ [AUTO-START] Comando SSH de inicialização da câmera disparado!")
        except Exception as e:
            self.get_logger().error(f"❌ [AUTO-START] Falha ao disparar câmera por SSH: {e}")

        self.get_logger().info("🚀 [AUTO-START] Iniciando o classificador visual local (vision_lid_classifier_simple.py)...")
        try:
            classifier_path = os.path.join(_SCRIPTS_DIR, "vision_lid_classifier_simple.py")
            # Dispara o classificador de visão como subprocesso em background no notebook
            self._classifier_proc = subprocess.Popen(
                ["python3", classifier_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info("✅ [AUTO-START] Classificador de visão local rodando em background!")
        except Exception as e:
            self.get_logger().error(f"❌ [AUTO-START] Falha ao iniciar o classificador local: {e}")

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

    # ─────────────────────────────────────────────────────────
    # Dispatcher
    # ─────────────────────────────────────────────────────────

    def _trigger_routine(self, name: str, routine, resp):
        if not self._nav2_ready:
            self.get_logger().warn("Nav2 ainda não pronto — aguarde o log '✅ Nav2 pronto'")
            resp.success = False
            resp.message = "Nav2 ainda não pronto. Aguarde alguns segundos e tente novamente."
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

        # Dispara a rotina usando um timer de disparo único
        # para não bloquear o callback do service
        _timer_ref = [None]

        def _on_mission_complete(success: bool):
            self.get_logger().info(f"Missão '{name}' reportou conclusão (Success: {success}). Liberando Manager.")
            self._busy = False

        def _once():
            _t = _timer_ref[0]
            if _t is not None:
                self.destroy_timer(_t)
                _timer_ref[0] = None
            routine.start(on_complete=_on_mission_complete)

        _timer_ref[0] = self.create_timer(0.01, _once)

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
        # Finaliza o classificador local se estiver rodando
        if hasattr(manager, "_classifier_proc") and manager._classifier_proc is not None:
            manager.get_logger().info("🛑 [AUTO-STOP] Parando o classificador visual local...")
            try:
                manager._classifier_proc.terminate()
                manager._classifier_proc.wait(timeout=2.0)
            except Exception:
                manager._classifier_proc.kill()
        
        manager.destroy_node()
        manager.delivery.destroy_node()
        manager.failure.destroy_node()
        manager.restock.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
