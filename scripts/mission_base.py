#!/usr/bin/env python3
"""
mission_base.py
───────────────
Classe base para todas as rotinas de missão.
Fornece: navigate_to, dock, undock, return_to_dock, helpers de pose/quaternion.
"""
import math
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# ──────────────────────────────────────────
_WAYPOINT_NAMES = [
    "dock_station",
    "predock_point",
    "delivery_red",
    "delivery_blue",
    "failure_zone",
    "failure_pickup",
    "pickup_point",
    "inventory_point",
    "supply_point",
]


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float  # radianos


def yaw_to_quat_z_w(yaw: float):
    """Converte yaw 2D para componentes z, w do quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ──────────────────────────────────────────
class MissionBase(Node):
    """
    Classe base compartilhada por todas as rotinas de missão.
    Declara parâmetros, carrega waypoints e expõe métodos helpers.
    """

    def __init__(self, node_name: str = "mission_base"):
        super().__init__(node_name)

        # ── Parâmetros ──────────────────────────────────────
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("dock_tries", 2)
        self.declare_parameter("post_undock_stabilize_s", 8.0)
        self.declare_parameter("nav_input_max_age_s", 1.0)
        self.declare_parameter("nav_input_wait_timeout_s", 25.0)
        self.declare_parameter("nav_input_stable_samples", 5)

        for name in _WAYPOINT_NAMES:
            self.declare_parameter(name, [0.0, 0.0, 0.0])

        # ── Leitura ──────────────────────────────────────────
        self.frame_id: str = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.dock_tries: int = int(self.get_parameter("dock_tries").value)
        self.post_undock_stabilize_s: float = float(
            self.get_parameter("post_undock_stabilize_s").value
        )
        self.nav_input_max_age_s: float = float(
            self.get_parameter("nav_input_max_age_s").value
        )
        self.nav_input_wait_timeout_s: float = float(
            self.get_parameter("nav_input_wait_timeout_s").value
        )
        self.nav_input_stable_samples: int = int(
            self.get_parameter("nav_input_stable_samples").value
        )

        self.waypoints: Dict[str, Waypoint] = {
            name: self._wp_from_param(name) for name in _WAYPOINT_NAMES
        }

        # ── Estado da dock (atualizado por /dock_status) ─────
        self._is_docked: bool = True  # assume dockado até receber msg
        self.create_subscription(
            DockStatus, "/dock_status", self._on_dock_status, 10
        )
        self._last_scan_stamp = None
        self._last_odom_stamp = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Action clients ───────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.dock_client = ActionClient(self, Dock, "/dock")
        self.undock_client = ActionClient(self, Undock, "/undock")

    # ─────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────

    def _on_dock_status(self, msg: DockStatus):
        self._is_docked = msg.is_docked

    def _on_scan(self, msg: LaserScan):
        self._last_scan_stamp = msg.header.stamp

    def _on_odom(self, msg: Odometry):
        self._last_odom_stamp = msg.header.stamp

    def _wp_from_param(self, name: str) -> Waypoint:
        v = self.get_parameter(name).value
        return Waypoint(float(v[0]), float(v[1]), float(v[2]))

    def _stamp_age_s(self, stamp) -> Optional[float]:
        if stamp is None:
            return None
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9

    def _nav_inputs_status(self) -> tuple[bool, str]:
        scan_age = self._stamp_age_s(self._last_scan_stamp)
        if scan_age is None:
            return False, "sem /scan ainda"
        if scan_age > self.nav_input_max_age_s:
            return False, f"/scan atrasado {scan_age:.2f}s"

        odom_age = self._stamp_age_s(self._last_odom_stamp)
        if odom_age is None:
            return False, "sem /odom ainda"
        if odom_age > self.nav_input_max_age_s:
            return False, f"/odom atrasado {odom_age:.2f}s"

        if not self._tf_buffer.can_transform(
            self.frame_id, "base_link", Time(), timeout=Duration(seconds=0.05)
        ):
            return False, f"TF {self.frame_id}->base_link indisponível"

        return True, f"/scan {scan_age:.2f}s, /odom {odom_age:.2f}s, TF ok"

    def wait_for_nav_inputs(
        self, label: str, done_cb: Callable[[bool], None]
    ) -> None:
        start = self.get_clock().now()
        stable_count = 0
        last_log_sec = -1
        timer_ref = [None]

        def _tick():
            nonlocal stable_count, last_log_sec
            ok, status = self._nav_inputs_status()
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

            if ok:
                stable_count += 1
                if stable_count >= self.nav_input_stable_samples:
                    self.destroy_timer(timer_ref[0])
                    self.get_logger().info(f"Nav inputs prontos para {label}: {status}")
                    done_cb(True)
                return

            stable_count = 0
            elapsed_sec = int(elapsed)
            if elapsed_sec != last_log_sec:
                last_log_sec = elapsed_sec
                self.get_logger().warn(
                    f"Aguardando sensores/TF antes de {label}: {status}"
                )

            if elapsed >= self.nav_input_wait_timeout_s:
                self.destroy_timer(timer_ref[0])
                self.get_logger().error(
                    f"Timeout aguardando sensores/TF antes de {label}: {status}"
                )
                done_cb(False)

        timer_ref[0] = self.create_timer(0.2, _tick)

    # ─────────────────────────────────────────────────────────
    # Navegação
    # ─────────────────────────────────────────────────────────

    def navigate_to(self, wp_name: str, done_cb: Callable[[bool], None]) -> None:
        """Envia goal de navegação para o waypoint `wp_name`.

        `done_cb(True)`  → chegou com sucesso
        `done_cb(False)` → falha
        """
        self.wait_for_nav_inputs(
            wp_name,
            lambda ready: self._send_nav_goal(wp_name, done_cb)
            if ready
            else done_cb(False),
        )

    def _send_nav_goal(self, wp_name: str, done_cb: Callable[[bool], None]) -> None:
        if wp_name not in self.waypoints:
            self.get_logger().error(f"Waypoint desconhecido: {wp_name}")
            done_cb(False)
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/navigate_to_pose indisponível")
            done_cb(False)
            return

        wp = self.waypoints[wp_name]
        qz, qw = yaw_to_quat_z_w(wp.yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = Time().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Enviando goal {wp_name}: x={wp.x:.3f}, y={wp.y:.3f}, "
            f"yaw={wp.yaw:.3f}, frame={self.frame_id}, stamp=latest"
        )

        future = self.nav_client.send_goal_async(goal)

        def _on_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().error(f"Goal rejeitado: {wp_name}")
                done_cb(False)
                return
            self.get_logger().info(f"Goal aceito: {wp_name}")
            gh.get_result_async().add_done_callback(
                lambda rf: self._on_nav_result(rf, wp_name, done_cb)
            )

        future.add_done_callback(_on_sent)

    def _on_nav_result(self, fut, wp_name: str, done_cb: Callable[[bool], None]):
        status = fut.result().status
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(f"✅ Chegou em: {wp_name}")
            done_cb(True)
        else:
            self.get_logger().error(f"❌ Falha ao ir para {wp_name} — status={status}")
            done_cb(False)

    # ─────────────────────────────────────────────────────────
    # Undock
    # ─────────────────────────────────────────────────────────

    def undock(self, done_cb: Callable[[bool], None]) -> None:
        """Executa undock. Pula se o robô já estiver fora da dock."""
        if not self._is_docked:
            self.get_logger().info("Robô já undockado — pulando undock")
            done_cb(True)
            return

        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("/undock indisponível — continuando sem undock")
            done_cb(True)
            return

        future = self.undock_client.send_goal_async(Undock.Goal())

        def _on_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("Undock rejeitado — continuando")
                done_cb(True)
                return
            self.get_logger().info("Undock iniciado...")
            gh.get_result_async().add_done_callback(
                lambda rf: self._on_undock_result(rf, done_cb)
            )

        future.add_done_callback(_on_sent)

    def _on_undock_result(self, fut, done_cb: Callable[[bool], None]):
        status = fut.result().status
        wait_s = self.post_undock_stabilize_s
        self.get_logger().info(
            f"Undock físico concluído (status={status}). "
            f"Aguardando {wait_s:.1f}s para estabilizar lidar, odometria e AMCL..."
        )

        # Cria um timer de disparo único (one-shot)
        self._undock_timer = None
        def _after_wait():
            self._undock_timer.cancel()
            self.get_logger().info("Estabilização concluída. Iniciando navegação para predock_point...")
            self.navigate_to("predock_point", lambda nav_ok: done_cb(nav_ok))
            
        self._undock_timer = self.create_timer(3.0, _after_wait)

    # ─────────────────────────────────────────────────────────
    # Dock
    # ─────────────────────────────────────────────────────────

    def dock(
        self, done_cb: Callable[[bool], None], tries_left: Optional[int] = None
    ) -> None:
        """Executa dock com retentativas."""
        if tries_left is None:
            tries_left = self.dock_tries

        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/dock indisponível")
            done_cb(False)
            return

        self.get_logger().info(f"Tentando dock... ({tries_left} tentativas restantes)")
        future = self.dock_client.send_goal_async(Dock.Goal())

        def _on_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("Dock rejeitado")
                if tries_left > 1:
                    self.dock(done_cb, tries_left - 1)
                else:
                    done_cb(False)
                return
            self.get_logger().info("Dock goal enviado")
            gh.get_result_async().add_done_callback(
                lambda rf: self._on_dock_result(rf, done_cb, tries_left)
            )

        future.add_done_callback(_on_sent)

    def _on_dock_result(self, fut, done_cb: Callable[[bool], None], tries_left: int):
        status = fut.result().status
        if status == 4:
            self.get_logger().info("✅ Dock bem-sucedido!")
            done_cb(True)
        else:
            self.get_logger().warn(f"Dock falhou — status={status}")
            if tries_left > 1:
                self.dock(done_cb, tries_left - 1)
            else:
                self.get_logger().error("❌ Dock falhou após todas as tentativas")
                done_cb(False)

    # ─────────────────────────────────────────────────────────
    # Rotina de retorno
    # ─────────────────────────────────────────────────────────

    def return_to_dock(self, done_cb: Callable[[bool], None]) -> None:
        """Navega para predock_point e depois chama dock."""
        self.get_logger().info("Retornando para predock_point...")
        self.navigate_to(
            "predock_point",
            lambda ok: self.dock(done_cb) if ok else done_cb(False),
        )

    def finish_mission(self, success: bool, msg: str = ""):
        """Sinaliza ao mission_manager que esta rotina acabou."""
        if msg:
            if success:
                self.get_logger().info(msg)
            else:
                self.get_logger().error(msg)
        
        if hasattr(self, '_on_complete') and self._on_complete:
            self._on_complete(success)
