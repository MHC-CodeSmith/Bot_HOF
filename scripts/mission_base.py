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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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

        # ── Callback Group Reentrante ──────────────────────────
        self._cb_group = ReentrantCallbackGroup()

        # ── Estado da dock (atualizado por /dock_status) ─────
        self._is_docked: bool = True  # assume dockado até receber msg
        self.create_subscription(
            DockStatus, "/dock_status", self._on_dock_status, 10, callback_group=self._cb_group
        )
        self._last_scan_stamp = None
        self._last_odom_stamp = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10, callback_group=self._cb_group)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10, callback_group=self._cb_group)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Action clients & Publishers ────────────────────────
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self._current_goal_handle = None
        self._active_nav_timer = None
        self._is_cancelling = False

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self._cb_group)
        self.dock_client = ActionClient(self, Dock, "/dock", callback_group=self._cb_group)
        self.undock_client = ActionClient(self, Undock, "/undock", callback_group=self._cb_group)

    # ─────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────

    def publish_initial_pose(self, wp_name: str = "dock_station"):
        """Publica a pose inicial no AMCL para estabelecer a transformada map->odom."""
        if wp_name not in self.waypoints:
            wp_name = "dock_station"
        wp = self.waypoints.get(wp_name)
        if not wp:
            return

        qz, qw = yaw_to_quat_z_w(wp.yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = wp.x
        msg.pose.pose.position.y = wp.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(f"📍 Pose inicial (/initialpose) publicada em '{wp_name}': x={wp.x:.3f}, y={wp.y:.3f}")

    def cancel_current_mission(self):
        """Interrompe a missão corrente, cancela timers/goals ativos e para o robô."""
        self.get_logger().warn("⚠️ Solicitado cancelamento de missão em andamento...")
        self._is_cancelling = True

        if self._active_nav_timer is not None:
            try:
                self.destroy_timer(self._active_nav_timer)
            except Exception:
                pass
            self._active_nav_timer = None

        if hasattr(self, "_current_goal_handle") and self._current_goal_handle is not None:
            try:
                self.get_logger().info("Cancelando Goal Nav2 ativo...")
                self._current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Erro ao cancelar goal Nav2: {e}")
            self._current_goal_handle = None

        # Para motores do robô
        stop_msg = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(stop_msg)

        self._is_cancelling = False
        self.finish_mission(False, "Missão interrompida e cancelada pelo usuário.")

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
        initial_pose_sent = False
        timer_ref = [None]

        def _tick():
            nonlocal stable_count, last_log_sec, initial_pose_sent
            if self._is_cancelling:
                if timer_ref[0] is not None:
                    self.destroy_timer(timer_ref[0])
                self._active_nav_timer = None
                done_cb(False)
                return

            ok, status = self._nav_inputs_status()
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

            if ok:
                stable_count += 1
                if stable_count >= self.nav_input_stable_samples:
                    if timer_ref[0] is not None:
                        self.destroy_timer(timer_ref[0])
                    self._active_nav_timer = None
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
                # Se passou de 3s e o TF map->base_link está indisponível, envia initialpose automaticamente
                if elapsed > 3.0 and not initial_pose_sent and "TF" in status:
                    initial_pose_sent = True
                    self.get_logger().info("Tentando auto-publicar /initialpose (dock_station) para inicializar AMCL...")
                    self.publish_initial_pose("dock_station")

            if elapsed >= self.nav_input_wait_timeout_s:
                if timer_ref[0] is not None:
                    self.destroy_timer(timer_ref[0])
                self._active_nav_timer = None
                self.get_logger().error(
                    f"Timeout aguardando sensores/TF antes de {label}: {status}"
                )
                done_cb(False)

        timer_ref[0] = self.create_timer(0.2, _tick, callback_group=self._cb_group)
        self._active_nav_timer = timer_ref[0]

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
            self._current_goal_handle = gh
            if not gh.accepted:
                self.get_logger().error(f"Goal rejeitado: {wp_name}")
                self._current_goal_handle = None
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
        """Executa undock com watchdog de segurança para evitar travamentos."""
        if not self._is_docked:
            self.get_logger().info("Robô já undockado — pulando undock")
            done_cb(True)
            return

        if not self.undock_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("/undock indisponível — continuando sem undock")
            done_cb(True)
            return

        undock_finished = False
        watchdog = [None]

        def _finish(success: bool, reason: str = ""):
            nonlocal undock_finished
            if undock_finished:
                return
            undock_finished = True
            if watchdog[0] is not None:
                try:
                    self.destroy_timer(watchdog[0])
                except Exception:
                    pass
                watchdog[0] = None
            if reason:
                self.get_logger().info(f"Undock finalizado ({reason})")
            done_cb(success)

        # Watchdog de 10s: se o robô não responder action result, avança para o próximo passo sem travar
        watchdog[0] = self.create_timer(10.0, lambda: _finish(True, "watchdog timeout 10s"), callback_group=self._cb_group)

        try:
            future = self.undock_client.send_goal_async(Undock.Goal())

            def _on_sent(fut):
                try:
                    gh = fut.result()
                    if not gh.accepted:
                        self.get_logger().warn("Undock rejeitado — continuando")
                        _finish(True, "goal rejeitado")
                        return
                    self.get_logger().info("Undock iniciado...")
                    gh.get_result_async().add_done_callback(
                        lambda rf: self._on_undock_result_safe(rf, _finish)
                    )
                except Exception as e:
                    self.get_logger().warn(f"Exceção no envio de undock ({e}) — continuando")
                    _finish(True, f"exceção {e}")

            future.add_done_callback(_on_sent)
        except Exception as e:
            self.get_logger().warn(f"Falha ao chamar undock ({e}) — continuando")
            _finish(True, f"erro cliente {e}")

    def _on_undock_result_safe(self, fut, finish_cb: Callable[[bool, str], None]):
        try:
            status = fut.result().status
            wait_s = self.post_undock_stabilize_s
            self.get_logger().info(
                f"Undock físico concluído (status={status}). "
                f"Aguardando {wait_s:.1f}s para estabilizar lidar, odometria e AMCL..."
            )
        except Exception as e:
            self.get_logger().warn(f"Undock result exceção: {e}")

        # Timer de estabilização
        def _after_wait():
            self.get_logger().info("Estabilização concluída. Navegando para predock_point...")
            self.navigate_to("predock_point", lambda nav_ok: finish_cb(nav_ok, "sucesso"))

        self.create_timer(3.0, _after_wait, callback_group=self._cb_group)

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
