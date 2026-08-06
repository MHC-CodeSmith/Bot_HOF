#!/usr/bin/env python3
"""
mission_base.py
───────────────
Classe base limpa e direta para todas as rotinas de missão do TurtleBot 4.
Executa navegação via Nav2 (/navigate_to_pose) exatamente como no RViz!
"""
import math
import time
import threading
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

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


def yaw_to_quat_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionBase(Node):
    def __init__(self, node_name: str = "mission_base"):
        super().__init__(node_name)

        # ── Parâmetros ──────────────────────────────────────
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("dock_tries", 2)

        for name in _WAYPOINT_NAMES:
            self.declare_parameter(name, [0.0, 0.0, 0.0])

        self.frame_id: str = self.get_parameter("frame_id").get_parameter_value().string_value
        self.dock_tries: int = int(self.get_parameter("dock_tries").value)

        self.waypoints: Dict[str, Waypoint] = {
            name: self._wp_from_param(name) for name in _WAYPOINT_NAMES
        }

        self._cb_group = ReentrantCallbackGroup()
        self._is_docked: bool = True
        self.create_subscription(DockStatus, "/dock_status", self._on_dock_status, 10, callback_group=self._cb_group)

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self._current_goal_handle = None
        self._is_cancelling = False
        self._is_user_cancelled = False
        self._auto_dock_recovering = False

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self._cb_group)
        self.dock_client = ActionClient(self, Dock, "/dock", callback_group=self._cb_group)
        self.undock_client = ActionClient(self, Undock, "/undock", callback_group=self._cb_group)

    def _on_dock_status(self, msg: DockStatus):
        self._is_docked = msg.is_docked

    def _wp_from_param(self, name: str) -> Waypoint:
        v = self.get_parameter(name).value
        return Waypoint(float(v[0]), float(v[1]), float(v[2]))

    def cancel_current_mission(self):
        """Interrompe a missão corrente e cancela o Goal Nav2 ativo."""
        self.get_logger().warn("⚠️ Solicitado cancelamento de missão em andamento...")
        self._is_cancelling = True
        self._is_user_cancelled = True

        if hasattr(self, "_current_goal_handle") and self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._current_goal_handle = None

        stop_msg = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(stop_msg)

        self._is_cancelling = False
        self.finish_mission(False, "Missão cancelada pelo usuário.")

    def navigate_to(self, wp_name: str, done_cb: Callable[[bool], None]) -> None:
        """Envia goal de navegação para o waypoint `wp_name` diretamente via Nav2 (como no RViz)."""
        if self._is_cancelling:
            done_cb(False)
            return

        if wp_name not in self.waypoints:
            self.get_logger().error(f"Waypoint desconhecido: {wp_name}")
            done_cb(False)
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor Nav2 (/navigate_to_pose) indisponível!")
            done_cb(False)
            return

        wp = self.waypoints[wp_name]
        qz, qw = yaw_to_quat_z_w(wp.yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"🚀 Enviando Goal Nav2 para '{wp_name}': x={wp.x:.3f}, y={wp.y:.3f}")

        future = self.nav_client.send_goal_async(goal)

        def _on_sent(fut):
            try:
                gh = fut.result()
                self._current_goal_handle = gh
                if not gh.accepted:
                    self.get_logger().error(f"Goal Nav2 rejeitado para waypoint '{wp_name}'")
                    self._current_goal_handle = None
                    done_cb(False)
                    return
                self.get_logger().info(f"✅ Goal Nav2 aceito! Navegando para '{wp_name}'...")
                gh.get_result_async().add_done_callback(
                    lambda rf: self._on_nav_result(rf, wp_name, done_cb)
                )
            except Exception as e:
                self.get_logger().error(f"Exceção ao enviar goal Nav2 ({e})")
                done_cb(False)

        future.add_done_callback(_on_sent)

    def _on_nav_result(self, fut, wp_name: str, done_cb: Callable[[bool], None]):
        self._current_goal_handle = None
        try:
            status = fut.result().status
            if status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info(f"📍 Chegou ao waypoint '{wp_name}' com SUCESSO!")
                done_cb(True)
            else:
                self.get_logger().error(f"❌ Nav2 falhou para '{wp_name}' — status={status}")
                done_cb(False)
        except Exception as e:
            self.get_logger().error(f"Exceção no resultado do Nav2 ({e})")
            done_cb(False)

    def undock(self, done_cb: Callable[[bool], None]) -> None:
        """Executa undock físico se o robô estiver na dock."""
        if not self._is_docked:
            self.get_logger().info("Robô já undockado — pulando undock")
            done_cb(True)
            return

        if not self.undock_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("/undock indisponível — continuando navegação diretamente")
            done_cb(True)
            return

        try:
            future = self.undock_client.send_goal_async(Undock.Goal())

            def _on_sent(fut):
                try:
                    gh = fut.result()
                    if not gh.accepted:
                        self.get_logger().warn("Undock rejeitado — continuando navegação")
                        done_cb(True)
                        return
                    self.get_logger().info("Undock em andamento na base Create 3...")
                    gh.get_result_async().add_done_callback(
                        lambda rf: done_cb(True)
                    )
                except Exception:
                    done_cb(True)

            future.add_done_callback(_on_sent)
        except Exception:
            done_cb(True)

    def dock(self, done_cb: Callable[[bool], None], tries_left: Optional[int] = None) -> None:
        """Executa dock físico na estação de recarga."""
        if tries_left is None:
            tries_left = self.dock_tries

        if not self.dock_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("/dock indisponível")
            done_cb(False)
            return

        self.get_logger().info(f"Iniciando Docking... ({tries_left} tentativa(s))")
        try:
            future = self.dock_client.send_goal_async(Dock.Goal())

            def _on_sent(fut):
                try:
                    gh = fut.result()
                    if not gh.accepted:
                        self.get_logger().warn("Dock rejeitado")
                        if tries_left > 1:
                            self.dock(done_cb, tries_left - 1)
                        else:
                            done_cb(False)
                        return
                    gh.get_result_async().add_done_callback(
                        lambda rf: self._on_dock_result(rf, done_cb, tries_left)
                    )
                except Exception:
                    done_cb(False)

            future.add_done_callback(_on_sent)
        except Exception:
            done_cb(False)

    def _on_dock_result(self, fut, done_cb: Callable[[bool], None], tries_left: int):
        try:
            status = fut.result().status
            if status == 4:
                self.get_logger().info("✅ Docking concluído com SUCESSO!")
                done_cb(True)
            elif tries_left > 1:
                self.dock(done_cb, tries_left - 1)
            else:
                done_cb(False)
        except Exception:
            done_cb(False)

    def return_to_dock(self, done_cb: Callable[[bool], None]) -> None:
        """Navega para predock_point e em seguida acopla na Dock Station."""
        self.get_logger().info("Navegando para predock_point para iniciar o Docking...")
        self.navigate_to(
            "predock_point",
            lambda ok: self.dock(done_cb) if ok else done_cb(False),
        )

    def finish_mission(self, success: bool, msg: str = ""):
        """Finaliza a missão e notifica o mission_manager."""
        if msg:
            if success:
                self.get_logger().info(msg)
            else:
                self.get_logger().error(msg)

        if getattr(self, "_is_user_cancelled", False):
            self._is_user_cancelled = False
            if hasattr(self, "_on_complete") and self._on_complete:
                self._on_complete(False)
            return

        if not success and not getattr(self, "_is_docked", True) and not getattr(self, "_auto_dock_recovering", False):
            self._auto_dock_recovering = True
            self.get_logger().warn("🔋 BATERIA SAFEGUARD: Missão falhou enquanto undockado! Recolhendo robô para a Dock Station...")

            def _on_emergency_dock_done(dock_ok: bool):
                self._auto_dock_recovering = False
                if hasattr(self, "_on_complete") and self._on_complete:
                    self._on_complete(False)

            self.return_to_dock(_on_emergency_dock_done)
            return

        if hasattr(self, "_on_complete") and self._on_complete:
            self._on_complete(success)
