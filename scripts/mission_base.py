#!/usr/bin/env python3
"""
mission_base.py
───────────────
Classe base limpa e direta para todas as rotinas de missão do TurtleBot 4.
Executa navegação via Nav2 (/navigate_to_pose) exatamente como no RViz!
"""
import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

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


def receipt_is_fresh(received_at: Optional[float], now: float, max_age_s: float) -> bool:
    """Retorna True somente para uma medição recebida dentro da janela informada."""
    return received_at is not None and 0.0 <= now - received_at <= max_age_s


class MissionBase(Node):
    def __init__(self, node_name: str = "mission_base"):
        super().__init__(node_name)

        # ── Parâmetros ──────────────────────────────────────
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("dock_tries", 2)
        self.declare_parameter("dock_status_max_age_s", 3.0)
        self.declare_parameter("post_undock_stabilize_s", 8.0)
        self.declare_parameter("nav_input_max_age_s", 3.0)
        self.declare_parameter("nav_input_wait_timeout_s", 25.0)
        self.declare_parameter("nav_input_stable_samples", 5)

        for name in _WAYPOINT_NAMES:
            self.declare_parameter(name, [0.0, 0.0, 0.0])

        self.frame_id: str = self.get_parameter("frame_id").get_parameter_value().string_value
        self.base_frame_id: str = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self.dock_tries: int = int(self.get_parameter("dock_tries").value)
        self.dock_status_max_age_s = float(
            self.get_parameter("dock_status_max_age_s").value
        )
        self.post_undock_stabilize_s = float(self.get_parameter("post_undock_stabilize_s").value)
        self.nav_input_max_age_s = float(self.get_parameter("nav_input_max_age_s").value)
        self.nav_input_wait_timeout_s = float(self.get_parameter("nav_input_wait_timeout_s").value)
        self.nav_input_stable_samples = int(self.get_parameter("nav_input_stable_samples").value)
        self._validate_configuration()

        self.waypoints: Dict[str, Waypoint] = {
            name: self._wp_from_param(name) for name in _WAYPOINT_NAMES
        }

        self._cb_group = ReentrantCallbackGroup()
        self._is_docked: Optional[bool] = None
        self._last_dock_status_monotonic: Optional[float] = None
        self.create_subscription(DockStatus, "/dock_status", self._on_dock_status, 10, callback_group=self._cb_group)

        self._last_scan_monotonic: Optional[float] = None
        self._last_odom_monotonic: Optional[float] = None
        self._scan_subscription = None
        self._odom_subscription = None
        self._tf_buffer = None
        self._tf_listener = None

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self._current_goal_handle = None
        self._is_cancelling = False
        self._is_user_cancelled = False
        self._auto_dock_recovering = False
        self._finish_started = False
        self._completion_notified = False
        self._nav_wait_timer = None
        self._nav_wait_done_cb = None
        self._nav_wait_stable_count = 0
        self._nav_not_before_monotonic = 0.0
        self._nav_retry_timer = None

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self._cb_group)
        self.dock_client = ActionClient(self, Dock, "/dock", callback_group=self._cb_group)
        self.undock_client = ActionClient(self, Undock, "/undock", callback_group=self._cb_group)
        self._clear_global_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self._cb_group,
        )
        self._clear_local_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=self._cb_group,
        )

    def _validate_configuration(self) -> None:
        positive_values = {
            "dock_tries": self.dock_tries,
            "dock_status_max_age_s": self.dock_status_max_age_s,
            "post_undock_stabilize_s": self.post_undock_stabilize_s,
            "nav_input_max_age_s": self.nav_input_max_age_s,
            "nav_input_wait_timeout_s": self.nav_input_wait_timeout_s,
            "nav_input_stable_samples": self.nav_input_stable_samples,
        }
        invalid = [name for name, value in positive_values.items() if value <= 0]
        if invalid:
            raise ValueError(f"Parâmetros devem ser maiores que zero: {', '.join(invalid)}")

    def _on_dock_status(self, msg: DockStatus):
        self._is_docked = bool(msg.is_docked)
        self._last_dock_status_monotonic = time.monotonic()

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_monotonic = time.monotonic()

    def _on_odom(self, _msg: Odometry) -> None:
        self._last_odom_monotonic = time.monotonic()

    def _start_navigation_observers(self) -> None:
        if self._scan_subscription is not None:
            return
        self._last_scan_monotonic = None
        self._last_odom_monotonic = None
        self._scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )

    def _stop_navigation_observers(self) -> None:
        if self._tf_listener is not None:
            try:
                self._tf_listener.unregister()
            except Exception:
                pass
            self._tf_listener = None
        self._tf_buffer = None
        for attribute in ("_scan_subscription", "_odom_subscription"):
            subscription = getattr(self, attribute, None)
            setattr(self, attribute, None)
            if subscription is not None:
                try:
                    self.destroy_subscription(subscription)
                except Exception:
                    pass
        self._last_scan_monotonic = None
        self._last_odom_monotonic = None

    def _wp_from_param(self, name: str) -> Waypoint:
        v = self.get_parameter(name).value
        if not isinstance(v, (list, tuple)) or len(v) != 3:
            raise ValueError(f"Waypoint '{name}' deve conter exatamente [x, y, yaw]")
        if not all(math.isfinite(float(item)) for item in v):
            raise ValueError(f"Waypoint '{name}' contém valor não finito")
        return Waypoint(float(v[0]), float(v[1]), float(v[2]))

    def begin_mission(self, on_complete=None, watchdog_timeout_s: float = 180.0) -> None:
        """Inicializa o estado compartilhado de uma nova missão."""
        self._on_complete = on_complete
        self._finish_started = False
        self._completion_notified = False
        self._is_cancelling = False
        self._is_user_cancelled = False
        self._auto_dock_recovering = False
        self._cancel_navigation_wait()
        self._cancel_navigation_retry()
        self._start_navigation_observers()
        self.start_mission_watchdog(watchdog_timeout_s)

    def start_mission_watchdog(self, timeout_sec: float = 180.0) -> None:
        """Inicia temporizador de segurança para cancelar a missão se ela ultrapassar timeout_sec."""
        self.stop_mission_watchdog()

        def _on_watchdog_timeout():
            self.get_logger().warn(f"⚠️ Watchdog de missão atingiu o timeout de {timeout_sec}s!")
            self.finish_mission(False, "Mission watchdog timeout")

        self._watchdog_timer = self.create_timer(
            timeout_sec,
            _on_watchdog_timeout,
            callback_group=self._cb_group,
        )

    def stop_mission_watchdog(self) -> None:
        """Cancela o temporizador de segurança da missão."""
        if hasattr(self, "_watchdog_timer") and self._watchdog_timer is not None:
            try:
                self._watchdog_timer.cancel()
                self.destroy_timer(self._watchdog_timer)
            except Exception:
                pass
            self._watchdog_timer = None

    def clear_costmaps(self) -> bool:
        """Solicita limpeza dos costmaps pelos clientes ROS do próprio nó."""
        dispatched = True
        for label, client in (
            ("global", self._clear_global_client),
            ("local", self._clear_local_client),
        ):
            if not client.wait_for_service(timeout_sec=0.0):
                self.get_logger().warn(f"Serviço de limpeza do costmap {label} indisponível")
                dispatched = False
                continue
            try:
                future = client.call_async(ClearEntireCostmap.Request())
                future.add_done_callback(
                    lambda fut, costmap=label: self._on_costmap_clear_result(fut, costmap)
                )
            except Exception as exc:
                self.get_logger().error(f"Falha ao solicitar limpeza do costmap {label}: {exc}")
                dispatched = False
        return dispatched

    def _on_costmap_clear_result(self, future, label: str) -> None:
        try:
            future.result()
            self.get_logger().info(f"Costmap {label} limpo com confirmação do serviço")
        except Exception as exc:
            self.get_logger().error(f"Serviço de limpeza do costmap {label} falhou: {exc}")

    def cancel_current_mission(self):
        """Cancela a meta corrente e para no local, sem iniciar novo movimento."""
        self.get_logger().warn("⚠️ Solicitado cancelamento de missão em andamento...")
        self._is_cancelling = True
        self._is_user_cancelled = True
        self.stop_mission_watchdog()
        self._cancel_navigation_wait()
        self._cancel_navigation_retry()

        stop_msg = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(stop_msg)

        goal_handle = self._current_goal_handle
        if goal_handle is None:
            self.finish_mission(False, "Missão cancelada pelo usuário; nenhuma meta Nav2 ativa.")
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_response)
        except Exception as exc:
            self.get_logger().error(f"Falha ao solicitar cancelamento da meta: {exc}")
            self.finish_mission(False, "Cancelamento solicitado, mas a confirmação da meta falhou.")

    def _on_cancel_response(self, future) -> None:
        try:
            response = future.result()
            confirmed = bool(getattr(response, "goals_canceling", []))
            if confirmed:
                message = "Cancelamento da meta confirmado; robô mantido no local."
            else:
                message = "Servidor não confirmou meta em cancelamento; robô mantido no local."
        except Exception as exc:
            message = f"Falha ao confirmar cancelamento da meta: {exc}"
        self.finish_mission(False, message)

    def _navigation_inputs_status(self) -> tuple[bool, str]:
        now = time.monotonic()
        if now < self._nav_not_before_monotonic:
            remaining = self._nav_not_before_monotonic - now
            return False, f"estabilização pós-undock ({remaining:.1f}s restantes)"
        if not receipt_is_fresh(self._last_scan_monotonic, now, self.nav_input_max_age_s):
            return False, "/scan ausente ou antigo"
        if not receipt_is_fresh(self._last_odom_monotonic, now, self.nav_input_max_age_s):
            return False, "/odom ausente ou antigo"
        try:
            if self._tf_buffer is None:
                return False, "observador TF não inicializado"
            transform = self._tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame_id,
                Time(),
            )
            tf_stamp = Time.from_msg(transform.header.stamp)
            tf_age_s = (self.get_clock().now() - tf_stamp).nanoseconds / 1e9
            if tf_age_s < -0.5 or tf_age_s > self.nav_input_max_age_s:
                return False, f"TF {self.frame_id}->{self.base_frame_id} antiga ({tf_age_s:.2f}s)"
        except (TransformException, ValueError, RuntimeError) as exc:
            return False, f"TF {self.frame_id}->{self.base_frame_id} indisponível ({exc})"
        return True, "scan, odom e TF frescos"

    def _wait_for_navigation_inputs(self, done_cb: Callable[[bool], None]) -> None:
        self._cancel_navigation_wait()
        self._nav_wait_done_cb = done_cb
        self._nav_wait_stable_count = 0
        start = time.monotonic()
        deadline = max(start, self._nav_not_before_monotonic) + self.nav_input_wait_timeout_s
        last_reason = {"value": "ainda não medido"}

        def _check() -> None:
            if self._is_cancelling or (
                self._finish_started and not self._auto_dock_recovering
            ):
                self._complete_navigation_wait(False)
                return
            ready, reason = self._navigation_inputs_status()
            last_reason["value"] = reason
            self._nav_wait_stable_count = self._nav_wait_stable_count + 1 if ready else 0
            if self._nav_wait_stable_count >= self.nav_input_stable_samples:
                self.get_logger().info(
                    f"Entradas de navegação confirmadas por {self.nav_input_stable_samples} amostras: {reason}"
                )
                self._complete_navigation_wait(True)
            elif time.monotonic() >= deadline:
                self.get_logger().error(
                    f"Timeout aguardando entradas de navegação: {last_reason['value']}"
                )
                self._complete_navigation_wait(False)

        self._nav_wait_timer = self.create_timer(0.2, _check, callback_group=self._cb_group)

    def _complete_navigation_wait(self, ready: bool) -> None:
        callback = self._nav_wait_done_cb
        self._nav_wait_done_cb = None
        self._cancel_navigation_wait()
        if callback is not None:
            callback(ready)

    def _cancel_navigation_wait(self) -> None:
        timer = getattr(self, "_nav_wait_timer", None)
        self._nav_wait_timer = None
        self._nav_wait_done_cb = None
        if timer is not None:
            try:
                timer.cancel()
                self.destroy_timer(timer)
            except Exception:
                pass

    def _schedule_navigation_retry(
        self,
        wp_name: str,
        done_cb: Callable[[bool], None],
        retry_count: int,
    ) -> None:
        self._cancel_navigation_retry()

        def _retry() -> None:
            self._cancel_navigation_retry()
            if self._is_cancelling or (
                self._finish_started and not self._auto_dock_recovering
            ):
                done_cb(False)
                return
            self.navigate_to(wp_name, done_cb, retry_count)

        self._nav_retry_timer = self.create_timer(
            1.5,
            _retry,
            callback_group=self._cb_group,
        )

    def _cancel_navigation_retry(self) -> None:
        timer = getattr(self, "_nav_retry_timer", None)
        self._nav_retry_timer = None
        if timer is not None:
            try:
                timer.cancel()
                self.destroy_timer(timer)
            except Exception:
                pass

    def navigate_to(self, wp_name: str, done_cb: Callable[[bool], None], retry_count: int = 0) -> None:
        """Envia goal de navegação para o waypoint `wp_name` diretamente via Nav2 com retentativas automáticas."""
        if self._is_cancelling:
            done_cb(False)
            return

        if wp_name not in self.waypoints:
            self.get_logger().error(f"Waypoint desconhecido: {wp_name}")
            done_cb(False)
            return

        self.get_logger().info(f"Validando entradas de navegação antes do goal '{wp_name}'...")
        self._wait_for_navigation_inputs(
            lambda ready: self._send_navigation_goal(wp_name, done_cb, retry_count)
            if ready
            else done_cb(False)
        )

    def _send_navigation_goal(
        self,
        wp_name: str,
        done_cb: Callable[[bool], None],
        retry_count: int = 0,
    ) -> None:
        if self._is_cancelling or (
            self._finish_started and not self._auto_dock_recovering
        ):
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
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        attempt_str = f" (Tentativa {retry_count + 1}/3)" if retry_count > 0 else ""
        self.get_logger().info(f"🚀 Enviando Goal Nav2 para '{wp_name}': x={wp.x:.3f}, y={wp.y:.3f}{attempt_str}")

        future = self.nav_client.send_goal_async(goal)

        def _on_sent(fut):
            try:
                gh = fut.result()
                self._current_goal_handle = gh
                if not gh.accepted:
                    self.get_logger().error(f"Goal Nav2 rejeitado para waypoint '{wp_name}'")
                    self._current_goal_handle = None
                    if retry_count < 2 and not self._is_cancelling:
                        self.get_logger().warn(f"⚠️ Reenviando goal para '{wp_name}' em 1.5s após rejeição...")
                        self.clear_costmaps()
                        self._schedule_navigation_retry(
                            wp_name, done_cb, retry_count + 1
                        )
                    else:
                        done_cb(False)
                    return

                self.get_logger().info(f"✅ Goal Nav2 aceito! Navegando para '{wp_name}'...")
                gh.get_result_async().add_done_callback(
                    lambda rf: self._on_nav_result(rf, wp_name, done_cb, retry_count)
                )
            except Exception as e:
                self.get_logger().error(f"Exceção ao enviar goal Nav2 ({e})")
                done_cb(False)

        future.add_done_callback(_on_sent)

    def _on_nav_result(self, fut, wp_name: str, done_cb: Callable[[bool], None], retry_count: int = 0):
        self._current_goal_handle = None
        try:
            status = fut.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"📍 Chegou ao waypoint '{wp_name}' com SUCESSO!")
                done_cb(True)
            elif status == GoalStatus.STATUS_CANCELED or getattr(self, "_is_user_cancelled", False) or getattr(self, "_is_cancelling", False):
                self.get_logger().warn(f"🛑 Navegação para '{wp_name}' interrompida por cancelamento.")
                done_cb(False)
            else:
                self.get_logger().error(f"❌ Nav2 falhou para '{wp_name}' — status={status}")
                if retry_count < 2 and not getattr(self, "_is_user_cancelled", False) and not getattr(self, "_is_cancelling", False):
                    self.get_logger().warn(f"⚠️ Retentando navegação para '{wp_name}' (Tentativa {retry_count + 2}/3)...")
                    self.clear_costmaps()
                    self._schedule_navigation_retry(
                        wp_name, done_cb, retry_count + 1
                    )
                else:
                    done_cb(False)
        except Exception as e:
            self.get_logger().error(f"Exceção no resultado do Nav2 ({e})")
            done_cb(False)

    def undock(self, done_cb: Callable[[bool], None]) -> None:
        """Executa undock físico se o robô estiver na dock."""
        now = time.monotonic()
        if not receipt_is_fresh(
            self._last_dock_status_monotonic,
            now,
            self.dock_status_max_age_s,
        ):
            self.get_logger().error(
                "Estado da dock ausente ou antigo em /dock_status; undock recusado"
            )
            done_cb(False)
            return

        if self._is_docked is False:
            self.get_logger().info("Robô já undockado — pulando undock")
            done_cb(True)
            return

        if not self.undock_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("/undock indisponível; missão não continuará")
            done_cb(False)
            return

        try:
            future = self.undock_client.send_goal_async(Undock.Goal())

            def _on_sent(fut):
                try:
                    gh = fut.result()
                    if not gh.accepted:
                        self.get_logger().error("Undock rejeitado; missão não continuará")
                        done_cb(False)
                        return
                    def _on_undock_result(rf):
                        try:
                            status = rf.result().status
                        except Exception as exc:
                            self.get_logger().error(f"Falha ao ler resultado do undock: {exc}")
                            done_cb(False)
                            return
                        if status != GoalStatus.STATUS_SUCCEEDED:
                            self.get_logger().error(f"Undock falhou — status={status}")
                            done_cb(False)
                            return
                        self._is_docked = False
                        self._last_dock_status_monotonic = time.monotonic()
                        self._nav_not_before_monotonic = (
                            self._last_dock_status_monotonic
                            + self.post_undock_stabilize_s
                        )
                        self.get_logger().info(
                            "Undock confirmado; aguardando estabilização e dados frescos antes do próximo goal"
                        )
                        self.clear_costmaps()
                        done_cb(True)

                    gh.get_result_async().add_done_callback(_on_undock_result)
                except Exception as exc:
                    self.get_logger().error(f"Falha ao enviar goal de undock: {exc}")
                    done_cb(False)

            future.add_done_callback(_on_sent)
        except Exception as exc:
            self.get_logger().error(f"Exceção ao iniciar undock: {exc}")
            done_cb(False)

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
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._is_docked = True
                self._last_dock_status_monotonic = time.monotonic()
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
        if self._finish_started:
            return
        self._finish_started = True
        self.stop_mission_watchdog()
        self._cancel_navigation_wait()
        self._cancel_navigation_retry()
        if msg:
            if success:
                self.get_logger().info(msg)
            else:
                self.get_logger().error(msg)

        if getattr(self, "_is_user_cancelled", False):
            self._is_user_cancelled = False
            self._is_cancelling = False
            self._notify_mission_complete(False)
            return

        if not success and self._is_docked is False and not getattr(self, "_auto_dock_recovering", False):
            self._auto_dock_recovering = True
            self.get_logger().warn("🔋 BATERIA SAFEGUARD: Missão falhou enquanto undockado! Recolhendo robô para a Dock Station...")

            def _on_emergency_dock_done(dock_ok: bool):
                self._auto_dock_recovering = False
                self._notify_mission_complete(False)

            self.return_to_dock(_on_emergency_dock_done)
            return

        self._notify_mission_complete(success)

    def _notify_mission_complete(self, success: bool) -> None:
        if self._completion_notified:
            return
        self._completion_notified = True
        self._stop_navigation_observers()
        if hasattr(self, "_on_complete") and self._on_complete:
            self._on_complete(success)
