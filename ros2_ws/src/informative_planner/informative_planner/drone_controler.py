#!/usr/bin/python3
"""Wrapper around the MRS Octomap Planner waypoint interface."""

import json

import numpy as np
from mrs_modules_msgs.msg import OctomapPlannerDiagnostics
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from mrs_msgs.srv import Vec4
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from informative_planner import config

WAYPOINT_SETTLE_TIME = 0.05      # [s]
MIN_WAYPOINT_COMMAND_INTERVAL = 0.1  # [s]
# OctoMap Planner para de navegar quando bate em obstacle avoidance radius
# (~1m ao redor de troncos). Se o planner já está idle e o drone não avança
# mais que STUCK_IMPROVEMENT_THRESHOLD em STUCK_TIMEOUT segundos, aceita
# a posição atual como chegada.
STUCK_TIMEOUT = 1.5               # [s]
STUCK_IMPROVEMENT_THRESHOLD = 0.1 # [m]




class DroneController(Node):

    def __init__(self, uav_name='uav1'):
        super().__init__('drone_controller')

        self.uav_name = str(uav_name)
        self.goto_service = f'/{self.uav_name}/octomap_planner/goto'
        self.stop_service = f'/{self.uav_name}/octomap_planner/stop'
        self.diagnostics_topic = f'/{self.uav_name}/octomap_planner/diagnostics'
        self.waypoint_command_topic = f'/{self.uav_name}/planner_waypoint_cmd'
    
        self._current_goal = None
        self._current_position = None
        self._goal_active = False
        self._goal_request_pending = False
        self._goto_future = None
        self._stop_future = None
        self._stop_request_pending = False
        self._stop_sent_for_goal = False
        self._planner_idle = False
        self._diagnostics_received = False
        self._last_waypoint_sent_time = None
        self._goal_started_time = None
        self._goal_timeout_sec = None
        self._goal_finished_time = None
        self._saw_planner_busy_for_goal = False
        self._best_approach_distance = None
        self._best_approach_time = None
        self._watchdog_reference_position = None
        self._watchdog_last_motion_time = None
        self.arrived = False
        self._halted = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)


        self._client = self.create_client(Vec4, self.goto_service)
        self._stop_client = self.create_client(Trigger, self.stop_service)
        self._waypoint_command_pub = self.create_publisher(
            String,
            self.waypoint_command_topic,
            10,
        )
        self._diagnostics_sub = self.create_subscription(
            OctomapPlannerDiagnostics,
            self.diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            f'/{self.uav_name}/estimation_manager/odom_main',
            self._odom_callback,
            10,
        )
        self._watchdog_timer = self.create_timer(0.5, self._check_goal_watchdog)

    # -----------------------------------------------------------------

    # ------------------------------------------------------------------
    # Envia o waypoint atual ao planner OctoMap
    # ------------------------------------------------------------------
    @property
    def current_goal(self):
        """Return the active/requested goal as an immutable tuple."""
        if self._current_goal is None:
            return None
        return tuple(self._current_goal)

    @property
    def goal_active(self):
        """Return whether the UAV is currently executing a goal."""
        return bool(self._goal_active)

    # TODO(early-handoff): hoje o executor espera chegada completa (tolerância
    # 0.75 m + Octomap Planner idle + settle) antes de aceitar o PRÓXIMO
    # waypoint, o que zera a velocidade a cada perna de 5 m do RRT. A análise
    # de perfil de movimento (2026-09-01) mostrou que isso custa ~0.1 m/s de
    # velocidade média vs o greedy (v_mov 0.52 vs 0.62 m/s no clustered) — um
    # confundidor de "perfil de voo", não de inteligência do planner.
    # Melhoria planejada (não crítica para a comparação teórica atual):
    #   - para waypoints INTERMEDIÁRIOS do segmento commitado do RRT, enviar o
    #     próximo goto assim que a odometria entrar num raio folgado (~2.5 m)
    #     do goal atual, SEM esperar idle (o octomap_planner aceita goto em
    #     voo e replaneja da posição corrente; ver replan_after no mapplan);
    #   - manter o critério estrito (0.75 m + idle) apenas no ÚLTIMO waypoint
    #     do segmento;
    #   - ajustar o watchdog e o flag `arrived`, que hoje assumem um goal por
    #     vez, e a validação de colisão do próximo segmento no feeder
    #     (gaussian_feeder._peek_committed_path).
    # Mantém a resolução de decisão de 5 m com pernas longas de voo — remove o
    # stop-and-go sem aumentar RRT_STEP_SIZE (que quebraria a escala de vãos
    # entre troncos, o modelo de 1 medição/nó e o alcance do horizonte).
    def is_ready_for_waypoint(self):
        return self.waypoint_wait_reason() is None

    def halt(self):
        """Fim de missão: recusa novos goals e para o Octomap Planner."""
        self._halted = True
        if self._goal_active:
            self._request_planner_stop()

    def waypoint_wait_reason(self):
        if self._halted:
            return 'missao encerrada'

        if not self._diagnostics_received:
            return 'diagnostico do Octomap Planner'

        if self._goal_request_pending:
            return 'resposta do Octomap Planner'

        if self._goal_active:
            return 'Octomap Planner finalizar o goal'

        if self._stop_request_pending:
            return 'servico stop do Octomap Planner'

        if not self._settle_time_elapsed():
            return 'estabilizacao apos fim do goal'

        if not self._min_command_interval_elapsed():
            return 'intervalo minimo entre comandos'

        if self._diagnostics_received and not self._planner_idle:
            return 'Octomap Planner ficar idle'

        return None

    def _send_current_waypoint(self, waypoint):
        return self.send_waypoint(waypoint)

    def send_waypoint(self, waypoint):
        wait_reason = self.waypoint_wait_reason()
        if wait_reason is not None:
            self.get_logger().info(
                f'WP not sent: waiting for {wait_reason}.',
                throttle_duration_sec=2.0,
            )
            return False

        if self._goal_request_pending:
            self.get_logger().info(
                'WP not sent: waiting for planner response.',
                throttle_duration_sec=2.0,
            )
            return False

        if self._goal_active:
            self.get_logger().info(
                'WP not sent: UAV is still travelling to the current WP.',
                throttle_duration_sec=2.0,
            )
            return False

        if not self._client.service_is_ready():
            self.get_logger().warn(
                f'Waiting for service {self.goto_service} before sending WP.',
                throttle_duration_sec=2.0,
            )
            return False

        waypoint = np.asarray(waypoint, dtype=float).reshape(-1)
        if waypoint.shape[0] < 3 or not np.all(np.isfinite(waypoint[:3])):
            self.get_logger().error(
                f'WP inválido recebido do planner: {waypoint.tolist()}'
            )
            return False

        waypoint = tuple(float(value) for value in waypoint[:3])
        goal_timeout_sec = self._goal_timeout_for(waypoint)
        self._current_goal = waypoint
        self._goal_active = False
        self._goal_request_pending = True
        self._planner_idle = False
        self._last_waypoint_sent_time = self.get_clock().now()
        self._goal_started_time = self._last_waypoint_sent_time
        self._goal_timeout_sec = goal_timeout_sec
        self._goal_finished_time = None
        self._stop_sent_for_goal = False
        self._saw_planner_busy_for_goal = False
        self._best_approach_distance = None
        self._best_approach_time = None
        self._watchdog_reference_position = (
            self._current_position.copy()
            if self._current_position is not None else None
        )
        self._watchdog_last_motion_time = self._goal_started_time
        self.arrived = False

        x, y, z = self._current_goal
        #z = 2.0
        yaw = 0.0
        self.get_logger().info(
            f' Sending WP to OctoMap planner: '
            f'x={x:.3f}  y={y:.3f}  z={z:.1f}  yaw={yaw:.2f} '
            f'timeout={goal_timeout_sec:.1f}s'
        )

        request = Vec4.Request()
        request.goal[0] = x
        request.goal[1] = y
        request.goal[2] = z
        request.goal[3] = yaw

        self._goto_future = self._client.call_async(request)
        self._goto_future.add_done_callback(self._goto_done_callback)
        self._publish_waypoint_command(x, y, z, yaw)
        return True

    def _publish_waypoint_command(self, x, y, z, yaw):
        payload = {
            'uav_name': self.uav_name,
            'source_node': self.get_name(),
            'topic': self.waypoint_command_topic,
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'yaw': float(yaw),
            'stamp_sec': float(self.get_clock().now().nanoseconds) * 1e-9,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._waypoint_command_pub.publish(msg)

    def _diagnostics_callback(self, msg):
        self._planner_idle = bool(msg.idle)
        self._diagnostics_received = True

        if not self._goal_active:
            return

        if not self._planner_idle:
            self._saw_planner_busy_for_goal = True
            return

        if self._saw_planner_busy_for_goal:
            distance = self._distance_to_current_goal()
            if distance is None:
                return

            if distance <= config.WAYPOINT_ARRIVAL_TOLERANCE:
                self._finish_goal_from_planner()
                return

            # Detecção de drone travado: o OctoMap Planner já está idle mas o
            # drone parou a ~1m do goal por obstacle avoidance radius (troncos).
            # Se não houve melhora significativa em STUCK_TIMEOUT segundos,
            # aceita a posição atual como chegada.
            now = self.get_clock().now()
            if (self._best_approach_distance is None
                    or distance < self._best_approach_distance - STUCK_IMPROVEMENT_THRESHOLD):
                self._best_approach_distance = distance
                self._best_approach_time = now
            else:
                elapsed = (now - self._best_approach_time).nanoseconds * 1e-9
                if elapsed >= STUCK_TIMEOUT:
                    self.get_logger().warn(
                        f'Drone travado a {distance:.2f} m do goal por '
                        f'{elapsed:.1f}s (melhor={self._best_approach_distance:.2f}m); '
                        f'OctoMap Planner idle — aceitando chegada.'
                    )
                    self._finish_goal_from_planner()
                    return

            self.get_logger().info(
                'Octomap Planner idle, mas UAV ainda não chegou ao '
                f'goal: distância={distance:.2f} m',
                throttle_duration_sec=1.0,
            )

    def _odom_callback(self, msg):
        """Cache the latest UAV position for goal-arrival validation."""
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=float)
        source_frame = msg.header.frame_id
        if not source_frame or source_frame == config.PLANNING_FRAME:
            self._current_position = position
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                config.PLANNING_FRAME,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                'Não foi possível transformar odometria %s -> %s: %s'
                % (source_frame, config.PLANNING_FRAME, exc),
                throttle_duration_sec=2.0,
            )
            return

        rotation = self._quaternion_rotation_matrix(
            transform.transform.rotation
        )
        translation = transform.transform.translation
        transformed = rotation.dot(position)
        transformed += np.array([
            translation.x,
            translation.y,
            translation.z,
        ])
        self._current_position = transformed

    @staticmethod
    def _quaternion_rotation_matrix(quaternion):
        """Convert a geometry quaternion to a 3-by-3 rotation matrix."""
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        norm = x * x + y * y + z * z + w * w
        if norm <= 1e-12:
            return np.eye(3)
        scale = 2.0 / norm
        return np.array([
            [1.0 - scale * (y * y + z * z),
             scale * (x * y - z * w),
             scale * (x * z + y * w)],
            [scale * (x * y + z * w),
             1.0 - scale * (x * x + z * z),
             scale * (y * z - x * w)],
            [scale * (x * z - y * w),
             scale * (y * z + x * w),
             1.0 - scale * (x * x + y * y)],
        ], dtype=float)

    def _distance_to_current_goal(self):
        """Return the current 3-D distance to the commanded goal."""
        if self._current_goal is None or self._current_position is None:
            return None
        goal = np.asarray(self._current_goal[:3], dtype=float)
        return float(np.linalg.norm(self._current_position - goal))

    # ------------------------------------------------------------------
    # Callback de resposta do serviço: meta aceita, não chegada
    # ------------------------------------------------------------------
    def _goto_done_callback(self, future):
        self._goal_request_pending = False
        self._goto_future = None
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'WP accepted by planner. '
                    f'Waiting for MRS planner to finish it. '
                    f'Planner msg: "{response.message}"'
                )
                self._goal_active = True
                self.arrived = False
            else:
                self.get_logger().error(
                    f'WP rejected by planner. '
                    f'Planner msg: "{response.message}".'
                )
                self._current_goal = None
                self._goal_timeout_sec = None
                self._clear_goal_watchdog_motion()
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')
            self._current_goal = None
            self._goal_timeout_sec = None
            self._clear_goal_watchdog_motion()

    def _finish_goal_from_planner(self):
        if not self._goal_active:
            return

        self.get_logger().info('Octomap Planner is idle; MRS goal finished.')
        self.arrived = True
        self._goal_active = False
        self._current_goal = None
        self._goal_finished_time = self.get_clock().now()
        self._goal_timeout_sec = None
        self._saw_planner_busy_for_goal = False
        self._best_approach_distance = None
        self._best_approach_time = None
        self._clear_goal_watchdog_motion()

    def _check_goal_watchdog(self):
        if (
            not self._goal_active
            or self._goal_started_time is None
            or self._stop_request_pending
        ):
            return

        timeout_sec = self._goal_timeout_sec
        if timeout_sec is None:
            timeout_sec = float(getattr(config, 'GOAL_TIMEOUT_MIN_SEC', 25.0))

        if self._current_position is None:
            self.get_logger().warn(
                'Goal watchdog aguardando odometria; não vou cortar sem '
                'saber se o drone está parado.',
                throttle_duration_sec=5.0,
            )
            return

        now = self.get_clock().now()
        if (
            self._watchdog_reference_position is None
            or self._watchdog_last_motion_time is None
        ):
            self._watchdog_reference_position = self._current_position.copy()
            self._watchdog_last_motion_time = now
            return

        motion_eps = float(getattr(config, 'GOAL_WATCHDOG_MOTION_EPS', 0.2))
        moved = float(np.linalg.norm(
            self._current_position - self._watchdog_reference_position
        ))
        if moved >= max(motion_eps, 0.0):
            self._watchdog_reference_position = self._current_position.copy()
            self._watchdog_last_motion_time = now
            return

        stopped_for = (
            now - self._watchdog_last_motion_time
        ).nanoseconds * 1e-9
        if stopped_for < timeout_sec:
            return

        distance = self._distance_to_current_goal()
        if distance is None:
            distance_text = 'nan'
        else:
            distance_text = f'{distance:.2f}'
        self.get_logger().warn(
            f'Drone aparentemente parado por {stopped_for:.1f}s '
            f'(mov={moved:.2f}m < {motion_eps:.2f}m, '
            f'dist_goal={distance_text}m); '
            f'calling {self.stop_service} as a watchdog.'
        )
        self._request_planner_stop()

    def _request_planner_stop(self):
        if self._stop_sent_for_goal or self._stop_request_pending:
            return

        if not self._stop_client.service_is_ready():
            self.get_logger().warn(
                f'Waiting for service {self.stop_service} to stop planner.',
                throttle_duration_sec=2.0,
            )
            return

        self._stop_sent_for_goal = True
        self._stop_request_pending = True
        self.get_logger().info(
            f'Calling {self.stop_service} to stop replanning.'
        )
        self._stop_future = self._stop_client.call_async(Trigger.Request())
        self._stop_future.add_done_callback(self._stop_done_callback)

    def _stop_done_callback(self, future):
        self._stop_request_pending = False
        self._stop_future = None
        self._goal_finished_time = self.get_clock().now()
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Octomap Planner stopped. Msg: "{response.message}"'
                )
            else:
                self.get_logger().warn(
                    f'Octomap Planner stop returned false. '
                    f'Msg: "{response.message}"'
                )
        except Exception as e:
            self.get_logger().error(f'Stop service call exception: {e}')

        self.arrived = False
        self._goal_active = False
        self._current_goal = None
        self._goal_timeout_sec = None
        self._saw_planner_busy_for_goal = False
        self._clear_goal_watchdog_motion()

    def _settle_time_elapsed(self):
        if self._goal_finished_time is None:
            return True

        elapsed = self.get_clock().now() - self._goal_finished_time
        return elapsed.nanoseconds >= int(WAYPOINT_SETTLE_TIME * 1e9)

    def _min_command_interval_elapsed(self):
        if self._last_waypoint_sent_time is None:
            return True

        elapsed = self.get_clock().now() - self._last_waypoint_sent_time
        return elapsed.nanoseconds >= int(MIN_WAYPOINT_COMMAND_INTERVAL * 1e9)

    def _goal_elapsed(self, seconds):
        if self._goal_started_time is None:
            return False

        elapsed = self.get_clock().now() - self._goal_started_time
        return elapsed.nanoseconds >= int(seconds * 1e9)

    def _clear_goal_watchdog_motion(self):
        self._watchdog_reference_position = None
        self._watchdog_last_motion_time = None

    def _goal_timeout_for(self, waypoint):
        """Return watchdog timeout scaled by commanded travel distance."""
        min_sec = float(getattr(config, 'GOAL_TIMEOUT_MIN_SEC', 25.0))
        max_sec = float(getattr(config, 'GOAL_TIMEOUT_MAX_SEC', min_sec))
        buffer_sec = float(getattr(config, 'GOAL_TIMEOUT_BUFFER_SEC', 8.0))
        speed_factor = float(getattr(config, 'GOAL_TIMEOUT_SPEED_FACTOR', 3.0))
        speed = max(float(getattr(config, 'REFERENCE_SPEED', 2.0)), 0.1)

        distance = None
        if self._current_position is not None:
            goal = np.asarray(waypoint[:3], dtype=float)
            distance = float(np.linalg.norm(self._current_position - goal))

        dynamic_sec = min_sec
        if distance is not None and np.isfinite(distance):
            dynamic_sec = buffer_sec + speed_factor * distance / speed

        timeout = max(min_sec, dynamic_sec)
        return min(max(timeout, min_sec), max(max_sec, min_sec))




def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
