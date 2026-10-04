#!/usr/bin/python3
"""Executor de waypoints via Nav2 (action NavigateToPose).

Substitui o DroneController (octomap_planner do MRS) para robôs terrestres
como o LIMO. Expõe a MESMA interface usada pelo gaussian_feeder e pelo
benchmark_planner: send_waypoint(), waypoint_wait_reason(),
is_ready_for_waypoint(), current_goal, goal_active e arrived.
"""

import json
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import numpy as np
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros

from informative_planner import config

WAYPOINT_SETTLE_TIME = 0.05          # [s]
MIN_WAYPOINT_COMMAND_INTERVAL = 0.1  # [s]

_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class Nav2Controller(Node):

    def __init__(
        self,
        robot_name='limo',
        action_name='/navigate_to_pose',
        odom_topic='/odom',
    ):
        super().__init__('nav2_controller')

        self.uav_name = str(robot_name)
        self.action_name = str(action_name)
        self.odom_topic = str(odom_topic)
        self.waypoint_command_topic = f'/{self.uav_name}/planner_waypoint_cmd'

        self._current_goal = None
        self._current_position = None
        self._goal_active = False
        self._goal_request_pending = False
        self._cancel_request_pending = False
        self._goal_handle = None
        # Incrementado a cada goal; callbacks de goals antigos são ignorados.
        self._goal_seq = 0
        self._last_waypoint_sent_time = None
        self._goal_started_time = None
        self._goal_timeout_sec = None
        self._goal_finished_time = None
        self._watchdog_reference_position = None
        self._watchdog_last_motion_time = None
        self.arrived = False
        self._halted = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name,
        )
        self._waypoint_command_pub = self.create_publisher(
            String,
            self.waypoint_command_topic,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
        )
        self._watchdog_timer = self.create_timer(0.5, self._check_goal_watchdog)

        self.get_logger().info(
            f'Nav2Controller: action={self.action_name} '
            f'odom={self.odom_topic} frame={config.PLANNING_FRAME}'
        )

    # ------------------------------------------------------------------
    # Interface usada pelo planner
    # ------------------------------------------------------------------
    @property
    def current_goal(self):
        """Return the active/requested goal as an immutable tuple."""
        if self._current_goal is None:
            return None
        return tuple(self._current_goal)

    @property
    def goal_active(self):
        """Return whether the robot is currently executing a goal."""
        return bool(self._goal_active)

    def is_ready_for_waypoint(self):
        return self.waypoint_wait_reason() is None

    def halt(self):
        """Fim de missão: recusa novos goals e cancela o goal ativo."""
        self._halted = True
        if self._goal_active or self._goal_request_pending:
            self.get_logger().info('Missão encerrada: cancelando goal no Nav2.')
            self._request_cancel()

    def waypoint_wait_reason(self):
        if self._halted:
            return 'missao encerrada'

        if not self._action_client.server_is_ready():
            return f'action server {self.action_name}'

        if self._goal_request_pending:
            return 'Nav2 aceitar o goal'

        if self._goal_active:
            return 'Nav2 finalizar o goal'

        if self._cancel_request_pending:
            return 'cancelamento do goal no Nav2'

        if not self._settle_time_elapsed():
            return 'estabilizacao apos fim do goal'

        if not self._min_command_interval_elapsed():
            return 'intervalo minimo entre comandos'

        return None

    def send_waypoint(self, waypoint):
        wait_reason = self.waypoint_wait_reason()
        if wait_reason is not None:
            self.get_logger().info(
                f'WP not sent: waiting for {wait_reason}.',
                throttle_duration_sec=2.0,
            )
            return False

        waypoint = np.asarray(waypoint, dtype=float).reshape(-1)
        if waypoint.shape[0] < 2 or not np.all(np.isfinite(waypoint[:2])):
            self.get_logger().error(
                f'WP inválido recebido do planner: {waypoint.tolist()}'
            )
            return False

        z = float(waypoint[2]) if waypoint.shape[0] >= 3 else 0.0
        waypoint = (float(waypoint[0]), float(waypoint[1]), z)
        goal_timeout_sec = self._goal_timeout_for(waypoint)
        self._goal_seq += 1
        self._current_goal = waypoint
        self._goal_active = False
        self._goal_request_pending = True
        self._goal_handle = None
        self._last_waypoint_sent_time = self.get_clock().now()
        self._goal_started_time = self._last_waypoint_sent_time
        self._goal_timeout_sec = goal_timeout_sec
        self._goal_finished_time = None
        self._watchdog_reference_position = (
            self._current_position.copy()
            if self._current_position is not None else None
        )
        self._watchdog_last_motion_time = self._goal_started_time
        self.arrived = False

        x, y, _ = waypoint
        # Orientação final = direção de deslocamento, para o Nav2 não girar
        # no lugar ao chegar (goal checker também confere yaw).
        yaw = self._heading_to(x, y)
        self.get_logger().info(
            f' Sending WP to Nav2: x={x:.3f}  y={y:.3f}  yaw={yaw:.2f} '
            f'timeout={goal_timeout_sec:.1f}s'
        )

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)
        seq = self._goal_seq
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda fut: self._goal_response_callback(fut, seq)
        )
        self._publish_waypoint_command(x, y, z, yaw)
        return True

    # ------------------------------------------------------------------
    # Callbacks do action
    # ------------------------------------------------------------------
    def _goal_response_callback(self, future, seq):
        if seq != self._goal_seq:
            return
        self._goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'NavigateToPose send_goal exception: {e}')
            self._drop_goal()
            return

        if not goal_handle.accepted:
            self.get_logger().error('WP rejected by Nav2.')
            self._drop_goal()
            return

        self.get_logger().info('WP accepted by Nav2. Waiting for result.')
        self._goal_handle = goal_handle
        self._goal_active = True
        self.arrived = False
        goal_handle.get_result_async().add_done_callback(
            lambda fut: self._result_callback(fut, seq)
        )
        if self._halted:
            # halt() chegou enquanto o goal ainda estava sendo aceito.
            self._request_cancel()

    def _result_callback(self, future, seq):
        if seq != self._goal_seq:
            return
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'NavigateToPose result exception: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        distance = self._distance_to_current_goal()
        distance_text = 'nan' if distance is None else f'{distance:.2f}'
        status_name = _STATUS_NAMES.get(status, str(status))

        if status == GoalStatus.STATUS_SUCCEEDED:
            arrived = True
        elif (
            status == GoalStatus.STATUS_ABORTED
            and distance is not None
            and distance <= config.NAV2_ABORT_ACCEPT_DISTANCE
        ):
            # Mesmo papel do "drone travado perto do goal" no backend MRS.
            self.get_logger().warn(
                f'Nav2 abortou a {distance_text} m do goal; aceitando chegada.'
            )
            arrived = True
        else:
            arrived = False

        self.get_logger().info(
            f'Nav2 goal finished: status={status_name} '
            f'dist_goal={distance_text}m arrived={arrived}'
        )
        self._finish_goal(arrived)

    def _finish_goal(self, arrived):
        self.arrived = bool(arrived)
        self._goal_active = False
        self._goal_request_pending = False
        self._goal_handle = None
        self._current_goal = None
        self._goal_finished_time = self.get_clock().now()
        self._goal_timeout_sec = None
        self._clear_goal_watchdog_motion()

    def _drop_goal(self):
        self._goal_request_pending = False
        self._goal_active = False
        self._goal_handle = None
        self._current_goal = None
        self._goal_timeout_sec = None
        self._clear_goal_watchdog_motion()

    # ------------------------------------------------------------------
    # Watchdog: robô parado por muito tempo -> cancela o goal
    # ------------------------------------------------------------------
    def _check_goal_watchdog(self):
        if (
            not self._goal_active
            or self._goal_started_time is None
            or self._cancel_request_pending
        ):
            return

        timeout_sec = self._goal_timeout_sec
        if timeout_sec is None:
            timeout_sec = float(getattr(config, 'GOAL_TIMEOUT_MIN_SEC', 25.0))

        if self._current_position is None:
            self.get_logger().warn(
                'Goal watchdog aguardando odometria; não vou cortar sem '
                'saber se o robô está parado.',
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
            self._current_position[:2]
            - self._watchdog_reference_position[:2]
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
        distance_text = 'nan' if distance is None else f'{distance:.2f}'
        self.get_logger().warn(
            f'Robô aparentemente parado por {stopped_for:.1f}s '
            f'(mov={moved:.2f}m < {motion_eps:.2f}m, '
            f'dist_goal={distance_text}m); cancelando goal no Nav2.'
        )
        self._request_cancel()

    def _request_cancel(self):
        if self._goal_handle is None or self._cancel_request_pending:
            return
        self._cancel_request_pending = True
        seq = self._goal_seq
        self._goal_handle.cancel_goal_async().add_done_callback(
            lambda fut: self._cancel_done_callback(fut, seq)
        )

    def _cancel_done_callback(self, future, seq):
        self._cancel_request_pending = False
        if seq != self._goal_seq:
            return
        try:
            accepted = len(future.result().goals_canceling) > 0
        except Exception as e:
            self.get_logger().error(f'NavigateToPose cancel exception: {e}')
            accepted = False
        if not accepted:
            self.get_logger().warn('Nav2 recusou o cancelamento do goal.')
        # Invalida o resultado que ainda vier deste goal (CANCELED).
        self._goal_seq += 1
        self._finish_goal(arrived=False)

    # ------------------------------------------------------------------
    # Odometria / geometria
    # ------------------------------------------------------------------
    def _odom_callback(self, msg):
        """Cache the latest robot position in PLANNING_FRAME."""
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

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = config.PLANNING_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _heading_to(self, x, y):
        if self._current_position is None:
            return 0.0
        dx = x - float(self._current_position[0])
        dy = y - float(self._current_position[1])
        if math.hypot(dx, dy) < 1e-3:
            return 0.0
        return math.atan2(dy, dx)

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
        """Return the current planar (x, y) distance to the goal."""
        if self._current_goal is None or self._current_position is None:
            return None
        goal = np.asarray(self._current_goal[:2], dtype=float)
        return float(np.linalg.norm(self._current_position[:2] - goal))

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

    def _clear_goal_watchdog_motion(self):
        self._watchdog_reference_position = None
        self._watchdog_last_motion_time = None

    def _goal_timeout_for(self, waypoint):
        """Return watchdog timeout scaled by commanded travel distance."""
        min_sec = float(getattr(config, 'GOAL_TIMEOUT_MIN_SEC', 25.0))
        max_sec = float(getattr(config, 'GOAL_TIMEOUT_MAX_SEC', min_sec))
        buffer_sec = float(getattr(config, 'GOAL_TIMEOUT_BUFFER_SEC', 8.0))
        speed_factor = float(getattr(config, 'GOAL_TIMEOUT_SPEED_FACTOR', 3.0))
        speed = max(float(getattr(config, 'REFERENCE_SPEED', 0.5)), 0.1)

        distance = None
        if self._current_position is not None:
            goal = np.asarray(waypoint[:2], dtype=float)
            distance = float(np.linalg.norm(self._current_position[:2] - goal))

        dynamic_sec = min_sec
        if distance is not None and np.isfinite(distance):
            dynamic_sec = buffer_sec + speed_factor * distance / speed

        timeout = max(min_sec, dynamic_sec)
        return min(max(timeout, min_sec), max(max_sec, min_sec))
