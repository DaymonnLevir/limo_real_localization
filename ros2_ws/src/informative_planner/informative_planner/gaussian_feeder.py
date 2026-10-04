#!/usr/bin/env python3

"""ROS2 node that updates and displays the exploration Gaussian map."""

import csv
import fcntl
import json
import os
import pathlib
import signal
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tree_mapper.msg import ObjectDetectionArray

from informative_planner import config
from informative_planner.controllers import build_controller
from informative_planner.controllers import declare_controller_parameters
from informative_planner.exploration_gp import ExplorationGP
from informative_planner.planner import find_next_waypoint_ucb
from informative_planner.rrt import _is_collision_free
from informative_planner.utils import generate_waypoint_candidates
from informative_planner.visited_map import VisitedMap


@dataclass(frozen=True)
class MappedTree:
    """Tree obstacle received from tree_mapper."""

    tree_id: int
    x: float
    y: float
    radius: float


VALID_MAP_SOURCES = ('kalman', 'gp_predict')


def compute_gp_field(
    exploration_gp: ExplorationGP,
    bounds: tuple[float, float, float, float],
    resolution: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Calculate GP mean/std over a regular grid for visualization."""
    if resolution is None:
        resolution = config.INFO_GAIN_MAP_RESOLUTION

    x_min, x_max, y_min, y_max = bounds
    xs = np.arange(x_min, x_max + 0.5 * resolution, resolution)
    ys = np.arange(y_min, y_max + 0.5 * resolution, resolution)

    xx, yy = np.meshgrid(xs, ys)
    x_test = np.column_stack([xx.ravel(), yy.ravel()])
    mean_flat, std_flat = exploration_gp.predict(x_test)

    gp_mean = mean_flat.reshape(len(ys), len(xs))
    gp_std = std_flat.reshape(len(ys), len(xs))

    return xs, ys, gp_mean, gp_std


def compute_kalman_grid_field(
    exploration_gp: ExplorationGP,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return posterior mean/std directly on the Kalman candidate grid."""
    grid_xy = exploration_gp.X_grid
    xs = np.unique(grid_xy[:, 0])
    ys = np.unique(grid_xy[:, 1])

    gp_mean = np.full((len(ys), len(xs)), np.nan)
    gp_std = np.full((len(ys), len(xs)), np.nan)
    std_values = np.sqrt(np.maximum(np.diag(exploration_gp.cov_), 0.0))

    x_to_idx = {float(x): i for i, x in enumerate(xs)}
    y_to_idx = {float(y): i for i, y in enumerate(ys)}

    for pos, mean, std in zip(grid_xy, exploration_gp.mean_, std_values):
        x_idx = x_to_idx[float(pos[0])]
        y_idx = y_to_idx[float(pos[1])]
        gp_mean[y_idx, x_idx] = mean
        gp_std[y_idx, x_idx] = std

    return xs, ys, gp_mean, gp_std


def grid_extent(
    xs: np.ndarray,
    ys: np.ndarray,
) -> tuple[float, float, float, float]:
    """Return imshow extent using grid centers."""
    dx = float(np.median(np.diff(xs))) if len(xs) > 1 else 1.0
    dy = float(np.median(np.diff(ys))) if len(ys) > 1 else 1.0

    return (
        float(xs[0] - 0.5 * dx),
        float(xs[-1] + 0.5 * dx),
        float(ys[0] - 0.5 * dy),
        float(ys[-1] + 0.5 * dy),
    )


class GaussianFeeder(Node):
    """Feed point-quality observations into the GP and show a live map."""

    def __init__(self):
        """Create subscriptions, GP state, and the live matplotlib figure."""
        super().__init__('gaussian_feeder')

        self.last_drone_position = None
        self.uav_name = str(self.declare_parameter('uav_name', 'limo').value)
        self.show_plot = bool(
            self.declare_parameter('show_plot', True).value
        )
        # Estado do plot publicado em /<uav>/ipp_viz (JSON) para desenhar a mesma
        # janela fora do robô (scripts/ipp_viewer.py), sem matplotlib no LIMO.
        self.publish_viz = bool(
            self.declare_parameter('publish_viz', True).value
        )
        self.viz_pub = None
        if self.publish_viz:
            self.viz_pub = self.create_publisher(
                String,
                '/%s/ipp_viz' % self.uav_name,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        self._instance_lock_handle = None
        self._acquire_instance_lock()
        self._output_dir = str(self.declare_parameter('output_dir', '').value)
        # false: planeja sem /tree_map_full (RRT sem obstáculos de tronco; o
        # costmap do Nav2 desvia). Útil quando não há nuvem 3D p/ o tree_mapper.
        self.require_tree_map = bool(
            self.declare_parameter('require_tree_map', True).value
        )
        self.mission_done_topic = str(
            self.declare_parameter(
                'mission_done_topic',
                config.MISSION_DONE_TOPIC,
            ).value or config.MISSION_DONE_TOPIC
        )
        self._mission_done = False
        self._final_outputs_saved = False
        map_bounds = self.declare_parameter(
            'map_bounds',
            list(config.MAP_BOUNDS),
        ).value
        if len(map_bounds) != 4:
            raise ValueError(
                'map_bounds must contain [x_min, x_max, y_min, y_max]'
            )
        self.bounds = tuple(float(value) for value in map_bounds)
        self.observed_positions = []
        self.observed_scores = []
        self.map_source = self._get_configured_map_source()
        self.tree_map = None
        self.tree_map_frame_id = None
        self.tree_map_received_at = None
        self.pending_waypoint = None
        self.pending_score = float('-inf')
        self.pending_for_goal = None
        self.pending_tree_ids = frozenset()
        self.pending_plan_info = None
        self.pending_gp_snapshot = None
        self.preplanned_for_goal = None
        self.committed_path_queue = []
        self.committed_plan_info = None
        # Último path do RRT enviado (H pontos, sem a origem), só para o plot.
        self.last_plan_path = []
        self.committed_tree_ids = frozenset()
        self.committed_path_total = 0

        # Executor de waypoints: 'nav2' (NavigateToPose) ou 'mrs'.
        backend, navigate_action, odom_topic = (
            declare_controller_parameters(self)
        )
        self.drone_controler = build_controller(
            backend,
            self.uav_name,
            navigate_action,
            odom_topic,
        )

        self.point_quality_sub = self.create_subscription(
            String,
            '/%s/point_quality' % self.uav_name,
            self.point_quality_callback,
            10,
        )

        self.tree_map_sub = self.create_subscription(
            ObjectDetectionArray,
            '/tree_map_full',
            self.tree_map_callback,
            1,
        )

        self.mission_done_sub = self.create_subscription(
            String,
            self.mission_done_topic,
            self.mission_done_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.candidates = generate_waypoint_candidates(self.bounds)

        self.exploration_gp = ExplorationGP(
            candidates_xy=self.candidates[:, :2],
            length_scale=config.GP_ELL,
            signal_std=config.GP_SIGMA_S,
            noise_std=config.GP_SIGMA_N,
            beta=config.GP_BETA,
            observation_model=config.GP_OBSERVATION_MODEL,
            observation_footprint_radius=(
                config.GP_OBSERVATION_FOOTPRINT_RADIUS
            ),
            observation_footprint_sigma=config.GP_OBSERVATION_FOOTPRINT_SIGMA,
        )
        self.visited_map = VisitedMap(
            self.exploration_gp.observation_model,
            saturation=config.RRT_VISIT_SATURATION,
            decay_halflife_observations=getattr(
                config,
                'RRT_VISIT_DECAY_HALFLIFE_OBS',
                None,
            ),
        )

        if self.show_plot:
            self._init_plot()
        self._publish_viz(None)
        self.get_logger().info(
            'Gaussian feeder pronto: aguardando mensagens em '
            f'/{self.uav_name}/point_quality | mapa={self.map_source} | '
            f'H={self.exploration_gp.observation_model.describe()} | '
            f'visited={self.visited_map.describe()} | '
            f'commit={self._configured_commit_count()} | '
            f'preplan_gp_change_max='
            f'{self._preplan_gp_change_threshold():.3f} | '
            f'bounds={self.bounds} | show_plot={self.show_plot}'
        )

        self.point_quality_period = Duration(seconds=0.5)  # 2 Hz
        self.last_point_quality_time = None

    def tree_map_callback(self, msg: ObjectDetectionArray):
        """Armazene o snapshot confirmado mais recente."""
        if msg.header.frame_id != config.PLANNING_FRAME:
            self.get_logger().error(
                'Ignorando /tree_map_full em frame inesperado: '
                f'{msg.header.frame_id!r}; esperado={config.PLANNING_FRAME!r}',
                throttle_duration_sec=2.0,
            )
            return

        tree_map = tuple(
            MappedTree(
                tree_id=int(detection.id),
                x=float(detection.pose.position.x),
                y=float(detection.pose.position.y),
                radius=max(float(detection.radius), 0.0),
            )
            for detection in msg.detections
        )

        self.tree_map = tree_map
        self.tree_map_frame_id = msg.header.frame_id
        self.tree_map_received_at = self.get_clock().now()

        self.get_logger().info(
            f'Mapa atualizado: {len(tree_map)} árvores; '
            f'frame={self.tree_map_frame_id}',
            throttle_duration_sec=2.0,
        )

    def _acquire_instance_lock(self):
        """Prevent two planner instances from commanding the same UAV."""
        safe_uav_name = ''.join(
            char if char.isalnum() or char in ('-', '_') else '_'
            for char in self.uav_name
        )
        lock_path = pathlib.Path('/tmp') / (
            f'informative_planner_gaussian_feeder_{safe_uav_name}.lock'
        )
        handle = open(lock_path, 'a+', encoding='utf-8')
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            handle.seek(0)
            owner_pid = handle.read().strip() or 'desconhecido'
            handle.close()
            raise RuntimeError(
                f'Já existe um gaussian_feeder para {self.uav_name}; '
                f'PID registrado={owner_pid}'
            ) from exc

        handle.seek(0)
        handle.truncate()
        handle.write(str(os.getpid()))
        handle.flush()
        self._instance_lock_handle = handle

    def _release_instance_lock(self):
        """Release the per-UAV planner lock if held by this process."""
        if self._instance_lock_handle is None:
            return
        try:
            fcntl.flock(
                self._instance_lock_handle.fileno(),
                fcntl.LOCK_UN,
            )
        finally:
            self._instance_lock_handle.close()
            self._instance_lock_handle = None

    def mission_done_callback(self, msg):
        """Fim do budget: para o robô e salva os dados do GP uma vez."""
        if self._mission_done:
            return
        self._mission_done = True
        self.get_logger().info(
            f'Fim de missão recebido ({msg.data}); parando o robô e salvando.'
        )
        halt = getattr(self.drone_controler, 'halt', None)
        if halt is not None:
            halt()
        self._save_final_outputs()

    def point_quality_callback(self, msg):
        """Update the GP and redraw the map from a point_quality message."""
        if self._mission_done:
            return
        now = self.get_clock().now()
        if self.last_point_quality_time is not None:
            elapsed_ns = now.nanoseconds - self.last_point_quality_time.nanoseconds
            if 0 <= elapsed_ns < self.point_quality_period.nanoseconds:
                return  # Limita a taxa de processamento para evitar sobrecarga
        self.last_point_quality_time = now
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(
                f'Mensagem point_quality inválida: {msg.data}'
            )
            return

        try:
            position_x = self._get_first_available(
                data,
                'drone_position_x',
                'position_x',
            )
            position_y = self._get_first_available(
                data,
                'drone_position_y',
                'position_y',
            )
            drone_position = np.array(
                [
                    float(position_x),
                    float(position_y),
                ],
                dtype=float,
            )
            point_score = float(data['point_score'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                'Mensagem point_quality sem posição ou point_score válidos'
            )
            return

        if (
            not np.all(np.isfinite(drone_position))
            or not np.isfinite(point_score)
        ):
            self.get_logger().warn(
                'Mensagem point_quality contém NaN ou infinito'
            )
            return

        position_is_new = not (
            self.last_drone_position is not None
            and np.allclose(drone_position, self.last_drone_position)
        )
        if position_is_new:
            self.last_drone_position = drone_position.copy()
            self.exploration_gp.add_observation(drone_position, point_score)
            self.visited_map.add_observation(drone_position)
            self.observed_positions.append(drone_position.copy())
            self.observed_scores.append(point_score)
            self._publish_viz(drone_position)
            if self.show_plot:
                self._update_plot(drone_position)

        wait_reason = self.drone_controler.waypoint_wait_reason()
        if wait_reason is not None:
            if not self._has_committed_path():
                self._maybe_preplan(drone_position)
            self.get_logger().info(
                f'GP atualizado: pos=({drone_position[0]:.2f}, '
                f'{drone_position[1]:.2f}), score={point_score:.2f}, '
                f'obs={self.exploration_gp.n_observations} | '
                f'aguardando {wait_reason}',
                throttle_duration_sec=1.0,
            )
            return

        if self.tree_map is None and not self.require_tree_map:
            self.get_logger().warn(
                'require_tree_map=false: planejando sem /tree_map_full '
                '(colisão com troncos fica a cargo do Nav2).'
            )
            self.tree_map = ()
            self.tree_map_frame_id = config.PLANNING_FRAME

        if self.tree_map is None:
            self.get_logger().warn(
                'Aguardando o primeiro snapshot em /tree_map_full',
                throttle_duration_sec=2.0,
            )
            return

        best_next, best_score, plan_info = self._peek_committed_path()
        plan_source = 'committed'
        if best_next is None:
            best_next, best_score, plan_info = self._consume_preplan()
            plan_source = 'preplan'
        if best_next is None:
            best_next, best_score, plan_info = self._plan_from(
                drone_position
            )
            plan_source = 'fresh'
        if best_next is None:
            self.get_logger().warn(
                'Nenhum waypoint viável encontrado dentro do passo máximo; '
                'mantendo UAV no objetivo atual.',
                throttle_duration_sec=2.0,
            )
            return

        self.get_logger().info(
            f'best_next: [{best_next[0]:.3f}, {best_next[1]:.3f}, '
            f'{best_next[2]:.1f}] score={best_score:.3f}',
            throttle_duration_sec=1.0,
        )
        self._log_plan_info(plan_info, plan_source)
        sent = False
        try:
            sent = self.drone_controler.send_waypoint(np.array(best_next))
        except Exception as e:
            self.get_logger().error(f'Erro ao enviar waypoint: {e}')
        if sent:
            if plan_source == 'committed':
                self._advance_committed_path()
            elif plan_source in ('fresh', 'preplan'):
                self._commit_plan_tail(plan_info)
                self._remember_plan_path(plan_info)
            self._reset_preplan()
            self._publish_viz(drone_position)
            if self.show_plot:
                self._update_plot(drone_position)

        self.get_logger().info(
            f'GP atualizado: pos=({drone_position[0]:.2f}, '
            f'{drone_position[1]:.2f}), score={point_score:.2f}, '
            f'obs={self.exploration_gp.n_observations}'
        )

    def _plan_from(self, start_xy):
        """Calculate a next waypoint from a given planning origin."""
        return find_next_waypoint_ucb(
            exploration_gp=self.exploration_gp,
            candidates=self.candidates,
            drone_xy=np.asarray(start_xy[:2], dtype=float),
            trees=self.tree_map,
            min_dist=1.5,
            max_step=25.0,
            alpha=1.5,
            beta=0.3,
            map_bounds=self.bounds,
            visited_map=self.visited_map,
            return_plan_info=True,
        )

    @staticmethod
    def _format_float(value):
        """Format a numeric diagnostic value for compact ROS logs."""
        try:
            return f'{float(value):.3f}'
        except (TypeError, ValueError):
            return 'nan'

    @staticmethod
    def _format_xy_pair(xy):
        """Format a 2-D point for compact ROS logs."""
        if xy is None:
            return '[nan, nan]'
        try:
            return f'[{float(xy[0]):.2f}, {float(xy[1]):.2f}]'
        except (IndexError, TypeError, ValueError):
            return '[nan, nan]'

    def _format_plan_path(self, path):
        """Format the selected RRT path as a short list of xy pairs."""
        if not path:
            return '[]'
        return '[' + ', '.join(
            self._format_xy_pair(point) for point in path
        ) + ']'

    def _log_plan_info(self, plan_info, source):
        """Log path-level diagnostics for the selected planner decision."""
        if not plan_info:
            return

        planner = plan_info.get('planner', 'planner')
        label = 'RRT path' if planner == 'rrt_informative' else 'Plan path'
        details = (
            f'{label} {source}: '
            f'first={self._format_xy_pair(plan_info.get("first"))} '
            f'leaf={self._format_xy_pair(plan_info.get("leaf"))} '
            f'path_score={self._format_float(plan_info.get("path_score"))} '
            f'ucb_accum={self._format_float(plan_info.get("ucb_accum"))} '
            f'info_gain_accum='
            f'{self._format_float(plan_info.get("info_gain_accum"))} '
            f'novelty_accum='
            f'{self._format_float(plan_info.get("novelty_accum"))} '
            f'revisit_accum='
            f'{self._format_float(plan_info.get("revisit_accum"))} '
            f'cost={self._format_float(plan_info.get("cost"))}'
        )

        if 'path' in plan_info:
            details += f' path={self._format_plan_path(plan_info["path"])}'

        details += (
            f' depth={plan_info.get("depth", "nan")} '
            f'nodes={plan_info.get("node_count", "nan")} '
            f'mode={plan_info.get("score_mode", "unknown")}'
        )
        if 'committed_index' in plan_info:
            details += (
                f' committed={plan_info.get("committed_index", "nan")}/'
                f'{plan_info.get("committed_total", "nan")} '
                f'queued_after='
                f'{plan_info.get("committed_remaining_after_send", "nan")} '
                f'waypoint='
                f'{self._format_xy_pair(plan_info.get("committed_waypoint"))}'
            )
        if 'gp_change' in plan_info:
            details += (
                f' gp_change={self._format_float(plan_info.get("gp_change"))}'
                f' gp_mean_change='
                f'{self._format_float(plan_info.get("gp_change_mean"))}'
                f' gp_std_change='
                f'{self._format_float(plan_info.get("gp_change_std"))}'
                f' gp_ucb_change='
                f'{self._format_float(plan_info.get("gp_change_ucb"))}'
            )
        self.get_logger().info(details)

    @staticmethod
    def _configured_commit_count():
        """Return how many RRT waypoints should be followed per replan."""
        try:
            count = int(getattr(config, 'RRT_COMMITTED_WAYPOINTS', 1))
        except (TypeError, ValueError):
            count = 1
        return max(count, 1)

    def _has_committed_path(self):
        """Return whether a committed path tail is available."""
        if self._configured_commit_count() <= 1:
            self._clear_committed_path()
            return False
        return bool(self.committed_path_queue)

    def _commit_plan_tail(self, plan_info):
        """Commit the first N waypoints of a selected RRT path."""
        self._clear_committed_path()

        commit_count = self._configured_commit_count()
        if commit_count <= 1 or not plan_info:
            return
        if plan_info.get('planner') != 'rrt_informative':
            return

        path = plan_info.get('path') or []
        if len(path) <= 1:
            return

        total = min(commit_count, len(path))
        for index, xy in enumerate(path[1:total], start=2):
            try:
                point_xy = np.asarray(xy[:2], dtype=float)
            except (IndexError, TypeError, ValueError):
                continue
            if point_xy.shape[0] < 2 or not np.all(np.isfinite(point_xy)):
                continue

            waypoint = np.array([
                float(point_xy[0]),
                float(point_xy[1]),
                config.FLIGHT_HEIGHT,
            ])
            self.committed_path_queue.append({
                'waypoint': waypoint,
                'index': index,
            })

        if not self.committed_path_queue:
            return

        self.committed_plan_info = dict(plan_info)
        self.committed_tree_ids = self._current_tree_ids()
        self.committed_path_total = total
        self.get_logger().info(
            'Committed RRT path armado: '
            f'seguindo {total} waypoints deste RRT; '
            f'fila={len(self.committed_path_queue)} | '
            f'leaf={self._format_xy_pair(plan_info.get("leaf"))} '
            f'path_score={self._format_float(plan_info.get("path_score"))}'
        )

    def _peek_committed_path(self):
        """Return the next committed waypoint without removing it yet."""
        if not self._has_committed_path():
            return None, float('-inf'), None

        if not self.drone_controler.arrived:
            self.get_logger().info(
                'Descartando committed RRT path: goal anterior não concluiu '
                'normalmente.'
            )
            self._clear_committed_path()
            return None, float('-inf'), None

        entry = self.committed_path_queue[0]
        waypoint = entry['waypoint'].copy()

        tree_ids = self._current_tree_ids()
        if tree_ids != self.committed_tree_ids:
            if not self._committed_segment_collision_free(waypoint):
                self.get_logger().info(
                    'Descartando committed RRT path: mapa de árvores mudou '
                    'e o próximo segmento não é mais collision-free.'
                )
                self._clear_committed_path()
                return None, float('-inf'), None
            self.committed_tree_ids = tree_ids
            self.get_logger().info(
                'Mantendo committed RRT path após mudança no mapa: '
                'próximo segmento continua collision-free.',
                throttle_duration_sec=2.0,
            )

        plan_info = (
            dict(self.committed_plan_info)
            if self.committed_plan_info else {}
        )
        plan_info['committed_index'] = int(entry['index'])
        plan_info['committed_total'] = int(self.committed_path_total)
        plan_info['committed_remaining_after_send'] = (
            len(self.committed_path_queue) - 1
        )
        plan_info['committed_waypoint'] = (
            float(waypoint[0]),
            float(waypoint[1]),
        )

        score = self._current_ucb_at(waypoint)
        if not np.isfinite(score):
            score = float(plan_info.get(
                'first_waypoint_score',
                plan_info.get('path_score', float('-inf')),
            ))
        return waypoint, score, plan_info

    def _advance_committed_path(self):
        """Drop the committed waypoint that was just accepted."""
        if self.committed_path_queue:
            self.committed_path_queue.pop(0)
        if not self.committed_path_queue:
            self._clear_committed_path()

    def _clear_committed_path(self):
        """Clear the committed RRT path tail."""
        self.committed_path_queue = []
        self.committed_plan_info = None
        self.committed_tree_ids = frozenset()
        self.committed_path_total = 0

    def _committed_segment_collision_free(self, waypoint):
        """Return whether current position -> committed waypoint is feasible."""
        if self.last_drone_position is None:
            return False
        start_xy = np.asarray(self.last_drone_position[:2], dtype=float)
        goal_xy = np.asarray(waypoint[:2], dtype=float)
        return _is_collision_free(start_xy, goal_xy, self._tree_obstacles())

    def _tree_obstacles(self):
        """Return current tree map as circular RRT obstacles."""
        return [
            (float(tree.x), float(tree.y), max(float(tree.radius), 0.0))
            for tree in (self.tree_map or [])
        ]

    def _current_ucb_at(self, waypoint):
        """Return current nearest-grid UCB at a waypoint for logging."""
        try:
            xy = np.asarray(waypoint[:2], dtype=float)
            values = np.asarray(self.exploration_gp.ucb_grid(), dtype=float)
            distances = np.linalg.norm(self.candidates[:, :2] - xy, axis=1)
            return float(values[int(np.argmin(distances))])
        except (IndexError, TypeError, ValueError):
            return float('nan')

    def _gp_map_snapshot(self):
        """Return the GP decision-map state used to validate cached plans."""
        std = np.sqrt(np.maximum(np.diag(self.exploration_gp.cov_), 0.0))
        return {
            'mean': self.exploration_gp.mean_.copy(),
            'std': std,
            'ucb': np.asarray(self.exploration_gp.ucb_grid(), dtype=float),
        }

    @staticmethod
    def _normalized_rms_delta(current, reference, denominator):
        """Return RMS(current-reference) normalized by a stable denominator."""
        current = np.asarray(current, dtype=float)
        reference = np.asarray(reference, dtype=float)
        denominator = max(float(denominator), 1e-9)
        if current.shape != reference.shape:
            return float('inf')
        diff = current - reference
        return float(np.sqrt(np.mean(diff * diff)) / denominator)

    def _gp_map_change(self, snapshot):
        """Measure how much the current GP map diverged from a snapshot."""
        if not snapshot:
            return {
                'change': float('inf'),
                'mean': float('inf'),
                'std': float('inf'),
                'ucb': float('inf'),
            }

        current = self._gp_map_snapshot()
        signal_scale = max(float(config.GP_SIGMA_S), 1.0)
        ucb_scale = max(
            float(np.ptp(snapshot['ucb'])),
            float(config.GP_BETA) * signal_scale,
            1.0,
        )

        mean_change = self._normalized_rms_delta(
            current['mean'],
            snapshot['mean'],
            signal_scale,
        )
        std_change = self._normalized_rms_delta(
            current['std'],
            snapshot['std'],
            signal_scale,
        )
        ucb_change = self._normalized_rms_delta(
            current['ucb'],
            snapshot['ucb'],
            ucb_scale,
        )

        return {
            'change': max(mean_change, std_change, ucb_change),
            'mean': mean_change,
            'std': std_change,
            'ucb': ucb_change,
        }

    def _preplan_gp_change_threshold(self):
        """Return configured GP-map change threshold for cached preplans."""
        try:
            return float(getattr(
                config,
                'RRT_PREPLAN_GP_CHANGE_THRESHOLD',
                0.20,
            ))
        except (TypeError, ValueError):
            return 0.20

    def _maybe_preplan(self, drone_position):
        """Precompute the following goal near the active goal."""
        if config.RRT_PREPLAN_DISTANCE <= 0.0 or self.tree_map is None:
            return

        current_goal = self.drone_controler.current_goal
        if current_goal is None or not self.drone_controler.goal_active:
            return
        if self.preplanned_for_goal == current_goal:
            return

        goal_xy = np.asarray(current_goal[:2], dtype=float)
        distance_to_goal = float(np.linalg.norm(drone_position - goal_xy))
        if distance_to_goal > config.RRT_PREPLAN_DISTANCE:
            return

        # Mark before computing so this goal is attempted only once, even if
        # the stochastic planner cannot produce a path on this attempt.
        self.preplanned_for_goal = current_goal
        waypoint, score, plan_info = self._plan_from(goal_xy)
        if waypoint is None:
            self.get_logger().warn(
                'Pré-planejamento não encontrou um waypoint; '
                'uma nova tentativa será feita após a chegada.',
                throttle_duration_sec=2.0,
            )
            return

        self.pending_waypoint = np.asarray(waypoint, dtype=float)
        self.pending_score = float(score)
        self.pending_for_goal = current_goal
        self.pending_tree_ids = self._current_tree_ids()
        self.pending_plan_info = dict(plan_info) if plan_info else None
        self.pending_gp_snapshot = self._gp_map_snapshot()
        self.get_logger().info(
            f'Próximo waypoint pré-planejado a {distance_to_goal:.2f} m '
            f'do goal atual: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, '
            f'{waypoint[2]:.2f}]'
        )
        self._log_plan_info(self.pending_plan_info, 'precomputed')

    def _consume_preplan(self):
        """Return a valid pending plan after normal goal completion."""
        if self.pending_waypoint is None:
            return None, float('-inf'), None

        if not (
            self.drone_controler.arrived
            and self.pending_for_goal == self.preplanned_for_goal
        ):
            self.get_logger().info(
                'Descartando pré-plano: goal anterior não concluiu '
                'normalmente.'
            )
            self._reset_preplan()
            return None, float('-inf'), None

        if not self._committed_segment_collision_free(self.pending_waypoint):
            self.get_logger().info(
                'Descartando pré-plano: próximo segmento não é mais '
                'collision-free.'
            )
            self._reset_preplan()
            return None, float('-inf'), None

        change_metrics = self._gp_map_change(self.pending_gp_snapshot)
        change_threshold = self._preplan_gp_change_threshold()
        if (
            change_threshold > 0.0
            and change_metrics['change'] > change_threshold
        ):
            self.get_logger().info(
                'Descartando pré-plano: mapa GP mudou além do limiar '
                f'({change_metrics["change"]:.3f} > '
                f'{change_threshold:.3f}; '
                f'mean={change_metrics["mean"]:.3f}, '
                f'std={change_metrics["std"]:.3f}, '
                f'ucb={change_metrics["ucb"]:.3f}).'
            )
            self._reset_preplan()
            return None, float('-inf'), None

        plan_info = (
            dict(self.pending_plan_info)
            if self.pending_plan_info else None
        )
        if plan_info is not None:
            plan_info['gp_change'] = float(change_metrics['change'])
            plan_info['gp_change_mean'] = float(change_metrics['mean'])
            plan_info['gp_change_std'] = float(change_metrics['std'])
            plan_info['gp_change_ucb'] = float(change_metrics['ucb'])
            plan_info['tree_set_changed'] = (
                self.pending_tree_ids != self._current_tree_ids()
            )

        current_score = self._current_ucb_at(self.pending_waypoint)
        if not np.isfinite(current_score):
            current_score = self.pending_score

        if self.pending_tree_ids != self._current_tree_ids():
            self.get_logger().info(
                'Mantendo pré-plano apesar de mudança no conjunto de '
                'árvores: mapa GP ainda compatível '
                f'({change_metrics["change"]:.3f} <= '
                f'{max(change_threshold, 0.0):.3f}).'
            )

        return self.pending_waypoint.copy(), current_score, plan_info

    def _current_tree_ids(self):
        """Return IDs from the latest confirmed tree-map snapshot."""
        return frozenset(
            tree.tree_id for tree in (self.tree_map or ())
        )

    def _reset_preplan(self):
        """Clear all cached preplanning state."""
        self.pending_waypoint = None
        self.pending_score = float('-inf')
        self.pending_for_goal = None
        self.pending_tree_ids = frozenset()
        self.pending_plan_info = None
        self.pending_gp_snapshot = None
        self.preplanned_for_goal = None

    def _init_plot(self):
        """Create a simple interactive matplotlib map."""
        plt.ion()

        xs, ys, gp_mean, _, colorbar_label = self._compute_plot_field()
        extent = grid_extent(xs, ys)

        self.fig, self.ax = plt.subplots(figsize=(7, 6))
        self.gp_image = self.ax.imshow(
            gp_mean,
            extent=extent,
            origin='lower',
            cmap='viridis',
            vmin=0.0,
            vmax=1.0,
            interpolation='nearest',
        )
        self.colorbar = self.fig.colorbar(self.gp_image, ax=self.ax)
        self.colorbar.set_label(colorbar_label)

        self.obs_artist = self.ax.scatter(
            [],
            [],
            c='tab:red',
            s=24,
            label='Observações',
        )
        self.drone_artist = self.ax.scatter(
            [],
            [],
            c='white',
            edgecolors='black',
            marker='*',
            s=150,
            label='Drone',
            zorder=6,
        )
        # Árvores do /tree_map_full: círculos com o raio usado como obstáculo
        # pelo RRT (+ centro marcado, pois o raio é pequeno na escala do mapa).
        self.tree_patches = []
        self.tree_center_artist = self.ax.scatter(
            [],
            [],
            marker='o',
            s=40,
            facecolors='none',
            edgecolors='tab:red',
            linewidths=1.5,
            label='Árvores (obstáculos RRT)',
            zorder=4,
        )
        # Path do RRT (H pontos) enviado por último e o goal atual do Nav2.
        self.path_line, = self.ax.plot(
            [],
            [],
            linestyle='--',
            color='cyan',
            linewidth=1.5,
            label='Path RRT (H pontos)',
            zorder=5,
        )
        self.path_points_artist = self.ax.scatter(
            [],
            [],
            marker='o',
            s=45,
            c='cyan',
            edgecolors='black',
            zorder=5,
        )
        self.path_labels = []
        # Contador de score sempre visível (canto superior esquerdo).
        self.score_text = self.ax.text(
            0.02, 0.98, '',
            transform=self.ax.transAxes,
            va='top', ha='left',
            fontsize=9, color='white', family='monospace',
            bbox=dict(facecolor='black', alpha=0.6, edgecolor='none'),
            zorder=9,
        )
        self.last_plan_score = float('nan')
        self.goal_artist = self.ax.scatter(
            [],
            [],
            marker='X',
            s=120,
            c='magenta',
            edgecolors='black',
            label='Goal atual',
            zorder=7,
        )

        # Eixos fixos exatamente nos map_bounds (o imshow cobre meia célula a mais
        # em cada lado, pois os centros das células caem sobre as bordas).
        x_min, x_max, y_min, y_max = self.bounds
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title(self._plot_title())
        self.ax.legend(loc='upper right')
        self.fig.tight_layout()

        plt.show(block=False)
        self._flush_plot()

    def _update_plot(self, drone_position: np.ndarray):
        """Redraw the GP mean map after each observation."""
        if not plt.fignum_exists(self.fig.number):
            return

        xs, ys, gp_mean, _, colorbar_label = self._compute_plot_field()
        vmax = max(1.0, float(np.nanmax(gp_mean)))

        self.gp_image.set_data(gp_mean)
        self.gp_image.set_extent(grid_extent(xs, ys))
        self.gp_image.set_clim(0.0, vmax)
        self.colorbar.set_label(colorbar_label)
        self.drone_artist.set_offsets(np.atleast_2d(drone_position))

        if self.observed_positions:
            obs_xy = np.vstack(self.observed_positions)
        else:
            obs_xy = np.empty((0, 2))
        self.obs_artist.set_offsets(obs_xy)
        self._draw_trees()
        self._draw_plan_path()
        self._draw_score_counter()

        self.ax.set_title(self._plot_title())
        self._flush_plot()

    def _score_counter_text(self):
        """Texto do contador de score (último, acumulado, obs, árvores, path)."""
        scores = self.observed_scores
        last = scores[-1] if scores else float('nan')
        total = float(np.sum(scores)) if scores else 0.0
        mean = float(np.mean(scores)) if scores else float('nan')
        return (
            f'score atual : {last:6.2f}\n'
            f'acumulado   : {total:6.2f}  (média {mean:.2f})\n'
            f'obs: {len(scores):4d}   árvores: {len(self.tree_map or ()):3d}\n'
            f'path_score RRT: {getattr(self, "last_plan_score", float("nan")):.3f}'
        )

    def _draw_score_counter(self):
        """Atualiza o contador de score (último, acumulado, obs, árvores, path)."""
        self.score_text.set_text(self._score_counter_text())

    def _publish_viz(self, drone_position):
        """Publica em /<uav>/ipp_viz tudo o que o _update_plot desenha."""
        if self.viz_pub is None:
            return
        try:
            xs, ys, gp_mean, _, colorbar_label = self._compute_plot_field()
            mean = np.round(np.asarray(gp_mean, dtype=float), 4)
            goal = getattr(self.drone_controler, 'current_goal', None)
            goal_active = bool(getattr(self.drone_controler, 'goal_active', False))
            state = {
                'stamp': self.get_clock().now().nanoseconds * 1e-9,
                'bounds': list(self.bounds),
                'xs': [float(v) for v in xs],
                'ys': [float(v) for v in ys],
                # NaN vira null (JSON padrão); o viewer converte de volta
                'mean': [[None if not np.isfinite(v) else float(v) for v in row]
                         for row in mean],
                'colorbar_label': colorbar_label,
                'title': self._plot_title(),
                'drone': (None if drone_position is None
                          else [float(drone_position[0]), float(drone_position[1])]),
                'observations': [[float(p[0]), float(p[1])]
                                 for p in self.observed_positions],
                'trees': [[cx, cy, r] for cx, cy, r in self._tree_obstacles()],
                'plan_start': (None if self.last_drone_position is None else
                               [float(self.last_drone_position[0]),
                                float(self.last_drone_position[1])]),
                'plan_path': [[float(p[0]), float(p[1])] for p in self.last_plan_path],
                'goal': ([float(goal[0]), float(goal[1])]
                         if goal is not None and goal_active else None),
                'score_text': self._score_counter_text(),
            }
            self.viz_pub.publish(String(data=json.dumps(state, allow_nan=False)))
        except Exception as exc:  # o plot remoto nunca pode derrubar o planejador
            self.get_logger().warn(
                f'Falha ao publicar ipp_viz: {exc}',
                throttle_duration_sec=5.0,
            )

    def _remember_plan_path(self, plan_info):
        """Guarda os H pontos do path do RRT recém-enviado (para o plot)."""
        path = (plan_info or {}).get('path') or []
        try:
            self.last_plan_score = float((plan_info or {}).get('path_score'))
        except (TypeError, ValueError):
            self.last_plan_score = float('nan')
        points = []
        for xy in path:  # path[0] já é o 1º waypoint (não inclui a origem)
            try:
                point = np.asarray(xy[:2], dtype=float)
            except (IndexError, TypeError, ValueError):
                continue
            if point.shape[0] >= 2 and np.all(np.isfinite(point)):
                points.append(point)
        self.last_plan_path = points

    def _draw_trees(self):
        """Redesenha as árvores do mapa como os círculos de obstáculo do RRT."""
        for patch in self.tree_patches:
            patch.remove()
        self.tree_patches = []
        obstacles = self._tree_obstacles()
        for cx, cy, radius in obstacles:
            patch = Circle(
                (cx, cy),
                max(radius, 0.05),
                facecolor='tab:red',
                edgecolor='darkred',
                alpha=0.45,
                zorder=3,
            )
            self.ax.add_patch(patch)
            self.tree_patches.append(patch)
        centers = (
            np.array([[cx, cy] for cx, cy, _ in obstacles])
            if obstacles else np.empty((0, 2))
        )
        self.tree_center_artist.set_offsets(centers)

    def _draw_plan_path(self):
        """Redesenha o path do RRT (H pontos numerados) e o goal atual."""
        for label in self.path_labels:
            label.remove()
        self.path_labels = []
        points = self.last_plan_path
        if points:
            xy = np.vstack(points)
            start = self.last_drone_position
            if start is not None:
                xy_line = np.vstack([np.asarray(start[:2], dtype=float), xy])
            else:
                xy_line = xy
            self.path_line.set_data(xy_line[:, 0], xy_line[:, 1])
            self.path_points_artist.set_offsets(xy)
            for index, point in enumerate(points, start=1):
                self.path_labels.append(self.ax.annotate(
                    str(index),
                    (float(point[0]), float(point[1])),
                    xytext=(4, 4),
                    textcoords='offset points',
                    color='white',
                    fontsize=8,
                    zorder=8,
                ))
        else:
            self.path_line.set_data([], [])
            self.path_points_artist.set_offsets(np.empty((0, 2)))

        goal = getattr(self.drone_controler, 'current_goal', None)
        if goal is not None and getattr(self.drone_controler, 'goal_active', False):
            self.goal_artist.set_offsets(np.atleast_2d(
                np.asarray(goal[:2], dtype=float)
            ))
        else:
            self.goal_artist.set_offsets(np.empty((0, 2)))

    def _compute_plot_field(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, str]:
        """Compute the configured map source for plotting."""
        if self.map_source == 'gp_predict':
            xs, ys, gp_mean, gp_std = compute_gp_field(
                self.exploration_gp,
                self.bounds,
            )
            return xs, ys, gp_mean, gp_std, 'Média posterior GP predict'

        xs, ys, gp_mean, gp_std = compute_kalman_grid_field(
            self.exploration_gp,
        )
        return xs, ys, gp_mean, gp_std, 'Média posterior Kalman'

    def _plot_title(self):
        """Return the current plot title."""
        source_label = {
            'kalman': 'Kalman grid',
            'gp_predict': 'GP predict contínuo',
        }[self.map_source]
        n_trees = len(self.tree_map or ())
        return (
            f'Mapa GP de exploração ({source_label}) | '
            f'obs={self.exploration_gp.n_observations} | '
            f'árvores={n_trees} | path RRT={len(self.last_plan_path)} pts'
        )

    def _get_configured_map_source(self):
        """Read and validate the configured map source."""
        map_source = getattr(
            config,
            'GAUSSIAN_FEEDER_MAP_SOURCE',
            'kalman',
        ).lower()
        if map_source not in VALID_MAP_SOURCES:
            self.get_logger().warn(
                'GAUSSIAN_FEEDER_MAP_SOURCE inválido: '
                f'{map_source}. Usando kalman.'
            )
            return 'kalman'
        return map_source

    @staticmethod
    def _get_first_available(data: dict, *keys: str):
        """Return the first present value from a list of JSON field names."""
        for key in keys:
            if key in data:
                return data[key]
        raise KeyError(keys[0])

    def _flush_plot(self):
        """Let matplotlib process pending GUI events."""
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def _save_final_outputs(self) -> None:
        """Save observations, GP grid state, and final map figures to disk.

        Uses Figure + FigureCanvasAgg directly so the save is independent
        of the interactive matplotlib state and safe to call during shutdown.
        """
        if not self._output_dir or self._final_outputs_saved:
            return
        self._final_outputs_saved = True
        try:
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            import matplotlib.cm as mcm
            import matplotlib.colors as mcolors

            out_dir = pathlib.Path(self._output_dir) / 'gaussian_feeder'
            out_dir.mkdir(parents=True, exist_ok=True)

            # ── observations.csv ────────────────────────────────────────────
            with open(out_dir / 'observations.csv', 'w', newline='',
                      encoding='utf-8') as fh:
                w = csv.writer(fh)
                w.writerow(['x', 'y', 'point_score'])
                for pos, score in zip(self.observed_positions, self.observed_scores):
                    w.writerow([float(pos[0]), float(pos[1]), float(score)])

            # ── gp_grid.csv ─────────────────────────────────────────────────
            stds = np.sqrt(np.maximum(np.diag(self.exploration_gp.cov_), 0.0))
            ucb_vals = self.exploration_gp.ucb_grid()
            with open(out_dir / 'gp_grid.csv', 'w', newline='',
                      encoding='utf-8') as fh:
                w = csv.writer(fh)
                w.writerow(['x', 'y', 'mean', 'std', 'ucb'])
                for (x, y), mean, std, ucb in zip(
                    self.exploration_gp.X_grid,
                    self.exploration_gp.mean_,
                    stds,
                    ucb_vals,
                ):
                    w.writerow([float(x), float(y), float(mean),
                                float(std), float(ucb)])

            # ── shared GP field ─────────────────────────────────────────────
            xs, ys, gp_mean, _ = compute_kalman_grid_field(self.exploration_gp)
            ext = grid_extent(xs, ys)
            vmax = max(1.0, float(np.nanmax(gp_mean)))
            n_obs = self.exploration_gp.n_observations

            def _make_fig():
                fig = Figure(figsize=(7, 6))
                FigureCanvasAgg(fig)
                return fig

            def _base_map(ax, fig):
                im = ax.imshow(
                    gp_mean, extent=ext, origin='lower',
                    cmap='viridis', interpolation='nearest',
                    vmin=0.0, vmax=vmax,
                )
                cb = fig.colorbar(im, ax=ax)
                cb.set_label('visibility score (GP mean)')
                ax.set_xlabel('X [m]')
                ax.set_ylabel('Y [m]')
                ax.set_aspect('equal')
                return im

            # ── Plot 1: GP mean map only ────────────────────────────────────
            fig1 = _make_fig()
            ax1 = fig1.add_subplot(111)
            _base_map(ax1, fig1)
            ax1.set_title(f'GP posterior mean — {n_obs} observations')
            fig1.tight_layout()
            fig1.savefig(str(out_dir / 'gp_map_mean.png'),
                         dpi=150, bbox_inches='tight')

            # ── Plot 2: GP mean map + trajectory ────────────────────────────
            fig2 = _make_fig()
            ax2 = fig2.add_subplot(111)
            _base_map(ax2, fig2)

            if self.observed_positions:
                obs_xy = np.array(self.observed_positions)
                n = len(obs_xy)
                cmap_t = mcm.get_cmap('plasma')
                norm_t = mcolors.Normalize(vmin=0, vmax=max(n - 1, 1))

                for i in range(n - 1):
                    ax2.plot(
                        obs_xy[i:i + 2, 0], obs_xy[i:i + 2, 1],
                        color=cmap_t(norm_t(i)),
                        linewidth=1.5, alpha=0.85, solid_capstyle='round',
                    )
                ax2.scatter(
                    obs_xy[:, 0], obs_xy[:, 1],
                    c=np.arange(n), cmap='plasma',
                    s=18, zorder=5, linewidths=0,
                )
                ax2.scatter(
                    *obs_xy[0], marker='o', s=90, c='white',
                    edgecolors='black', linewidths=1.5, zorder=6, label='start',
                )
                ax2.scatter(
                    *obs_xy[-1], marker='*', s=130, c='yellow',
                    edgecolors='black', linewidths=0.5, zorder=6, label='end',
                )
                ax2.legend(loc='upper right', fontsize=8)

            ax2.set_title(f'GP posterior mean + trajectory — {n_obs} observations')
            fig2.tight_layout()
            fig2.savefig(str(out_dir / 'gp_map_trajectory.png'),
                         dpi=150, bbox_inches='tight')

            self.get_logger().info(
                f'GP final salvo em: {out_dir} '
                f'({n_obs} obs, {len(self.observed_positions)} posições)'
            )
        except Exception as exc:
            self.get_logger().error(f'Erro ao salvar mapa GP final: {exc}')

    def destroy_node(self):
        """Save GP outputs, then close the live matplotlib figure."""
        self._save_final_outputs()
        if hasattr(self, 'fig') and plt.fignum_exists(self.fig.number):
            plt.close(self.fig)
        self._release_instance_lock()
        return super().destroy_node()


def main():
    """Run the Gaussian feeder node."""
    rclpy.init()
    node = None
    executor = SingleThreadedExecutor()

    def _handle_sigterm(signum, frame):
        executor.shutdown(timeout_sec=0.0)

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        node = GaussianFeeder()
        executor.add_node(node)
        executor.add_node(node.drone_controler)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Save GP outputs FIRST — before any ROS cleanup that may throw when
        # the context is already invalid (rcl_shutdown called by SIGINT handler).
        if node is not None:
            node._save_final_outputs()
        try:
            executor.shutdown()
        except Exception:
            pass
        if node is not None:
            try:
                node.drone_controler.destroy_node()
            except Exception:
                pass
            try:
                if hasattr(node, 'fig') and plt.fignum_exists(node.fig.number):
                    plt.close(node.fig)
                node._release_instance_lock()
                super(GaussianFeeder, node).destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
