#!/usr/bin/env python3
"""ROS 2 benchmark planner that swaps in simple waypoint strategies."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

from informative_planner import config
from informative_planner.benchmark_logging import BenchmarkRunLogger
from informative_planner.benchmark_strategies import build_strategy
from informative_planner.controllers import build_controller
from informative_planner.controllers import declare_controller_parameters
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tree_mapper.msg import ObjectDetectionArray


class BenchmarkPlanner(Node):
    """Send benchmark waypoints through the MRS Octomap Planner."""

    def __init__(self):
        super().__init__('benchmark_planner')

        self.uav_name = str(self.declare_parameter('uav_name', 'limo').value)
        self.strategy_name = str(
            self.declare_parameter('strategy', 'random_waypoint').value
        )
        self.bounds = self._read_bounds()
        self.flight_height = float(
            self.declare_parameter(
                'flight_height',
                config.FLIGHT_HEIGHT,
            ).value
        )
        self.grid_resolution = float(
            self.declare_parameter(
                'waypoint_grid_resolution',
                config.WAYPOINT_GRID_RESOLUTION,
            ).value
        )
        self.min_dist = float(
            self.declare_parameter('min_dist', 1.5).value
        )
        self.d_step = _positive_or_none(
            float(
                self.declare_parameter(
                    'd_step',
                    config.MAX_WAYPOINT_STEP or 0.0,
                ).value
            )
        )
        self.time_budget_sec = float(
            self.declare_parameter(
                'time_budget_sec',
                config.TIME_BUDGET,
            ).value
        )
        self.random_seed = self._read_random_seed()
        self.output_root = str(
            self.declare_parameter(
                'output_dir',
                str(Path.home() / 'ipp_results'),
            ).value
        )
        self.run_name = str(
            self.declare_parameter('run_name', '').value
        ) or self._default_run_name()
        self.world_name = str(
            self.declare_parameter('world_name', '').value
        )
        self.world_file = str(
            self.declare_parameter('world_file', '').value
        )
        self.ground_truth_csv_path = str(
            self.declare_parameter('ground_truth_csv_path', '').value
        )
        self.ground_truth_dap_m = float(
            self.declare_parameter('ground_truth_dap_m', 0.0).value
        )
        self.tick_period_sec = float(
            self.declare_parameter('planner_tick_period_sec', 0.5).value
        )
        self.odom_startup_timeout_sec = float(
            self.declare_parameter('odom_startup_timeout_sec', 60.0).value
        )
        self.odom_stale_timeout_sec = float(
            self.declare_parameter('odom_stale_timeout_sec', 20.0).value
        )
        self.lawnmower_line_spacing = _positive_or_none(
            float(
                self.declare_parameter(
                    'lawnmower_line_spacing',
                    self.d_step or config.WAYPOINT_GRID_RESOLUTION,
                ).value
            )
        )
        self.lawnmower_track_spacing = _positive_or_none(
            float(
                self.declare_parameter(
                    'lawnmower_track_spacing',
                    self.d_step or config.WAYPOINT_GRID_RESOLUTION,
                ).value
            )
        )
        self.lawnmower_margin = float(
            self.declare_parameter('lawnmower_margin', 0.0).value
        )
        self.lawnmower_endpoints_only = bool(
            self.declare_parameter('lawnmower_endpoints_only', False).value
        )
        self.trajectory_sample_period = float(
            self.declare_parameter(
                'trajectory_sample_period_sec',
                0.5,
            ).value
        )
        self.trajectory_sample_distance = float(
            self.declare_parameter(
                'trajectory_sample_min_distance_m',
                0.05,
            ).value
        )

        (
            self.controller_backend,
            self.navigate_action,
            self.odom_topic,
        ) = declare_controller_parameters(self)
        default_tree_topic = '/%s/tree_list' % self.uav_name
        self.tree_list_topic = str(
            self.declare_parameter(
                'tree_list_topic',
                default_tree_topic,
            ).value or default_tree_topic
        )
        self.tree_measuring_map_frame = str(
            self.declare_parameter(
                'tree_measuring_map_frame',
                config.PLANNING_FRAME,
            ).value or config.PLANNING_FRAME
        )
        self.tree_mapper_map_topic = str(
            self.declare_parameter(
                'tree_mapper_map_topic',
                '/tree_map_full',
            ).value or '/tree_map_full'
        )
        default_waypoint_topic = '/%s/planner_waypoint_cmd' % self.uav_name
        self.external_waypoint_topic = str(
            self.declare_parameter(
                'external_waypoint_topic',
                default_waypoint_topic,
            ).value or default_waypoint_topic
        )

        self.strategy = build_strategy(
            strategy_name=self.strategy_name,
            bounds=self.bounds,
            height=self.flight_height,
            resolution=self.grid_resolution,
            min_dist=self.min_dist,
            d_step=self.d_step,
            random_seed=self.random_seed,
            lawnmower_line_spacing=self.lawnmower_line_spacing,
            lawnmower_track_spacing=self.lawnmower_track_spacing,
            lawnmower_margin=self.lawnmower_margin,
            lawnmower_endpoints_only=self.lawnmower_endpoints_only,
        )
        self.drone_controler = (
            build_controller(
                self.controller_backend,
                self.uav_name,
                self.navigate_action,
                self.odom_topic,
            )
            if self.strategy.sends_waypoints
            else None
        )
        self.current_position = None
        self.start_time = None
        self.finished = False
        self._wall_start_time = time.monotonic()
        self._last_odom_wall_time: float | None = None
        self._finish_reason: str | None = None

        self.logger = BenchmarkRunLogger(
            output_root=self.output_root,
            run_name=self.run_name,
            metadata=self._metadata(),
            sample_period_sec=self.trajectory_sample_period,
            min_sample_distance_m=self.trajectory_sample_distance,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
        )
        self.point_quality_sub = self.create_subscription(
            String,
            '/%s/point_quality' % self.uav_name,
            self._point_quality_callback,
            10,
        )
        self.tree_sub = self.create_subscription(
            String,
            self.tree_list_topic,
            self._tree_list_callback,
            10,
        )
        self.tree_mapper_sub = self.create_subscription(
            ObjectDetectionArray,
            self.tree_mapper_map_topic,
            self._tree_mapper_map_callback,
            10,
        )
        self.mission_done_topic = str(
            self.declare_parameter(
                'mission_done_topic',
                config.MISSION_DONE_TOPIC,
            ).value or config.MISSION_DONE_TOPIC
        )
        self.mission_done_pub = self.create_publisher(
            String,
            self.mission_done_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.external_waypoint_sub = None
        if not self.strategy.sends_waypoints:
            self.external_waypoint_sub = self.create_subscription(
                String,
                self.external_waypoint_topic,
                self._external_waypoint_callback,
                10,
            )
        self.timer = self.create_timer(self.tick_period_sec, self._tick)

        self.get_logger().info(
            'Benchmark planner ready: strategy=%s, budget=%.1fs, output=%s'
            % (self.strategy.name, self.time_budget_sec,
               self.logger.output_dir)
        )
        if not self.strategy.sends_waypoints:
            self.get_logger().info(
                'Passive benchmark mode: recording external waypoints from %s'
                % self.external_waypoint_topic
            )

    def _tick(self) -> None:
        if self.finished:
            return

        if self._elapsed_sec() >= self.time_budget_sec:
            self._finish('time_budget_elapsed')
            return

        if self.current_position is None:
            wall_elapsed = time.monotonic() - self._wall_start_time
            if wall_elapsed >= self.odom_startup_timeout_sec:
                self._finish('odom_timeout')
                return
            self.get_logger().info(
                'Waiting for odometry on %s (%.0f / %.0fs).'
                % (self.odom_topic, wall_elapsed, self.odom_startup_timeout_sec),
                throttle_duration_sec=2.0,
            )
            return

        # Odom was available but stopped → estimation_manager likely crashed
        if time.monotonic() - self._last_odom_wall_time > self.odom_stale_timeout_sec:
            self._finish('odom_lost')
            return

        if not self.strategy.sends_waypoints:
            self.get_logger().info(
                'Recording external planner; benchmark will not send goals.',
                throttle_duration_sec=5.0,
            )
            return

        wait_reason = self.drone_controler.waypoint_wait_reason()
        if wait_reason is not None:
            self.get_logger().info(
                'Waiting for %s.' % wait_reason,
                throttle_duration_sec=2.0,
            )
            return

        if self.strategy.is_done():
            self._finish('strategy_complete')
            return

        result = self.strategy.next_waypoint(self.current_position[:2])
        if result is None:
            self._finish('no_feasible_waypoint')
            return

        sent = self.drone_controler.send_waypoint(result.waypoint)
        if not sent:
            return

        self.strategy.on_waypoint_sent(result.waypoint)
        self.logger.record_waypoint(
            time_sec=self._elapsed_sec(),
            waypoint_xyz=result.waypoint,
            strategy_name=self.strategy.name,
            info=result.info,
        )
        self.get_logger().info(
            'Waypoint %d sent: x=%.2f y=%.2f z=%.2f'
            % (
                len(self.logger.waypoint_commands),
                result.waypoint[0],
                result.waypoint[1],
                result.waypoint[2],
            )
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._last_odom_wall_time = time.monotonic()

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info('Benchmark mission timer started.')

        position = msg.pose.pose.position
        current = np.array([position.x, position.y, position.z], dtype=float)
        self.current_position = current
        self.logger.record_odom(self._elapsed_sec(), current)

    def _tree_list_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn('Invalid tree_list JSON: %s' % exc)
            return

        if not isinstance(payload, list):
            self.get_logger().warn('tree_list payload is not a list')
            return

        self.logger.record_tree_list(self._elapsed_sec(), payload)

    def _point_quality_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn('Invalid point_quality JSON: %s' % exc)
            return

        try:
            self.logger.record_point_quality(
                time_sec=self._elapsed_sec(),
                position_x=float(payload['position_x']),
                position_y=float(payload['position_y']),
                point_score=float(payload['point_score']),
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn('Malformed point_quality payload: %s' % exc)

    def _tree_mapper_map_callback(self, msg: ObjectDetectionArray) -> None:
        trees = []
        for detection in msg.detections:
            trees.append({
                'map_id': int(detection.id),
                'x': float(detection.pose.position.x),
                'y': float(detection.pose.position.y),
                'z': float(detection.pose.position.z),
                'radius_m': float(detection.radius),
                'diameter_m': float(detection.diameter),
                'cluster_label': int(detection.cluster_label),
                'cluster_points': int(detection.cluster_points),
                'fit_error': float(detection.fit_error),
                # v2: 'confidence' virou geometric_score; chave mantida.
                'confidence': float(detection.geometric_score),
                'confirmed': True,
            })

        self.logger.record_tree_mapper_map(
            time_sec=self._elapsed_sec(),
            frame_id=msg.header.frame_id,
            trees=trees,
        )

    def _external_waypoint_callback(self, msg: String) -> None:
        if self.strategy.sends_waypoints:
            return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn('Invalid waypoint JSON: %s' % exc)
            return

        try:
            waypoint = np.array([
                float(payload['x']),
                float(payload['y']),
                float(payload['z']),
            ], dtype=float)
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                'External waypoint payload missing x/y/z: %s' % msg.data
            )
            return

        info = {
            key: value
            for key, value in payload.items()
            if key not in ('x', 'y', 'z')
        }
        self.logger.record_waypoint(
            time_sec=self._elapsed_sec(),
            waypoint_xyz=waypoint,
            strategy_name=self.strategy.name,
            info=info,
        )
        self.get_logger().info(
            'External waypoint %d recorded: x=%.2f y=%.2f z=%.2f'
            % (
                len(self.logger.waypoint_commands),
                waypoint[0],
                waypoint[1],
                waypoint[2],
            )
        )

    def _finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self._finish_reason = reason
        # Avisa primeiro (robô para e feeder salva em paralelo), depois grava.
        self._publish_mission_done(reason)
        output_dir = self.logger.finalize(reason)
        self.get_logger().info(
            'Benchmark finished (%s). Artifacts saved in %s'
            % (reason, output_dir)
        )
        if rclpy.ok():
            rclpy.shutdown()

    def _publish_mission_done(self, reason: str) -> None:
        msg = String()
        msg.data = json.dumps({
            'reason': reason,
            'elapsed_sec': self._elapsed_sec(),
            'output_dir': str(self.logger.output_dir),
        })
        self.mission_done_pub.publish(msg)
        try:
            self.mission_done_pub.wait_for_all_acked(Duration(seconds=2.0))
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().info(
            'Mission done (%s) publicado em %s'
            % (reason, self.mission_done_topic)
        )

    def finish_if_needed(self, reason: str = 'shutdown') -> None:
        """Persist artifacts if the process is interrupted."""
        if not self.finished:
            pos = (
                self.current_position.tolist()
                if self.current_position is not None
                else None
            )
            self.logger.monitor.write_crash_report(
                reason,
                context={
                    'waypoints_sent': len(self.logger.waypoint_commands),
                    'trees_detected': len(self.logger.latest_tree_list),
                    'tree_mapper_trees_detected': len(
                        self.logger.latest_tree_mapper_map
                    ),
                    'last_position': pos,
                    'elapsed_sec': self._elapsed_sec(),
                },
            )
            self.logger.finalize(reason)

    def _read_bounds(self) -> tuple[float, float, float, float]:
        value = self.declare_parameter(
            'map_bounds',
            list(config.MAP_BOUNDS),
        ).value
        if len(value) != 4:
            raise ValueError(
                'map_bounds must contain [x_min, x_max, y_min, y_max]'
            )
        return tuple(float(item) for item in value)

    def _read_random_seed(self) -> int | None:
        value = int(self.declare_parameter('random_seed', -1).value)
        return None if value < 0 else value

    def _default_run_name(self) -> str:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        return '%s_%s' % (self.strategy_name, stamp)

    def _metadata(self) -> dict:
        return {
            'strategy': self.strategy.name,
            'world_name': self.world_name,
            'world_file': self.world_file,
            'ground_truth_csv_path': self.ground_truth_csv_path,
            'ground_truth_dap_m': self.ground_truth_dap_m,
            'uav_name': self.uav_name,
            'map_bounds': self.bounds,
            'flight_height_m': self.flight_height,
            'grid_resolution_m': self.grid_resolution,
            'min_dist_m': self.min_dist,
            'd_step_m': self.d_step,
            'time_budget_sec': self.time_budget_sec,
            'random_seed': self.random_seed,
            'lawnmower_line_spacing_m': self.lawnmower_line_spacing,
            'lawnmower_track_spacing_m': self.lawnmower_track_spacing,
            'lawnmower_margin_m': self.lawnmower_margin,
            'odom_topic': self.odom_topic,
            'tree_list_topic': self.tree_list_topic,
            'tree_measuring_map_frame': self.tree_measuring_map_frame,
            'tree_mapper_map_topic': self.tree_mapper_map_topic,
            'external_waypoint_topic': self.external_waypoint_topic,
            'benchmark_sends_waypoints': self.strategy.sends_waypoints,
        }

    def _elapsed_sec(self) -> float:
        if self.start_time is None:
            return 0.0
        elapsed = self.get_clock().now() - self.start_time
        return float(elapsed.nanoseconds) * 1e-9


def _positive_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    value = float(value)
    if value <= 0.0:
        return None
    return value


def main() -> None:
    """Run the benchmark planner and waypoint controller nodes."""
    rclpy.init()
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = BenchmarkPlanner()
        executor.add_node(node)
        if node.drone_controler is not None:
            executor.add_node(node.drone_controler)
        executor.spin()
    finally:
        executor.shutdown()
        if node is not None:
            node.finish_if_needed()
            if node.drone_controler is not None:
                node.drone_controler.destroy_node()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # Exit with non-zero code so the batch script can detect failed runs.
    _error_reasons = ('odom_timeout', 'odom_lost')
    if node is not None and node._finish_reason in _error_reasons:
        sys.exit(2)


if __name__ == '__main__':
    main()
