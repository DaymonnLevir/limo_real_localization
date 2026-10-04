"""Run logging helpers for planner benchmark experiments."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from .benchmark_monitor import SystemMonitor


class BenchmarkRunLogger:
    """Collect and persist trajectory, waypoint, and tree-map artifacts."""

    def __init__(
        self,
        output_root: str,
        run_name: str,
        metadata: dict[str, Any],
        sample_period_sec: float = 0.5,
        min_sample_distance_m: float = 0.05,
    ):
        self.run_name = _safe_name(run_name)
        self.output_dir = Path(output_root).expanduser() / self.run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.metadata = dict(metadata)
        self.metadata['created_at_utc'] = datetime.now(
            timezone.utc
        ).isoformat()
        self.sample_period_sec = max(float(sample_period_sec), 0.0)
        self.min_sample_distance_m = max(float(min_sample_distance_m), 0.0)

        self.trajectory_samples: list[dict[str, float]] = []
        self.trajectory_raw: list[dict[str, float]] = []
        self.waypoint_commands: list[dict[str, Any]] = []
        self.tree_snapshots: list[dict[str, Any]] = []  # kept for summary count
        self.latest_tree_list: list[dict[str, Any]] = []
        self.latest_tree_mapper_map: list[dict[str, Any]] = []
        self.tree_mapper_frame_id = ''
        self.tree_mapper_snapshot_count = 0

        # Snapshots são escritos linha-a-linha para não perder dados em crashes.
        self._snapshots_file = (
            self.output_dir / 'tree_snapshots.jsonl'
        ).open('w', encoding='utf-8')
        self.tree_maps_dir = self.output_dir / 'tree_maps'
        self.tree_maps_dir.mkdir(parents=True, exist_ok=True)
        self._tree_mapper_snapshots_file = (
            self.tree_maps_dir / 'tree_mapper_snapshots.jsonl'
        ).open('w', encoding='utf-8')

        self.monitor = SystemMonitor(self.output_dir, sample_interval_sec=1.0)
        self.monitor.start()

        self.distance_xy_m = 0.0
        self.distance_3d_m = 0.0
        self.finished = False

        self._last_distance_position: np.ndarray | None = None
        self._last_sample_position: np.ndarray | None = None
        self._last_sample_time_sec: float | None = None

        self.point_quality_samples: list[dict[str, float]] = []

    def record_odom(self, time_sec: float, position_xyz: np.ndarray) -> None:
        """Record odometry and update traveled distance."""
        position = np.asarray(position_xyz[:3], dtype=float)
        if position.shape[0] != 3 or not np.all(np.isfinite(position)):
            return

        if self._last_distance_position is not None:
            delta = position - self._last_distance_position
            self.distance_3d_m += float(np.linalg.norm(delta))
            self.distance_xy_m += float(np.linalg.norm(delta[:2]))
        self._last_distance_position = position.copy()

        self.trajectory_raw.append({
            'time_sec': float(time_sec),
            'x': float(position[0]),
            'y': float(position[1]),
            'z': float(position[2]),
        })

        if not self._should_store_sample(time_sec, position):
            return

        self.trajectory_samples.append({
            'time_sec': float(time_sec),
            'x': float(position[0]),
            'y': float(position[1]),
            'z': float(position[2]),
        })
        self._last_sample_position = position.copy()
        self._last_sample_time_sec = float(time_sec)

    def record_waypoint(
        self,
        time_sec: float,
        waypoint_xyz: np.ndarray,
        strategy_name: str,
        info: dict[str, Any] | None = None,
    ) -> None:
        """Record one waypoint command accepted by the planner node."""
        waypoint = np.asarray(waypoint_xyz[:3], dtype=float)
        payload = {
            'time_sec': float(time_sec),
            'strategy': strategy_name,
            'x': float(waypoint[0]),
            'y': float(waypoint[1]),
            'z': float(waypoint[2]),
            'info': info or {},
        }
        self.waypoint_commands.append(payload)

    def record_tree_list(
        self,
        time_sec: float,
        trees: list[dict[str, Any]],
    ) -> None:
        """Record a tree_node map snapshot from /tree_list."""
        cleaned = [_json_safe_tree(item) for item in trees]
        self.latest_tree_list = cleaned
        snapshot = {
            'time_sec': float(time_sec),
            'tree_count': len(cleaned),
            'trees': cleaned,
        }
        self.tree_snapshots.append(snapshot)
        self._snapshots_file.write(json.dumps(snapshot, sort_keys=True) + '\n')
        self._snapshots_file.flush()

    def record_point_quality(
        self,
        time_sec: float,
        position_x: float,
        position_y: float,
        point_score: float,
    ) -> None:
        """Record a point-quality observation from /point_quality."""
        self.point_quality_samples.append({
            'time_sec': float(time_sec),
            'x': float(position_x),
            'y': float(position_y),
            'point_score': float(point_score),
        })

    def record_tree_mapper_map(
        self,
        time_sec: float,
        frame_id: str,
        trees: list[dict[str, Any]],
    ) -> None:
        """Record a confirmed tree_mapper map snapshot from /tree_map_full."""
        cleaned = [_json_safe_tree(item) for item in trees]
        self.latest_tree_mapper_map = cleaned
        self.tree_mapper_frame_id = str(frame_id or '')
        self.tree_mapper_snapshot_count += 1
        snapshot = {
            'time_sec': float(time_sec),
            'source_method': 'tree_mapper/tree_map_fuser',
            'topic': self.metadata.get(
                'tree_mapper_map_topic',
                '/tree_map_full',
            ),
            'frame_id': self.tree_mapper_frame_id,
            'tree_count': len(cleaned),
            'trees': cleaned,
        }
        self._tree_mapper_snapshots_file.write(
            json.dumps(snapshot, sort_keys=True) + '\n'
        )
        self._tree_mapper_snapshots_file.flush()

    def finalize(self, reason: str) -> Path:
        """Write all benchmark artifacts and return the output directory."""
        if self.finished:
            return self.output_dir
        self.finished = True

        map_sources = self._tree_map_sources()
        summary = {
            'metadata': self.metadata,
            'finish_reason': reason,
            'distance_xy_m': self.distance_xy_m,
            'distance_3d_m': self.distance_3d_m,
            'run_duration_sec': self._run_duration_sec(),
            'trajectory_raw_count': len(self.trajectory_raw),
            'trajectory_sample_count': len(self.trajectory_samples),
            'waypoint_count': len(self.waypoint_commands),
            'tree_count_final': len(self.latest_tree_list),
            'tree_count_final_by_method': {
                'tree_measuring': len(self.latest_tree_list),
                'tree_mapper': len(self.latest_tree_mapper_map),
            },
            'tree_maps': {
                'manifest': 'tree_maps/manifest.json',
                'compatibility_alias': {
                    'tree_map_final.json': 'tree_measuring',
                    'tree_snapshots.jsonl': 'tree_measuring',
                },
                'methods': map_sources,
            },
            'files': {
                'trajectory_raw_csv': 'trajectory_raw.csv',
                'trajectory_csv': 'trajectory.csv',
                'waypoints_csv': 'waypoints.csv',
                'tree_map_json': 'tree_map_final.json',
                'tree_snapshots_jsonl': 'tree_snapshots.jsonl',
                'tree_maps_manifest_json': 'tree_maps/manifest.json',
                'tree_measuring_map_json': (
                    'tree_maps/tree_measuring_map_final.json'
                ),
                'tree_mapper_map_json': (
                    'tree_maps/tree_mapper_map_final.json'
                ),
                'tree_mapper_snapshots_jsonl': (
                    'tree_maps/tree_mapper_snapshots.jsonl'
                ),
                'tree_mapper_native_dir': 'tree_maps/tree_mapper_native',
                'trajectory_tree_map_png': 'trajectory_tree_map.png',
                'point_quality_csv': 'point_quality.csv',
            },
        }
        self._snapshots_file.close()
        self._tree_mapper_snapshots_file.close()
        monitor_summary = self.monitor.stop()
        summary['system_monitor'] = monitor_summary
        summary['files']['system_monitor_csv'] = 'system_monitor.csv'
        self._write_json('run_summary.json', summary)
        self._write_json('tree_map_final.json', self.latest_tree_list)
        self._write_json(
            'tree_maps/tree_measuring_map_final.json',
            {
                'source': map_sources['tree_measuring'],
                'tree_count': len(self.latest_tree_list),
                'trees': self.latest_tree_list,
            },
        )
        self._write_json(
            'tree_maps/tree_mapper_map_final.json',
            {
                'source': map_sources['tree_mapper'],
                'frame_id': self.tree_mapper_frame_id,
                'tree_count': len(self.latest_tree_mapper_map),
                'trees': self.latest_tree_mapper_map,
            },
        )
        self._write_json(
            'tree_maps/manifest.json',
            {
                'schema_version': 1,
                'benchmark_run': self.run_name,
                'benchmark_strategy': self.metadata.get('strategy'),
                'benchmark_world': self.metadata.get('world_name'),
                'ground_truth_csv_path': self.metadata.get(
                    'ground_truth_csv_path'
                ),
                'ground_truth_dap_m': self.metadata.get(
                    'ground_truth_dap_m'
                ),
                'maps': map_sources,
                'compatibility_alias': {
                    'tree_map_final.json': 'tree_measuring',
                    'tree_snapshots.jsonl': 'tree_measuring',
                },
            },
        )
        # tree_snapshots.jsonl já foi escrito incrementalmente por record_tree_list
        self._write_csv(
            'trajectory_raw.csv',
            ('time_sec', 'x', 'y', 'z'),
            self.trajectory_raw,
        )
        self._write_csv(
            'trajectory.csv',
            ('time_sec', 'x', 'y', 'z'),
            self.trajectory_samples,
        )
        self._write_csv(
            'waypoints.csv',
            ('time_sec', 'strategy', 'x', 'y', 'z', 'info'),
            self._waypoint_rows(),
        )
        self._write_csv(
            'point_quality.csv',
            ('time_sec', 'x', 'y', 'point_score'),
            self.point_quality_samples,
        )
        # Figura por último: todos os dados já estão em disco se ela falhar.
        try:
            self._write_trajectory_tree_map('trajectory_tree_map.png')
        except Exception as exc:  # noqa: BLE001
            print(f'[benchmark_logging] trajectory_tree_map falhou: {exc}')
        return self.output_dir

    def _tree_map_sources(self) -> dict[str, dict[str, Any]]:
        strategy = self.metadata.get('strategy')
        return {
            'tree_measuring': {
                'mapping_method': 'tree_measuring/tree_node',
                'benchmark_run': self.run_name,
                'benchmark_strategy': strategy,
                'topic': self.metadata.get('tree_list_topic'),
                'message_type': 'std_msgs/msg/String (JSON list)',
                'frame_id': self.metadata.get('tree_measuring_map_frame'),
                'frame_note': (
                    'Configured frame; the source message has no ROS header.'
                ),
                'received': bool(self.tree_snapshots),
                'snapshot_count': len(self.tree_snapshots),
                'tree_count_final': len(self.latest_tree_list),
                'final_map': 'tree_maps/tree_measuring_map_final.json',
                'snapshots': 'tree_snapshots.jsonl',
            },
            'tree_mapper': {
                'mapping_method': 'tree_mapper/tree_map_fuser',
                'benchmark_run': self.run_name,
                'benchmark_strategy': strategy,
                'topic': self.metadata.get('tree_mapper_map_topic'),
                'message_type': 'tree_mapper/msg/ObjectDetectionArray',
                'frame_id': self.tree_mapper_frame_id or None,
                'received': self.tree_mapper_snapshot_count > 0,
                'snapshot_count': self.tree_mapper_snapshot_count,
                'tree_count_final': len(self.latest_tree_mapper_map),
                'final_map': 'tree_maps/tree_mapper_map_final.json',
                'snapshots': 'tree_maps/tree_mapper_snapshots.jsonl',
                'native_artifacts_dir': 'tree_maps/tree_mapper_native',
            },
        }

    def _run_duration_sec(self) -> float:
        if self.trajectory_raw:
            return float(self.trajectory_raw[-1]['time_sec'])
        if self.trajectory_samples:
            return float(self.trajectory_samples[-1]['time_sec'])
        return 0.0

    def _should_store_sample(
        self,
        time_sec: float,
        position: np.ndarray,
    ) -> bool:
        if self._last_sample_position is None:
            return True
        if self._last_sample_time_sec is None:
            return True

        dt = float(time_sec) - self._last_sample_time_sec
        if dt < self.sample_period_sec:
            return False

        moved = float(np.linalg.norm(position - self._last_sample_position))
        return moved >= self.min_sample_distance_m

    def _waypoint_rows(self) -> list[dict[str, Any]]:
        rows = []
        for waypoint in self.waypoint_commands:
            row = dict(waypoint)
            row['info'] = json.dumps(row.get('info', {}), sort_keys=True)
            rows.append(row)
        return rows

    def _write_json(self, filename: str, payload: Any) -> None:
        path = self.output_dir / filename
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True),
            encoding='utf-8',
        )

    def _write_jsonl(self, filename: str, rows: list[dict[str, Any]]) -> None:
        path = self.output_dir / filename
        lines = [json.dumps(row, sort_keys=True) for row in rows]
        path.write_text('\n'.join(lines) + ('\n' if lines else ''),
                        encoding='utf-8')

    def _write_csv(
        self,
        filename: str,
        fieldnames: tuple[str, ...],
        rows: list[dict[str, Any]],
    ) -> None:
        path = self.output_dir / filename
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({
                    name: row.get(name, '')
                    for name in fieldnames
                })

    def _write_trajectory_tree_map(self, filename: str) -> None:
        """Save publication-quality trajectory + tree map (PNG + PDF)."""
        import os

        mpl_config_dir = self.output_dir / '.matplotlib'
        mpl_config_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('MPLCONFIGDIR', str(mpl_config_dir))

        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        import matplotlib.cm as cm
        import matplotlib.colors as mcolors
        import matplotlib.patheffects as path_effects
        from matplotlib.patches import Circle
        from matplotlib import rcParams

        # ── paper-style rcParams ──────────────────────────────────────────
        rcParams.update({
            'font.family':       'DejaVu Sans',
            'font.size':         8.5,
            'axes.labelsize':    9.5,
            'axes.titlesize':    10,
            'legend.fontsize':   7.8,
            'xtick.labelsize':   8,
            'ytick.labelsize':   8,
            'axes.linewidth':    0.6,
            'grid.linewidth':    0.4,
            'lines.linewidth':   1.0,
            'patch.linewidth':   0.6,
            'xtick.direction':   'in',
            'ytick.direction':   'in',
            'xtick.major.size':  3,
            'ytick.major.size':  3,
            'legend.framealpha': 0.9,
            'legend.edgecolor':  '0.7',
            'figure.dpi':        300,
            'savefig.dpi':       300,
            'pdf.fonttype':      42,   # embeds fonts in PDF (required by IEEE)
            'ps.fonttype':       42,
        })

        # ── figure: enough room for external legend/colorbar ─────────────
        FIG_W, FIG_H = 6.8, 5.8
        fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('$x$ (m)')
        ax.set_ylabel('$y$ (m)')
        ax.set_facecolor('#f8faf7')
        ax.grid(True, linestyle=':', linewidth=0.45, color='0.78', zorder=0)
        ax.set_axisbelow(True)

        # map-bounds rectangle
        bounds = self.metadata.get('map_bounds')
        if bounds and len(bounds) == 4:
            xmin, xmax, ymin, ymax = (float(v) for v in bounds)
            rect = mpatches.Rectangle(
                (xmin, ymin), xmax - xmin, ymax - ymin,
                linewidth=0.8, edgecolor='#6b7280',
                facecolor='#dce9d5', alpha=0.22,
                linestyle='--', label='Map bounds', zorder=1,
            )
            ax.add_patch(rect)

        # ── trajectory ────────────────────────────────────────────────────
        trajectory_rows = self.trajectory_raw or self.trajectory_samples
        trajectory = None
        if trajectory_rows:
            trajectory = np.array(
                [[s['x'], s['y']] for s in trajectory_rows]
            )
            ax.plot(trajectory[:, 0], trajectory[:, 1],
                    color='#265d9c', linewidth=1.25,
                    label='UAV trajectory', zorder=3)
            ax.scatter(*trajectory[0], marker='s', s=22, color='0.1',
                       edgecolors='white', linewidths=0.5,
                       label='Start', zorder=6)
            ax.scatter(*trajectory[-1], marker='D', s=22, color='#d97706',
                       edgecolors='white', linewidths=0.5,
                       label='End', zorder=6)

        # ── waypoints ─────────────────────────────────────────────────────
        if self.waypoint_commands:
            wps = np.array(
                [[w['x'], w['y']] for w in self.waypoint_commands]
            )
            ax.scatter(wps[:, 0], wps[:, 1], marker='+', s=20,
                       color='#7a7f87', linewidths=0.75, alpha=0.72,
                       label=f'Goal ($n$={len(wps)})', zorder=4)

        self._set_plot_limits(ax)

        # ── detected trees ────────────────────────────────────────────────
        tree_cmap = plt.get_cmap('RdYlGn')
        tree_norm = mcolors.Normalize(vmin=0.0, vmax=1.0)
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        cx_plot = 0.5 * (xlim[0] + xlim[1])
        cy_plot = 0.5 * (ylim[0] + ylim[1])
        for idx, tree in enumerate(self.latest_tree_list, start=1):
            tx = tree.get('x')
            ty = tree.get('y')
            if tx is None or ty is None:
                continue
            diam = tree.get('dbh') or tree.get('diameter')
            radius = max(0.5 * float(diam), 0.10) if diam is not None else 0.35
            cover_error = tree.get('cover_error')
            cover_quality = (
                0.0 if cover_error is None
                else 1.0 - min(max(float(cover_error), 0.0), 1.0)
            )
            tree_color = tree_cmap(tree_norm(cover_quality))
            tx = float(tx)
            ty = float(ty)
            ax.add_patch(Circle(
                (tx, ty), radius,
                facecolor=mcolors.to_rgba(tree_color, 0.24),
                edgecolor=mcolors.to_rgba(tree_color, 0.98),
                linewidth=1.15,
                label='Detected tree (DBH)' if idx == 1 else None,
                zorder=5,
            ))
            ax.scatter(tx, ty, marker='o', s=8, color='#1f2937',
                       linewidths=0, zorder=7)

            label_angle = math.atan2(ty - cy_plot, tx - cx_plot)
            if abs(tx - cx_plot) < 1e-6 and abs(ty - cy_plot) < 1e-6:
                label_angle = idx * math.pi * (3.0 - math.sqrt(5.0))
            label_radius = radius + 0.30
            label_x = tx + label_radius * math.cos(label_angle)
            label_y = ty + label_radius * math.sin(label_angle)
            label = ax.text(label_x, label_y, str(idx),
                            fontsize=6.0, ha='center', va='center',
                            color='#111827', zorder=9)
            label.set_path_effects([
                path_effects.Stroke(linewidth=2.0, foreground='white'),
                path_effects.Normal(),
            ])
            ax.plot([tx, label_x], [ty, label_y],
                    color=tree_color, linewidth=0.45, alpha=0.7, zorder=6)

        if self.latest_tree_list:
            sm = cm.ScalarMappable(norm=tree_norm, cmap=tree_cmap)
            sm.set_array([])
            cbar = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.025)
            cbar.set_label('Tree cover quality')
            cbar.ax.tick_params(labelsize=7.5)

        # ── scale bar (bottom-left) ───────────────────────────────────────
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        span = xlim[1] - xlim[0]
        bar_len = 10.0 if span > 30 else 5.0
        bx = xlim[0] + 0.05 * span
        by = ylim[0] + 0.04 * (ylim[1] - ylim[0])
        ax.plot([bx, bx + bar_len], [by, by],
                color='#111827', linewidth=1.7, solid_capstyle='butt', zorder=8)
        ax.text(bx + bar_len / 2, by + 0.015 * (ylim[1] - ylim[0]),
                f'{bar_len:.0f} m', ha='center', va='bottom',
                fontsize=7.0, color='#111827', zorder=8)

        # ── title and run summary outside the data area ───────────────────
        ax.set_title('Trajectory and Detected Tree Map', pad=8)
        info = (
            f'$d_{{xy}}$ = {self.distance_xy_m:.1f} m\n'
            f'Trees = {len(self.latest_tree_list)}\n'
            f'$T$ = {self._run_duration_sec():.0f} s'
        )
        fig.text(0.985, 0.965, info, fontsize=8.0, va='top', ha='right',
                 bbox=dict(boxstyle='round,pad=0.35', facecolor='white',
                           edgecolor='0.75', linewidth=0.6, alpha=0.96))

        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.095),
                  ncol=5, fontsize=7.6, frameon=False,
                  handlelength=1.6, handletextpad=0.45,
                  borderpad=0.2, labelspacing=0.35, columnspacing=0.9)

        fig.tight_layout(rect=(0.0, 0.055, 0.93, 0.98), pad=0.6)

        png_path = self.output_dir / filename
        pdf_path = self.output_dir / filename.replace('.png', '.pdf')
        fig.savefig(png_path, dpi=300, bbox_inches='tight')
        fig.savefig(pdf_path, bbox_inches='tight')
        plt.close(fig)

        # Restore rcParams defaults so other code is unaffected
        matplotlib.rcdefaults()

    def _set_plot_limits(self, ax) -> None:
        points = []
        points.extend(
            (sample['x'], sample['y'])
            for sample in (self.trajectory_raw or self.trajectory_samples)
        )
        points.extend(
            (waypoint['x'], waypoint['y'])
            for waypoint in self.waypoint_commands
        )
        for tree in self.latest_tree_list:
            tx = tree.get('x')
            ty = tree.get('y')
            if tx is None or ty is None:
                continue
            diam = tree.get('dbh') or tree.get('diameter')
            radius = max(0.5 * float(diam), 0.10) if diam is not None else 0.35
            tx = float(tx)
            ty = float(ty)
            points.extend((
                (tx - radius, ty - radius),
                (tx + radius, ty + radius),
            ))
        bounds = self.metadata.get('map_bounds')
        if bounds and len(bounds) == 4:
            x_min, x_max, y_min, y_max = (float(value) for value in bounds)
            points.extend((
                (x_min, y_min),
                (x_min, y_max),
                (x_max, y_min),
                (x_max, y_max),
            ))
        if not points:
            ax.set_xlim(-5.0, 5.0)
            ax.set_ylim(-5.0, 5.0)
            return

        arr = np.asarray(points, dtype=float)
        x_min, y_min = np.min(arr, axis=0)
        x_max, y_max = np.max(arr, axis=0)
        span_x = max(float(x_max - x_min), 1.0)
        span_y = max(float(y_max - y_min), 1.0)
        margin = max(0.06 * max(span_x, span_y), 3.0)
        x_min -= margin
        x_max += margin
        y_min -= margin
        y_max += margin

        span = max(float(x_max - x_min), float(y_max - y_min), 1.0)
        cx = 0.5 * (x_min + x_max)
        cy = 0.5 * (y_min + y_max)
        half = 0.5 * span
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)


def _json_safe_tree(item: dict[str, Any]) -> dict[str, Any]:
    cleaned = {}
    for key, value in item.items():
        if isinstance(value, bool):
            cleaned[str(key)] = value
        elif isinstance(value, (int, float)):
            v = float(value)
            # nan/inf não são válidos em JSON — converte para null em vez de
            # descartar a chave, preservando a estrutura para análise posterior.
            cleaned[str(key)] = v if math.isfinite(v) else None
        elif isinstance(value, str):
            cleaned[str(key)] = value
        elif value is None:
            cleaned[str(key)] = None
    return cleaned


def _safe_name(value: str) -> str:
    safe = ''.join(
        char if char.isalnum() or char in ('-', '_', '.') else '_'
        for char in value.strip()
    )
    return safe or 'benchmark_run'
