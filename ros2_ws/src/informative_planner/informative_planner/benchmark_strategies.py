"""Waypoint strategies used by the Gazebo benchmark planner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from informative_planner import config
from informative_planner.utils import generate_waypoint_candidates


@dataclass(frozen=True)
class StrategyResult:
    """Waypoint selected by a benchmark strategy."""

    waypoint: np.ndarray
    info: dict[str, Any]


class WaypointStrategy:
    """Base interface for benchmark waypoint strategies."""

    name = 'base'
    sends_waypoints = True

    def next_waypoint(self, current_xy: np.ndarray) -> StrategyResult | None:
        """Return the next waypoint to send to MRS."""
        raise NotImplementedError

    def on_waypoint_sent(self, waypoint: np.ndarray) -> None:
        """Update internal state after a waypoint command is accepted."""

    def is_done(self) -> bool:
        """Return True when the strategy has no more work."""
        return False


class RandomWaypointStrategy(WaypointStrategy):
    """Choose a random feasible candidate within d_step of the UAV."""

    name = 'random_waypoint'

    def __init__(
        self,
        bounds: tuple[float, float, float, float],
        height: float = config.FLIGHT_HEIGHT,
        resolution: float = config.WAYPOINT_GRID_RESOLUTION,
        min_dist: float = 1.5,
        d_step: float | None = config.MAX_WAYPOINT_STEP,
        seed: int | None = None,
    ):
        self.bounds = tuple(float(value) for value in bounds)
        self.height = float(height)
        self.min_dist = float(min_dist)
        self.d_step = _positive_or_none(d_step)
        self.rng = np.random.default_rng(seed)
        self.candidates = generate_waypoint_candidates(
            self.bounds,
            resolution=float(resolution),
            height=self.height,
        )

    def next_waypoint(self, current_xy: np.ndarray) -> StrategyResult | None:
        current_xy = np.asarray(current_xy[:2], dtype=float)
        cand_xy = self.candidates[:, :2]
        dists = np.linalg.norm(cand_xy - current_xy, axis=1)
        feasible = np.isfinite(dists)

        if self.min_dist > 0.0:
            feasible &= dists >= self.min_dist
        if self.d_step is not None:
            feasible &= dists <= self.d_step

        indices = np.flatnonzero(feasible)
        if indices.size == 0:
            return None

        chosen_idx = int(self.rng.choice(indices))
        waypoint = self.candidates[chosen_idx].astype(float, copy=True)
        return StrategyResult(
            waypoint=waypoint,
            info={
                'candidate_index': chosen_idx,
                'candidate_count': int(indices.size),
                'distance_from_current_m': float(dists[chosen_idx]),
                'd_step_m': self.d_step,
            },
        )


class LawnmowerStrategy(WaypointStrategy):
    """Walk a boustrophedon grid over the map bounds."""

    name = 'lawnmower'

    def __init__(
        self,
        bounds: tuple[float, float, float, float],
        height: float = config.FLIGHT_HEIGHT,
        line_spacing: float | None = config.MAX_WAYPOINT_STEP,
        track_spacing: float | None = config.MAX_WAYPOINT_STEP,
        margin: float = 0.0,
        endpoints_only: bool = False,
    ):
        self.bounds = _shrink_bounds(bounds, margin)
        self.height = float(height)
        self.line_spacing = float(
            line_spacing or config.WAYPOINT_GRID_RESOLUTION
        )
        self.track_spacing = float(
            track_spacing or config.WAYPOINT_GRID_RESOLUTION
        )
        self.endpoints_only = bool(endpoints_only)
        self.route = self._build_route()
        self.index = 0

    def next_waypoint(self, current_xy: np.ndarray) -> StrategyResult | None:
        del current_xy
        if self.index >= len(self.route):
            return None

        waypoint = self.route[self.index].astype(float, copy=True)
        return StrategyResult(
            waypoint=waypoint,
            info={
                'route_index': self.index,
                'route_length': len(self.route),
                'line_spacing_m': self.line_spacing,
                'track_spacing_m': self.track_spacing,
            },
        )

    def on_waypoint_sent(self, waypoint: np.ndarray) -> None:
        del waypoint
        self.index += 1

    def is_done(self) -> bool:
        return self.index >= len(self.route)

    def _build_route(self) -> list[np.ndarray]:
        x_min, x_max, y_min, y_max = self.bounds
        ys = _axis_values(y_min, y_max, self.line_spacing)

        # Always send intermediate waypoints along each lawnmower row.
        # This makes the coverage more restrictive because MRS receives
        # explicit waypoints between the two row endpoints instead of planning
        # the whole segment autonomously.
        xs = _axis_values(x_min, x_max, self.track_spacing)
        xs_fwd = xs
        xs_bwd = list(reversed(xs))

        route = []
        for row_idx, y in enumerate(ys):
            row_xs = xs_fwd if row_idx % 2 == 0 else xs_bwd
            for x in row_xs:
                route.append(np.array([x, y, self.height], dtype=float))
        return route


class ExternalPlannerStrategy(WaypointStrategy):
    """Record a planner running in another node without sending waypoints."""

    name = 'informative'
    sends_waypoints = False

    def next_waypoint(self, current_xy: np.ndarray) -> StrategyResult | None:
        del current_xy
        return None


def build_strategy(
    strategy_name: str,
    bounds: tuple[float, float, float, float],
    height: float,
    resolution: float,
    min_dist: float,
    d_step: float | None,
    random_seed: int | None,
    lawnmower_line_spacing: float | None,
    lawnmower_track_spacing: float | None,
    lawnmower_margin: float,
    lawnmower_endpoints_only: bool = False,
) -> WaypointStrategy:
    """Create one benchmark strategy from ROS parameters."""
    normalized = strategy_name.strip().lower().replace('-', '_')
    if normalized in ('random', 'random_waypoint'):
        return RandomWaypointStrategy(
            bounds=bounds,
            height=height,
            resolution=resolution,
            min_dist=min_dist,
            d_step=d_step,
            seed=random_seed,
        )
    if normalized in ('lawnmower', 'coverage'):
        return LawnmowerStrategy(
            bounds=bounds,
            height=height,
            line_spacing=lawnmower_line_spacing or d_step,
            track_spacing=lawnmower_track_spacing or d_step,
            margin=lawnmower_margin,
            endpoints_only=lawnmower_endpoints_only,
        )
    if normalized in ('external', 'passive', 'informative', 'gaussian_feeder'):
        return ExternalPlannerStrategy()
    raise ValueError(
        'Unknown benchmark strategy "%s". Use random_waypoint, lawnmower, '
        'or informative.'
        % strategy_name
    )


def _axis_values(start: float, stop: float, spacing: float) -> list[float]:
    if spacing <= 0.0:
        raise ValueError('Lawnmower spacing must be positive.')

    values = list(np.arange(start, stop + 0.5 * spacing, spacing))
    if not values or abs(values[-1] - stop) > 1e-6:
        values.append(float(stop))
    return [float(value) for value in values]


def _positive_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    value = float(value)
    if value <= 0.0:
        return None
    return value


def _shrink_bounds(
    bounds: tuple[float, float, float, float],
    margin: float,
) -> tuple[float, float, float, float]:
    x_min, x_max, y_min, y_max = (float(value) for value in bounds)
    margin = max(float(margin), 0.0)

    if 2.0 * margin >= min(x_max - x_min, y_max - y_min):
        return x_min, x_max, y_min, y_max
    return (
        x_min + margin,
        x_max - margin,
        y_min + margin,
        y_max - margin,
    )
