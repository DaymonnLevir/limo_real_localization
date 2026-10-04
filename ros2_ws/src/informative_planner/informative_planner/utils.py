"""Utility helpers for waypoint generation."""

from informative_planner import config

import numpy as np


def generate_waypoint_candidates(
    forest_bounds: tuple[float, float, float, float],
    resolution: float = None,
    height: float = None,
) -> np.ndarray:
    """Gera uma grade regular de waypoints candidatos."""
    if resolution is None:
        resolution = config.WAYPOINT_GRID_RESOLUTION
    if height is None:
        height = config.FLIGHT_HEIGHT

    if resolution <= 0.0:
        raise ValueError('resolution must be positive')

    x_min, x_max, y_min, y_max = forest_bounds

    # Grade que cobre EXATAMENTE os bounds do mapa, incluindo a borda superior.
    # np.arange(x_min, x_max, res) parava antes de x_max quando o vão não era
    # múltiplo de `resolution` (ex.: [-16, 16] com res=3 ia só até 14), fazendo
    # o grid do planner ficar menor que o mapa real. linspace com os dois
    # extremos inclusos corrige isso; a resolução efetiva se ajusta ao vão.
    def _axis(lo, hi):
        span = float(hi) - float(lo)
        n_intervals = max(1, int(round(span / resolution)))
        return np.linspace(float(lo), float(hi), n_intervals + 1)

    xs = _axis(x_min, x_max)
    ys = _axis(y_min, y_max)
    candidates = []
    for x in xs:
        for y in ys:
            candidates.append([x, y, height])
    return np.array(candidates)


def _nearest_waypoint(position, candidates):
    shortest_distance = None
    nearest_candidate = None
    idx_nearest_candidate = None
    for idx, candidate in enumerate(candidates):
        dist = float(
            np.linalg.norm(np.array(candidate) - np.array(position))
        )
        if shortest_distance is None or dist < shortest_distance:
            nearest_candidate = candidate
            shortest_distance = dist
            idx_nearest_candidate = idx
    return nearest_candidate, shortest_distance, idx_nearest_candidate


def _get_first_available(data: dict, *keys: str):
    """Return the first present value from a list of JSON field names."""
    for key in keys:
        if key in data:
            return data[key]
    raise KeyError(keys[0])
