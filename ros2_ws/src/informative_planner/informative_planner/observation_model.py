"""
observation_model.py
--------------------
Modelos modulares para montar o vetor de observação H do GP.

O filtro de Kalman do ExplorationGP usa uma observação escalar:

    z = H f + ruido

onde f é o campo discreto mantido na grade de waypoints.  O modelo antigo
era H = e_i: a medição atualiza só a célula mais próxima e propaga para as
vizinhas apenas via covariância do kernel.  O modelo de footprint distribui
H por uma vizinhança radial, aproximando a área realmente sensoriada pelo
LiDAR/tree-mapper.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class ObservationVector:
    """Representação esparsa de H."""

    indices: np.ndarray
    weights: np.ndarray

    def __post_init__(self) -> None:
        indices = np.asarray(self.indices, dtype=int)
        weights = np.asarray(self.weights, dtype=float)

        if indices.ndim != 1 or weights.ndim != 1:
            raise ValueError('ObservationVector expects 1-D arrays')
        if len(indices) == 0:
            raise ValueError('ObservationVector cannot be empty')
        if len(indices) != len(weights):
            raise ValueError('indices and weights must have the same length')
        if not np.all(np.isfinite(weights)):
            raise ValueError('weights must be finite')

        object.__setattr__(self, 'indices', indices)
        object.__setattr__(self, 'weights', weights)

    def cov_times(self, cov: np.ndarray) -> np.ndarray:
        """Return P H."""
        return cov[:, self.indices] @ self.weights

    def mean(self, mean_values: np.ndarray) -> float:
        """Return H mu."""
        return float(mean_values[self.indices] @ self.weights)

    def variance_from_cov_times(self, p_h: np.ndarray) -> float:
        """Return H P H from a previously computed p_h = P H."""
        return float(self.weights @ p_h[self.indices])


class ObservationModel:
    """Factory for sparse observation vectors H."""

    H_NAIVE = 'H_naive'
    FOOTPRINT = 'footprint'

    def __init__(
        self,
        x_grid: np.ndarray,
        mode: str = H_NAIVE,
        footprint_radius: float = 6.0,
        footprint_sigma: Optional[float] = None,
    ):
        self.X_grid = np.asarray(x_grid, dtype=float)
        if self.X_grid.ndim != 2 or self.X_grid.shape[1] < 2:
            raise ValueError('x_grid must have shape (M, >=2)')

        self.mode = self.normalize_mode(mode)
        self.footprint_radius = float(footprint_radius)
        if footprint_sigma is None:
            footprint_sigma = 0.5 * self.footprint_radius
        self.footprint_sigma = float(footprint_sigma)

        if self.mode == self.FOOTPRINT:
            if self.footprint_radius <= 0.0:
                raise ValueError('footprint_radius must be positive')
            if self.footprint_sigma <= 0.0:
                raise ValueError('footprint_sigma must be positive')

    @classmethod
    def normalize_mode(cls, mode: str) -> str:
        """Normalize aliases used in config files and experiments."""
        key = str(mode).strip().lower().replace('-', '_')
        if key in ('h_naive', 'naive', 'point', 'nearest'):
            return cls.H_NAIVE
        if key in ('footprint', 'sensor_footprint'):
            return cls.FOOTPRINT
        raise ValueError(
            "Unknown GP observation model %r. Use 'H_naive' or 'footprint'."
            % mode
        )

    def vector_at(self, pos_xy: np.ndarray) -> ObservationVector:
        """Build H for an observation taken at pos_xy."""
        if self.mode == self.FOOTPRINT:
            return self._footprint_vector_at(pos_xy)
        return self._naive_vector_at(pos_xy)

    def vector_for_cell_index(self, cell_index: int) -> ObservationVector:
        """Build the legacy one-hot H = e_i for an explicit grid cell."""
        i = int(cell_index)
        if i < 0 or i >= len(self.X_grid):
            raise IndexError('cell_index out of range')
        return ObservationVector(
            np.array([i], dtype=int),
            np.array([1.0], dtype=float),
        )

    def cell_index_for(self, pos_xy: np.ndarray) -> int:
        """Return the nearest grid cell to pos_xy."""
        pos = np.asarray(pos_xy[:2], dtype=float)
        return int(np.argmin(np.linalg.norm(self.X_grid[:, :2] - pos, axis=1)))

    def describe(self) -> str:
        """Human-readable configuration summary for logs."""
        if self.mode == self.FOOTPRINT:
            return (
                f'{self.mode}(radius={self.footprint_radius:.2f}m, '
                f'sigma={self.footprint_sigma:.2f}m)'
            )
        return self.mode

    def _naive_vector_at(self, pos_xy: np.ndarray) -> ObservationVector:
        return self.vector_for_cell_index(self.cell_index_for(pos_xy))

    def _footprint_vector_at(self, pos_xy: np.ndarray) -> ObservationVector:
        pos = np.asarray(pos_xy[:2], dtype=float)
        distances = np.linalg.norm(self.X_grid[:, :2] - pos, axis=1)
        indices = np.flatnonzero(distances <= self.footprint_radius)
        if len(indices) == 0:
            return self._naive_vector_at(pos_xy)

        weights = np.exp(
            -0.5 * (distances[indices] / self.footprint_sigma) ** 2
        )
        weight_sum = float(np.sum(weights))
        if not np.isfinite(weight_sum) or weight_sum <= 0.0:
            return self._naive_vector_at(pos_xy)

        return ObservationVector(indices, weights / weight_sum)
