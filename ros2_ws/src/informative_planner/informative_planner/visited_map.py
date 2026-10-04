"""Visited-space memory for the informative planner.

The GP covariance says how uncertain the latent utility field is. This module
tracks a separate geometric fact: where the UAV has already placed sensor
footprints. It uses the same ObservationModel/H as the GP so the visited map
and the uncertainty map speak the same spatial language.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from informative_planner.observation_model import ObservationModel
from informative_planner.observation_model import ObservationVector


class VisitState:
    """Path-local fantasy state for visited-space novelty."""

    __slots__ = (
        '_owner',
        '_parent',
        '_h',
        'novelty',
        'novelty_accum',
        'revisit',
        'revisit_accum',
        'depth',
    )

    def __init__(
        self,
        owner: 'VisitedMap',
        parent: Optional['VisitState'],
        h: Optional[ObservationVector],
        novelty: float,
        revisit: float,
    ):
        self._owner = owner
        self._parent = parent
        self._h = h
        self.novelty = float(novelty)
        self.revisit = float(revisit)
        self.novelty_accum = (
            parent.novelty_accum + self.novelty
            if parent is not None else 0.0
        )
        self.revisit_accum = (
            parent.revisit_accum + self.revisit
            if parent is not None else 0.0
        )
        self.depth = parent.depth + 1 if parent is not None else 0

    def observe_at(self, pos_xy: np.ndarray) -> 'VisitState':
        """Return path state after fantasizing a sensor footprint at pos_xy."""
        h = self._owner.observation_model.vector_at(pos_xy)
        revisit = self.footprint_mass(h)
        novelty = float(np.exp(-revisit / self._owner.saturation))
        return VisitState(self._owner, self, h, novelty, revisit)

    def footprint_mass(self, h: ObservationVector) -> float:
        """Return current visit mass under H, including path-local visits."""
        mass = float(h.weights @ self._owner.visit_mass[h.indices])

        state = self
        while state is not None and state._h is not None:
            mass += _weighted_overlap(h, state._h)
            state = state._parent

        return mass


class VisitedMap:
    """Grid memory of previously observed sensor footprints."""

    def __init__(
        self,
        observation_model: ObservationModel,
        saturation: float = 1.0,
        decay_halflife_observations: Optional[float] = None,
    ):
        self.observation_model = observation_model
        self.visit_mass = np.zeros(len(observation_model.X_grid), dtype=float)
        self.saturation = max(float(saturation), 1e-6)
        self.observation_count = 0

        if (
            decay_halflife_observations is None
            or float(decay_halflife_observations) <= 0.0
        ):
            self.decay_halflife_observations = None
            self.decay_factor = 1.0
        else:
            self.decay_halflife_observations = float(
                decay_halflife_observations
            )
            self.decay_factor = float(
                np.exp(np.log(0.5) / self.decay_halflife_observations)
            )

    def add_observation(self, pos_xy: np.ndarray) -> None:
        """Mark the footprint at pos_xy as visited."""
        self._apply_decay()
        h = self.observation_model.vector_at(pos_xy)
        self.visit_mass[h.indices] += h.weights
        self.observation_count += 1

    def root(self) -> VisitState:
        """Return path-local root state for RRT fantasy visitation."""
        return VisitState(self, None, None, 0.0, 0.0)

    def novelty_at(self, pos_xy: np.ndarray) -> float:
        """Return immediate novelty at pos_xy without changing state."""
        h = self.observation_model.vector_at(pos_xy)
        revisit = float(h.weights @ self.visit_mass[h.indices])
        return float(np.exp(-revisit / self.saturation))

    def describe(self) -> str:
        if self.decay_halflife_observations is None:
            decay = 'off'
        else:
            decay = f'half_life={self.decay_halflife_observations:.0f}obs'
        return (
            f'visited_map(H={self.observation_model.describe()}, '
            f'saturation={self.saturation:.2f}, decay={decay})'
        )

    def _apply_decay(self) -> None:
        if self.decay_factor >= 1.0:
            return
        if np.any(self.visit_mass):
            self.visit_mass *= self.decay_factor


def _weighted_overlap(a: ObservationVector, b: ObservationVector) -> float:
    """Return sum_i a_i*b_i over shared sparse indices."""
    total = 0.0
    ia = 0
    ib = 0
    while ia < len(a.indices) and ib < len(b.indices):
        ai = int(a.indices[ia])
        bi = int(b.indices[ib])
        if ai == bi:
            total += float(a.weights[ia] * b.weights[ib])
            ia += 1
            ib += 1
        elif ai < bi:
            ia += 1
        else:
            ib += 1
    return total
