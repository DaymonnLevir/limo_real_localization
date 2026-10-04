#!/usr/bin/env python3
"""Terrain model and height-above-ground normalization.

Keeps a persistent 2.5D elevation grid (DEM) in a fixed world frame, fed
incrementally with ground points, and normalizes clouds against it:

    z_norm(p) = z(p) - z_ground(x_p, y_p)

"""

import numpy as np
from scipy import ndimage

DEFAULT_CELL_SIZE = 0.5
DEFAULT_FILL_MAX_DIST = 3.0


class GroundModel:
    """Persistent elevation grid anchored to a fixed world origin."""

    def __init__(
        self,
        cell_size=DEFAULT_CELL_SIZE,
        origin=(-100.0, -100.0),
        shape=(400, 400),
        fill_max_dist=DEFAULT_FILL_MAX_DIST,
        smooth_cells=3,
        ground_tolerance=0.20,
        seed_percentile=25.0,
    ):
        self.cell_size = float(cell_size)
        self.origin = np.asarray(origin, dtype=float)
        self.shape = (int(shape[0]), int(shape[1]))
        self.fill_max_dist = float(fill_max_dist)
        self.smooth_cells = int(smooth_cells)
        self.ground_tolerance = float(ground_tolerance)
        self.seed_percentile = float(seed_percentile)

        self._z_sum = np.zeros(self.shape, dtype=np.float64)
        self._count = np.zeros(self.shape, dtype=np.int64)

        # Filled/smoothed surface and the distance to the nearest observation
        # that produced it. Rebuilt lazily; see _refresh.
        self._z_filled = None
        self._dist = None
        self._dirty = True

    # ---------------------------------------------------------------- grid

    def _index(self, xy):
        """World XY -> integer cell index, plus a mask of points inside the grid."""
        idx = np.floor((xy - self.origin) / self.cell_size).astype(np.int64)
        inside = (
            (idx[:, 0] >= 0)
            & (idx[:, 0] < self.shape[0])
            & (idx[:, 1] >= 0)
            & (idx[:, 1] < self.shape[1])
        )
        return idx, inside

    def _continuous_index(self, xy):
        """World XY -> fractional cell index measured from cell centers."""
        return (xy - self.origin) / self.cell_size - 0.5

    @property
    def observed(self):
        return self._count > 0

    # -------------------------------------------------------------- update

    def update(self, points, ground_mask=None):
        """Fold ground observations into the grid.

        points: (N, 3) in the world frame. ground_mask selects the ground
        returns; without one the ground is guessed per cell, which is only
        meant for running this module standalone.
        """
        points = np.asarray(points, dtype=float)
        if points.shape[0] == 0:
            return 0
        if ground_mask is None:
            ground_mask = self.guess_ground(points)
        ground = points[np.asarray(ground_mask, dtype=bool)]
        if ground.shape[0] == 0:
            return 0

        idx, inside = self._index(ground[:, :2])
        idx = idx[inside]
        z = ground[inside, 2]
        if z.size == 0:
            return 0

        # np.add.at over a raveled view: no per-point Python loop, and no
        # full-grid temporary the way np.bincount(minlength=...) would need.
        flat = idx[:, 0] * self.shape[1] + idx[:, 1]
        np.add.at(self._z_sum.ravel(), flat, z)
        np.add.at(self._count.ravel(), flat, 1)
        self._dirty = True
        return int(z.size)

    def guess_ground(self, points):
        """Crude standalone ground split: lowest stratum of each cell.

        Real runs should pass a mask from a dedicated segmenter; this only
        exists so the module is usable and testable on its own.
        """
        points = np.asarray(points, dtype=float)
        idx, inside = self._index(points[:, :2])
        mask = np.zeros(points.shape[0], dtype=bool)
        if not np.any(inside):
            return mask

        flat = idx[:, 0] * self.shape[1] + idx[:, 1]
        flat_in = flat[inside]
        z_in = points[inside, 2]

        order = np.argsort(flat_in, kind="stable")
        flat_s, z_s = flat_in[order], z_in[order]
        starts = np.flatnonzero(np.r_[True, flat_s[1:] != flat_s[:-1]])
        floor = np.empty(flat_s.shape[0], dtype=float)
        for begin, end in zip(starts, np.r_[starts[1:], flat_s.shape[0]]):
            floor[begin:end] = np.percentile(z_s[begin:end], self.seed_percentile)

        # floor/z_s are in sorted order; scatter back through `order`.
        keep = np.zeros(flat_s.shape[0], dtype=bool)
        keep[order] = z_s - floor <= self.ground_tolerance
        mask[inside] = keep
        return mask

    # ------------------------------------------------------------- surface

    def _refresh(self):
        """Fill holes from the nearest observed cell, then smooth."""
        if not self._dirty:
            return
        observed = self.observed
        if not np.any(observed):
            self._z_filled = np.zeros(self.shape, dtype=float)
            self._dist = np.full(self.shape, np.inf)
            self._dirty = False
            return

        z_mean = np.where(observed, self._z_sum / np.maximum(self._count, 1), 0.0)

        # One call yields both the nearest-observed-cell lookup (the fill) and
        # the distance to it (the confidence).
        dist, nearest = ndimage.distance_transform_edt(
            ~observed, return_distances=True, return_indices=True
        )
        filled = z_mean[tuple(nearest)]
        if self.smooth_cells > 1:
            filled = ndimage.uniform_filter(filled, self.smooth_cells)

        self._z_filled = filled
        self._dist = dist * self.cell_size
        self._dirty = False

    # --------------------------------------------------------------- query

    def ground_z(self, xy):
        """Bilinearly interpolated ground height at world XY."""
        self._refresh()
        coords = self._continuous_index(np.asarray(xy, dtype=float)).T
        return ndimage.map_coordinates(self._z_filled, coords, order=1, mode="nearest")

    def confidence_distance(self, xy):
        """Distance (m) to the nearest cell that actually saw the ground."""
        self._refresh()
        coords = self._continuous_index(np.asarray(xy, dtype=float)).T
        return ndimage.map_coordinates(self._dist, coords, order=0, mode="nearest")

    def normalize(self, points):
        """Full cloud in, heights above ground out.

        Returns (hag, valid): hag is z - z_ground per point; valid clears
        points outside the grid and points whose ground height had to be
        extrapolated further than fill_max_dist.
        """
        points = np.asarray(points, dtype=float)
        if points.shape[0] == 0:
            return np.empty(0), np.empty(0, dtype=bool)

        xy = points[:, :2]
        hag = points[:, 2] - self.ground_z(xy)

        _, inside = self._index(xy)
        valid = inside & (self.confidence_distance(xy) <= self.fill_max_dist)
        return hag, valid


def normalize_cloud(points, model=None, ground_mask=None, **kwargs):
    """One-shot convenience wrapper: build a model, fold the cloud in, normalize."""
    if model is None:
        model = GroundModel(**kwargs)
    model.update(points, ground_mask=ground_mask)
    hag, valid = model.normalize(points)
    return hag, valid, model
