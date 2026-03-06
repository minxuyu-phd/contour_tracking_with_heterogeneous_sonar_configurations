"""OccupancyGrid: Bayesian log-odds update with sensor-to-grid pipeline."""

from __future__ import annotations

import math
import threading

from .quadtree import QuadTree
from .sensors import MSISData, OdometryData, SBESData
from .transforms import msis_footprint, sbes_footprint


def _log_odds(p: float) -> float:
    """Convert probability to log-odds."""
    p = max(1e-9, min(1.0 - 1e-9, p))
    return math.log(p / (1.0 - p))


def _probability(lo: float) -> float:
    """Convert log-odds to probability."""
    return 1.0 / (1.0 + math.exp(-lo))


class OccupancyGrid:
    """2D occupancy grid backed by a quadtree, updated via log-odds Bayes rule."""

    def __init__(self, cell_size: float) -> None:
        self.cell_size = cell_size
        self.tree = QuadTree(cell_size)

        # Latest odometry pose (thread-safe via lock)
        self._pose_lock = threading.Lock()
        self._pose: OdometryData | None = None

    # ---- pose bookkeeping ----

    def update_pose(self, odom: OdometryData) -> None:
        with self._pose_lock:
            self._pose = odom

    def get_pose(self) -> OdometryData | None:
        with self._pose_lock:
            return self._pose

    # ---- sensor updates ----

    def update_sbes(self, data: SBESData, confidence: float, aperture_deg: float = 0.0, free_weight: float = 0.2) -> None:
        """Process a single-beam echo sounder measurement."""
        free_cells, hit_cells = sbes_footprint(data, self.cell_size, aperture_deg)
        free_delta = _log_odds(1.0 - confidence) * free_weight
        half_range = confidence - 0.5

        updates = [(c[0], c[1], free_delta) for c in free_cells]
        for cell, weight in hit_cells:
            prob = 0.5 + half_range * weight
            updates.append((cell[0], cell[1], _log_odds(prob)))

        self.tree.update_cells_batch(updates)

    def update_msis(self, data: MSISData, confidence: float = 0.9, aperture_deg: float = 0.0, bin_threshold: int = 0, free_weight: float = 0.1) -> None:
        """Process an MSIS scan beam."""
        free_cells, hit_cells = msis_footprint(data, self.cell_size, aperture_deg, bin_threshold)
        free_delta = _log_odds(0.3) * free_weight  # free observation
        half_range = confidence - 0.5

        updates = [(c[0], c[1], free_delta) for c in free_cells]
        for cell, weight in hit_cells:
            prob = 0.5 + half_range * weight
            updates.append((cell[0], cell[1], _log_odds(prob)))

        self.tree.update_cells_batch(updates)

    # ---- queries ----

    def get_probability(self, gx: int, gy: int) -> float:
        """Return occupancy probability for grid cell, 0.5 if unknown."""
        lo = self.tree.get_cell(gx, gy)
        if lo is None:
            return 0.5
        return _probability(lo)

    def query_rect(self, xmin: float, ymin: float, xmax: float, ymax: float) -> list[tuple[int, int, float]]:
        """Query cells in world-coordinate rect. Returns (gx, gy, probability)."""
        raw = self.tree.query_rect(xmin, ymin, xmax, ymax)
        return [(gx, gy, _probability(lo)) for gx, gy, lo in raw]

    def query_circle(self, cx: float, cy: float, r: float) -> list[tuple[int, int, float]]:
        """Query cells in world-coordinate circle. Returns (gx, gy, probability)."""
        raw = self.tree.query_circle(cx, cy, r)
        return [(gx, gy, _probability(lo)) for gx, gy, lo in raw]
