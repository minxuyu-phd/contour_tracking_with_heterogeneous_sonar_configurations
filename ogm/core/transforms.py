"""Coordinate transforms: body→world, world→grid, beam footprint sampling."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .sensors import MSISData, SBESData


def body_to_world(bx: float, by: float, pose_x: float, pose_y: float, heading: float) -> tuple[float, float]:
    """Transform a point from body frame to world frame."""
    c, s = math.cos(heading), math.sin(heading)
    wx = pose_x + c * bx - s * by
    wy = pose_y + s * bx + c * by
    return wx, wy


def world_to_grid(wx: float, wy: float, cell_size: float) -> tuple[int, int]:
    """Convert world coordinates to grid indices."""
    return int(math.floor(wx / cell_size)), int(math.floor(wy / cell_size))


def grid_to_world(gx: int, gy: int, cell_size: float) -> tuple[float, float]:
    """Convert grid indices to world coordinates (cell center)."""
    return (gx + 0.5) * cell_size, (gy + 0.5) * cell_size


_MIN_WEIGHT = 0.01


def _expand_hit_gaussian(
    wx: float,
    wy: float,
    beam_angle: float,
    dist: float,
    aperture_deg: float,
    cell_size: float,
) -> list[tuple[tuple[int, int], float]]:
    """Expand a single hit point with Gaussian weighting perpendicular to beam.

    Uses a sharper Gaussian exp(-2*(offset/sigma)^2) truncated at min_weight=0.01
    for faster decay and fewer cells. Returns list of (grid_cell, weight) where
    weight is in [_MIN_WEIGHT, 1.0].
    When aperture <= 0 or sigma < cell_size/2, returns only the center cell.
    """
    center_cell = world_to_grid(wx, wy, cell_size)

    if aperture_deg <= 0.0:
        return [(center_cell, 1.0)]

    sigma = dist * math.tan(math.radians(aperture_deg / 2.0))
    if sigma < cell_size / 2.0:
        return [(center_cell, 1.0)]

    perp_x = -math.sin(beam_angle)
    perp_y = math.cos(beam_angle)

    # Truncation radius: exp(-2(r/sigma)^2) >= MIN_WEIGHT
    # => r <= sigma * sqrt(ln(1/MIN_WEIGHT) / 2)
    max_offset = sigma * math.sqrt(math.log(1.0 / _MIN_WEIGHT) / 2.0)
    n_steps = int(math.ceil(max_offset / cell_size))

    # Vectorized computation of offsets, weights, and grid coordinates
    offsets = np.arange(-n_steps, n_steps + 1) * cell_size
    weights = np.exp(-2.0 * (offsets / sigma) ** 2)

    mask = weights >= _MIN_WEIGHT
    offsets = offsets[mask]
    weights = weights[mask]

    gxs = np.floor((wx + perp_x * offsets) / cell_size).astype(np.int64)
    gys = np.floor((wy + perp_y * offsets) / cell_size).astype(np.int64)

    # De-duplicate cells, keeping the maximum weight per unique cell
    cells = np.column_stack([gxs, gys])
    unique_cells, indices = np.unique(cells, axis=0, return_inverse=True)

    best_weights = np.zeros(len(unique_cells))
    np.maximum.at(best_weights, indices, weights)

    return [
        ((int(unique_cells[i, 0]), int(unique_cells[i, 1])), float(best_weights[i]))
        for i in range(len(unique_cells))
    ]


def sbes_footprint(
    data: SBESData, cell_size: float, aperture_deg: float = 0.0,
) -> tuple[list[tuple[int, int]], list[tuple[tuple[int, int], float]]]:
    """Compute SBES beam footprint cells.

    Returns (free_cells, hit_cells_with_weight):
      - free_cells: grid cells along beam before hit
      - hit_cells_with_weight: list of (cell, weight) with Gaussian expansion
    """
    step = cell_size
    n_steps = max(1, int(data.range_m / step))
    dx = math.cos(data.heading)
    dy = math.sin(data.heading)

    free_cells: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()

    for i in range(n_steps):
        dist = (i + 0.5) * step
        wx = data.x + dx * dist
        wy = data.y + dy * dist
        cell = world_to_grid(wx, wy, cell_size)
        if cell not in seen:
            seen.add(cell)
            free_cells.append(cell)

    # Hit at measured range — expand with Gaussian
    hx = data.x + dx * data.range_m
    hy = data.y + dy * data.range_m
    hit_cells = _expand_hit_gaussian(hx, hy, data.heading, data.range_m, aperture_deg, cell_size)

    # Remove hit cells from free list
    hit_set = {c for c, _ in hit_cells}
    free_cells = [c for c in free_cells if c not in hit_set]

    return free_cells, hit_cells


def msis_footprint(
    data: MSISData, cell_size: float, aperture_deg: float = 0.0, bin_threshold: int = 0,
) -> tuple[list[tuple[int, int]], list[tuple[tuple[int, int], float]]]:
    """Compute MSIS beam footprint cells.

    Returns (free_cells, hit_cells_with_weight):
      - free_cells: grid cells before first strong return
      - hit_cells_with_weight: list of (cell, weight) for echo bins above threshold
    """
    beam_world_angle = data.heading + data.beam_angle
    dx = math.cos(beam_world_angle)
    dy = math.sin(beam_world_angle)

    n_bins = len(data.bin_values)
    if n_bins == 0:
        return [], []

    bin_width = data.range_max / n_bins

    # Find first hit bin (absolute threshold) — vectorized
    hit_indices = np.nonzero(np.asarray(data.bin_values) >= bin_threshold)[0]
    first_hit = int(hit_indices[0]) if len(hit_indices) > 0 else -1

    free_cells: list[tuple[int, int]] = []
    hit_cells: list[tuple[tuple[int, int], float]] = []
    seen: set[tuple[int, int]] = set()

    # Free cells: bins before first hit
    end_free = first_hit if first_hit >= 0 else n_bins
    for i in range(end_free):
        dist = (i + 0.5) * bin_width
        wx = data.x + dx * dist
        wy = data.y + dy * dist
        cell = world_to_grid(wx, wy, cell_size)
        if cell not in seen:
            seen.add(cell)
            free_cells.append(cell)

    # Hit cells: bins from first hit onward that are above threshold
    if first_hit >= 0:
        for i in range(first_hit, n_bins):
            v = int(data.bin_values[i])
            if v >= bin_threshold:
                dist = (i + 0.5) * bin_width
                wx = data.x + dx * dist
                wy = data.y + dy * dist
                expanded = _expand_hit_gaussian(wx, wy, beam_world_angle, dist, aperture_deg, cell_size)
                for cell, weight in expanded:
                    if cell not in seen:
                        seen.add(cell)
                        hit_cells.append((cell, weight))

    # Remove any hit cells from free list
    hit_set = {c for c, _ in hit_cells}
    free_cells = [c for c in free_cells if c not in hit_set]

    return free_cells, hit_cells
