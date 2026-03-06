"""Contour extraction: find occupied boundary points within a circle."""

from __future__ import annotations

from .grid import OccupancyGrid
from .transforms import grid_to_world


def extract_contour(
    grid: OccupancyGrid,
    cx: float,
    cy: float,
    r: float,
    threshold: float = 0.65,
) -> list[tuple[float, float]]:
    """Extract contour points (world coords) within circle (cx, cy, r).

    A cell is a contour point if:
      1. Its occupancy probability >= threshold
      2. At least one of its 4-neighbors has probability < threshold or is unknown
    """
    cells = grid.query_circle(cx, cy, r)

    # Build a set of occupied cells for fast neighbor lookup
    occupied: set[tuple[int, int]] = set()
    for gx, gy, prob in cells:
        if prob >= threshold:
            occupied.add((gx, gy))

    contour_points: list[tuple[float, float]] = []
    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    for gx, gy in occupied:
        is_boundary = False
        for dx, dy in neighbors:
            nx, ny = gx + dx, gy + dy
            if (nx, ny) not in occupied:
                is_boundary = True
                break
        if is_boundary:
            wx, wy = grid_to_world(gx, gy, grid.cell_size)
            contour_points.append((wx, wy))

    return contour_points
