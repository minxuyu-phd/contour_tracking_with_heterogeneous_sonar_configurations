"""Contour post-processing: angular merge and outlier removal."""

from __future__ import annotations

import numpy as np


def angular_merge(
    points: np.ndarray,
    cx: float,
    cy: float,
    angle_threshold_deg: float,
) -> np.ndarray:
    """Merge contour points that are close in polar angle, keeping the nearest.

    Parameters
    ----------
    points : np.ndarray, shape (N, 2)
        World-coordinate contour points.
    cx, cy : float
        AUV position (pole for polar conversion).
    angle_threshold_deg : float
        Angular bin width in degrees; points closer than this are merged.

    Returns
    -------
    np.ndarray, shape (M, 2)
        Sorted-by-angle, merged points (M <= N).
    """
    if len(points) == 0:
        return points

    threshold_rad = np.radians(angle_threshold_deg)

    dx = points[:, 0] - cx
    dy = points[:, 1] - cy
    angles = np.arctan2(dy, dx)
    distances = np.hypot(dx, dy)

    order = np.argsort(angles)
    angles = angles[order]
    distances = distances[order]
    sorted_points = points[order]

    n = len(angles)
    keep: list[int] = []

    i = 0
    while i < n:
        # Find the end of the current angular bin
        j = i + 1
        while j < n and (angles[j] - angles[i]) < threshold_rad:
            j += 1
        # Keep the point with the smallest distance in [i, j)
        best = i + int(np.argmin(distances[i:j]))
        keep.append(best)
        i = j

    # Wrap-around check: merge first and last bin if angular gap < threshold
    if len(keep) >= 2:
        gap = (angles[keep[0]] + 2 * np.pi) - angles[keep[-1]]
        if gap < threshold_rad:
            # Keep whichever is closer
            if distances[keep[0]] <= distances[keep[-1]]:
                keep.pop()
            else:
                keep.pop(0)

    return sorted_points[keep]


def remove_outliers(
    points: np.ndarray,
    cx: float,
    cy: float,
    window_size: int,
    factor: float,
) -> np.ndarray:
    """Remove distance outliers using sliding-window median + MAD.

    Parameters
    ----------
    points : np.ndarray, shape (N, 2)
        Contour points sorted by polar angle.
    cx, cy : float
        AUV position.
    window_size : int
        Half-width *k* of the sliding window (full window = 2k+1).
    factor : float
        MAD multiplier threshold; points beyond ``factor * MAD`` from the
        local median are removed.

    Returns
    -------
    np.ndarray, shape (K, 2)
        Filtered points (K <= N).
    """
    n = len(points)
    if n < 2 * window_size + 1:
        return points

    k = window_size
    distances = np.hypot(points[:, 0] - cx, points[:, 1] - cy)

    # Circular padding
    padded = np.concatenate([distances[-k:], distances, distances[:k]])

    # Build sliding window matrix (N, 2k+1)
    indices = np.arange(n)[:, None] + np.arange(2 * k + 1)[None, :]
    windows = padded[indices]

    local_median = np.median(windows, axis=1)
    local_mad = np.median(np.abs(windows - local_median[:, None]), axis=1)
    local_mad = np.maximum(local_mad, 1e-6)

    keep = np.abs(distances - local_median) <= factor * local_mad
    return points[keep]


def ransac_filter(
    points: np.ndarray,
    cx: float,
    cy: float,
    max_distance: float = 0.5,
    n_iterations: int = 50,
    window_size: int = 8,
    vote_threshold: float = 0.5,
) -> np.ndarray:
    """Remove outliers using sliding-window RANSAC with line and arc models.

    For each sliding window along the angle-sorted contour, both a line and a
    circle-arc are fitted via RANSAC.  The model with more inliers wins, and
    the centre point of the window is labelled inlier/outlier accordingly.
    A point is kept if it is voted inlier by at least *vote_threshold* of the
    windows that contain it.

    Parameters
    ----------
    points : np.ndarray, shape (N, 2)
        Contour points sorted by polar angle (output of ``angular_merge``).
    cx, cy : float
        AUV position (used only to compute polar angles for sorting; the
        RANSAC models work in Cartesian space).
    max_distance : float
        Inlier distance threshold — maximum distance from a point to the
        fitted model for it to count as an inlier.
    n_iterations : int
        Number of RANSAC iterations for line fitting.  Arc fitting uses
        ``int(n_iterations * 1.5)`` iterations automatically.
    window_size : int
        Half-width *k* of the sliding window; the full window has 2k+1 points.
    vote_threshold : float
        Fraction of containing windows that must judge a point as inlier for
        it to be kept (0–1).

    Returns
    -------
    np.ndarray, shape (M, 2)
        Filtered points (M <= N).
    """
    n = len(points)
    full_w = 2 * window_size + 1
    if n < full_w:
        return points

    k = window_size
    arc_iterations = int(n_iterations * 1.5)

    # --- helper: RANSAC line fit -------------------------------------------
    def _fit_line_inliers(window_pts: np.ndarray) -> np.ndarray:
        """Return boolean inlier mask for the best line found by RANSAC."""
        m = len(window_pts)
        best_mask = np.zeros(m, dtype=bool)
        best_count = 0
        rng = np.random.default_rng()
        for _ in range(n_iterations):
            idx = rng.choice(m, size=2, replace=False)
            p1 = window_pts[idx[0]]
            p2 = window_pts[idx[1]]
            d = p2 - p1
            length = np.hypot(d[0], d[1])
            if length < 1e-12:
                continue
            # vectorised cross-product distance
            diff = window_pts - p1  # (m, 2)
            dists = np.abs(diff[:, 0] * d[1] - diff[:, 1] * d[0]) / length
            mask = dists < max_distance
            count = int(mask.sum())
            if count > best_count:
                best_count = count
                best_mask = mask
        return best_mask

    # --- helper: RANSAC arc (circle) fit -----------------------------------
    def _fit_arc_inliers(window_pts: np.ndarray) -> np.ndarray:
        """Return boolean inlier mask for the best circle found by RANSAC."""
        m = len(window_pts)
        best_mask = np.zeros(m, dtype=bool)
        best_count = 0
        rng = np.random.default_rng()
        for _ in range(arc_iterations):
            idx = rng.choice(m, size=3, replace=False)
            x1, y1 = window_pts[idx[0]]
            x2, y2 = window_pts[idx[1]]
            x3, y3 = window_pts[idx[2]]
            A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2
            if abs(A) < 1e-10:
                continue  # collinear
            sq1 = x1 * x1 + y1 * y1
            sq2 = x2 * x2 + y2 * y2
            sq3 = x3 * x3 + y3 * y3
            ccx = (sq1 * (y2 - y3) + sq2 * (y3 - y1) + sq3 * (y1 - y2)) / (2 * A)
            ccy = (sq1 * (x3 - x2) + sq2 * (x1 - x3) + sq3 * (x2 - x1)) / (2 * A)
            r = np.hypot(x1 - ccx, y1 - ccy)
            dists = np.abs(np.hypot(window_pts[:, 0] - ccx, window_pts[:, 1] - ccy) - r)
            mask = dists < max_distance
            count = int(mask.sum())
            if count > best_count:
                best_count = count
                best_mask = mask
        return best_mask

    # --- voting accumulator ------------------------------------------------
    inlier_votes = np.zeros(n, dtype=np.int32)
    total_votes = np.zeros(n, dtype=np.int32)

    # Circular padding of indices
    padded_idx = np.concatenate([np.arange(n - k, n), np.arange(n), np.arange(k)])

    for i in range(n):
        win_idx = padded_idx[i: i + full_w]
        window_pts = points[win_idx]

        line_mask = _fit_line_inliers(window_pts)
        arc_mask = _fit_arc_inliers(window_pts)

        best_mask = arc_mask if int(arc_mask.sum()) > int(line_mask.sum()) else line_mask

        # Centre point of the window is at position k
        total_votes[win_idx[k]] += 1
        if best_mask[k]:
            inlier_votes[win_idx[k]] += 1

    keep = inlier_votes >= (vote_threshold * total_votes)
    return points[keep]
