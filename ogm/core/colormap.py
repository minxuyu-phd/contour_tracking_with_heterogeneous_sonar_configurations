"""Jet colormap LUT (256 entries) for probability-to-color mapping."""

from __future__ import annotations


def _jet_value(t: float) -> tuple[int, int, int]:
    """Compute a single Jet colormap RGB for t in [0, 1]."""
    # Jet: blue(0) -> cyan(0.25) -> green(0.5) -> yellow(0.75) -> red(1)
    r = min(1.0, max(0.0, 1.5 - abs(4.0 * t - 3.0)))
    g = min(1.0, max(0.0, 1.5 - abs(4.0 * t - 2.0)))
    b = min(1.0, max(0.0, 1.5 - abs(4.0 * t - 1.0)))
    return int(r * 255), int(g * 255), int(b * 255)


# Pre-compute 256-entry LUT
JET_LUT: list[tuple[int, int, int]] = [_jet_value(i / 255.0) for i in range(256)]


def probability_to_color(prob: float) -> tuple[int, int, int]:
    """Map occupancy probability [0, 1] to Jet RGB color."""
    idx = max(0, min(255, int(prob * 255)))
    return JET_LUT[idx]
