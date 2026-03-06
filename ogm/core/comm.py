"""nng_comm communication: 4 Subscriber threads + 1 Replier contour service."""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from typing import Callable

import numpy as np
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent.parent.parent))

from nng_comm.scripts import Subscriber, Replier

from .config import AppConfig, PruningConfig, SensorConfig, ServiceConfig
from .contour import extract_contour
from .contour_filter import angular_merge, ransac_filter
from .grid import OccupancyGrid
from .sensors import (
    MSISData,
    OdometryData,
    RawMSISReading,
    RawSBESReading,
    SBESData,
    parse_msis,
    parse_odometry,
    parse_sbes,
)

logger = logging.getLogger(__name__)


class SensorSubscriber(threading.Thread):
    """Subscribe to a sensor address via nng_comm Subscriber and feed data to the grid."""

    def __init__(
        self,
        name: str,
        sensor_cfg: SensorConfig,
        parse_fn: Callable[[bytes | dict], object | None],
        handle_fn: Callable[[object], None],
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name=name, daemon=True)
        self.sensor_cfg = sensor_cfg
        self.parse_fn = parse_fn
        self.handle_fn = handle_fn
        self.stop_event = stop_event

    def run(self) -> None:
        logger.info("%s: connecting to %s", self.name, self.sensor_cfg.address)
        sub = Subscriber()
        try:
            if not sub.init(self.sensor_cfg.address):
                logger.error("%s: failed to init subscriber", self.name)
                return
            logger.info("%s: dial issued (non-blocking)", self.name)

            while not self.stop_event.is_set():
                msg = sub.receive(timeout_ms=500)
                if msg is None:
                    continue

                logger.debug("%s: received message", self.name)
                data = self.parse_fn(msg)
                if data is None:
                    logger.debug("%s: parse returned None (skipped)", self.name)
                else:
                    logger.debug("%s: dispatching to handler", self.name)
                    try:
                        self.handle_fn(data)
                    except Exception:
                        logger.exception("%s: handle error", self.name)

        except Exception:
            logger.exception("%s: fatal error", self.name)
        finally:
            sub.close()
        logger.info("%s: stopped", self.name)


class ContourServer:
    """Replier server responding to contour queries.

    Request JSON: {"x": float, "y": float, "r": float}
      - x/y are optional; if omitted, the current odometry position is used.
      - r is optional; defaults to 40.
    Response JSON: {"points": [[x1,y1], [x2,y2], ...]}
    """

    def __init__(
        self,
        service_cfg: ServiceConfig,
        grid: OccupancyGrid,
        stop_event: threading.Event,
    ) -> None:
        self.grid = grid
        self.threshold = service_cfg.contour_threshold
        self.angular_merge_threshold_deg = service_cfg.angular_merge_threshold_deg
        self.outlier_window_size = service_cfg.outlier_window_size
        self.outlier_factor = service_cfg.outlier_factor
        self.ransac_max_distance = service_cfg.ransac_max_distance
        self.ransac_n_iterations = service_cfg.ransac_n_iterations
        self.ransac_window_size = service_cfg.ransac_window_size
        self.ransac_vote_threshold = service_cfg.ransac_vote_threshold
        self.stop_event = stop_event
        self._contour_points: list[tuple[float, float]] = []
        self._contour_lock = threading.Lock()

        self._replier = Replier()
        self._replier.init(service_cfg.address)
        self._replier.set_handler(self._handle_request)

    def get_latest_contour(self) -> list[tuple[float, float]]:
        """Get the most recently computed contour points (for visualization)."""
        with self._contour_lock:
            return list(self._contour_points)

    def _handle_request(self, req: dict) -> dict:
        """Handle a contour request and return a response dict."""
        try:
            r = float(req.get("r", 40.0))

            if "x" in req and "y" in req:
                cx = float(req["x"])
                cy = float(req["y"])
            else:
                pose = self.grid.get_pose()
                if pose is None:
                    return {"error": "no odometry available"}
                cx = pose.x
                cy = pose.y

            raw_points = extract_contour(self.grid, cx, cy, r, self.threshold)
            if raw_points:
                pts = np.array(raw_points)
                pts = angular_merge(pts, cx, cy, self.angular_merge_threshold_deg)
                pts = ransac_filter(
                    pts, cx, cy,
                    max_distance=self.ransac_max_distance,
                    n_iterations=self.ransac_n_iterations,
                    window_size=self.ransac_window_size,
                    vote_threshold=self.ransac_vote_threshold,
                )
                points = pts.tolist()
            else:
                points = []
            with self._contour_lock:
                self._contour_points = points

            return {"points": points}
        except Exception:
            logger.exception("ContourServer: request handling error")
            return {"error": "bad request"}

    def start(self) -> None:
        logger.info("ContourServer: starting replier")
        self._replier.start()

    def stop(self) -> None:
        self._replier.stop()

    def join(self, timeout: float | None = None) -> None:
        if self._replier._thread is not None:
            self._replier._thread.join(timeout=timeout)

    def close(self) -> None:
        self._replier.close()


def start_subscribers(
    config: AppConfig,
    grid: OccupancyGrid,
    stop_event: threading.Event,
) -> list[SensorSubscriber]:
    """Create and start the four sensor subscriber threads."""

    def handle_odom(data: OdometryData) -> None:
        logger.debug("odom: x=%.2f y=%.2f heading=%.3f", data.x, data.y, data.heading)
        grid.update_pose(data)

    def handle_msis(raw: RawMSISReading) -> None:
        pose = grid.get_pose()
        if pose is None:
            logger.warning("msis: no pose available, skipping")
            return
        logger.debug(
            "msis: angle=%.1f° range_max=%.1f bins=%d pose=(%.2f,%.2f)",
            math.degrees(raw.beam_angle), raw.range_max, len(raw.bin_values), pose.x, pose.y,
        )
        data = MSISData(
            timestamp=time.time(),
            x=pose.x,
            y=pose.y,
            heading=pose.heading,
            beam_angle=raw.beam_angle,
            range_max=raw.range_max,
            bin_values=raw.bin_values,
        )
        grid.update_msis(
            data,
            confidence=config.msis.confidence,
            aperture_deg=config.msis.aperture,
            bin_threshold=config.msis.bin_threshold,
            free_weight=config.msis.free_weight,
        )

    def _make_sbes_handler(confidence: float, beam_angle_deg: float, aperture_deg: float, free_weight: float) -> Callable[[RawSBESReading], None]:
        beam_angle_rad = math.radians(beam_angle_deg)

        def handle_sbes(raw: RawSBESReading) -> None:
            pose = grid.get_pose()
            if pose is None:
                logger.warning("sbes: no pose available, skipping")
                return
            world_heading = pose.heading + beam_angle_rad
            logger.debug(
                "sbes: dist=%.2f heading=%.3f pose=(%.2f,%.2f)",
                raw.distance, world_heading, pose.x, pose.y,
            )
            data = SBESData(
                timestamp=time.time(),
                x=pose.x,
                y=pose.y,
                heading=world_heading,
                range_m=raw.distance,
            )
            # grid.update_sbes(data, confidence, aperture_deg, free_weight)  # temporarily disabled
        return handle_sbes

    subs = [
        SensorSubscriber("sub-odometry", config.odometry, parse_odometry, handle_odom, stop_event),
        SensorSubscriber("sub-msis", config.msis, parse_msis, handle_msis, stop_event),
        SensorSubscriber("sub-sbes-left", config.sbes_left, parse_sbes, _make_sbes_handler(config.sbes_left.confidence, config.sbes_left.beam_angle, config.sbes_left.aperture, config.sbes_left.free_weight), stop_event),
        SensorSubscriber("sub-sbes-right", config.sbes_right, parse_sbes, _make_sbes_handler(config.sbes_right.confidence, config.sbes_right.beam_angle, config.sbes_right.aperture, config.sbes_right.free_weight), stop_event),
    ]
    for s in subs:
        s.start()
    return subs


def start_contour_server(
    config: AppConfig,
    grid: OccupancyGrid,
    stop_event: threading.Event,
) -> ContourServer:
    """Create and start the contour Replier server."""
    server = ContourServer(
        config.service,
        grid,
        stop_event,
    )
    server.start()
    return server


class PruningThread(threading.Thread):
    """Daemon thread that periodically prunes grid cells far from the AUV."""

    def __init__(
        self,
        pruning_cfg: PruningConfig,
        grid: OccupancyGrid,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name="pruner", daemon=True)
        self.cfg = pruning_cfg
        self.grid = grid
        self.stop_event = stop_event
        self._last_x: float | None = None
        self._last_y: float | None = None
        self._cycle_count = 0

    def run(self) -> None:
        logger.info(
            "PruningThread: started (keep_radius=%.0fm, interval=%.1fs, min_move=%.1fm)",
            self.cfg.keep_radius, self.cfg.interval, self.cfg.min_move,
        )
        while not self.stop_event.wait(timeout=self.cfg.interval):
            try:
                self._tick()
            except Exception:
                logger.exception("PruningThread: error during pruning cycle")
        logger.info("PruningThread: stopped")

    def _tick(self) -> None:
        pose = self.grid.get_pose()
        if pose is None:
            return

        # Check minimum movement
        if self._last_x is not None:
            dx = pose.x - self._last_x
            dy = pose.y - self._last_y
            if dx * dx + dy * dy < self.cfg.min_move * self.cfg.min_move:
                return

        self._last_x = pose.x
        self._last_y = pose.y

        # Compute keep rectangle in grid coordinates
        cell = self.grid.cell_size
        r = self.cfg.keep_radius
        gx_min = int(math.floor((pose.x - r) / cell))
        gy_min = int(math.floor((pose.y - r) / cell))
        gx_max = int(math.floor((pose.x + r) / cell))
        gy_max = int(math.floor((pose.y + r) / cell))

        t0 = time.monotonic()
        removed = self.grid.tree.prune_outside_rect(gx_min, gy_min, gx_max, gy_max)
        elapsed_ms = (time.monotonic() - t0) * 1000.0

        if removed > 0:
            logger.info(
                "PruningThread: pruned %d cells in %.1fms (center=%.1f,%.1f)",
                removed, elapsed_ms, pose.x, pose.y,
            )

        self._cycle_count += 1
        if self.cfg.log_interval > 0 and self._cycle_count % self.cfg.log_interval == 0:
            total = self.grid.tree.count_cells()
            logger.info("PruningThread: total cells = %d", total)


def start_pruning(
    config: AppConfig,
    grid: OccupancyGrid,
    stop_event: threading.Event,
) -> PruningThread | None:
    """Create and start the pruning daemon thread if enabled."""
    if not config.pruning.enabled:
        logger.info("PruningThread: disabled by config")
        return None
    thread = PruningThread(config.pruning, grid, stop_event)
    thread.start()
    return thread
