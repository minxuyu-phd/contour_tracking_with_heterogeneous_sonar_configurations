"""Configuration loading and validation."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class SensorConfig:
    address: str
    confidence: float = 0.9
    beam_angle: float = 0.0  # sensor mount angle (degrees) relative to body frame
    aperture: float = 0.0    # horizontal beam aperture (degrees), 0 = no expansion
    bin_threshold: int = 0   # MSIS absolute bin threshold (uint8), 0 = accept all
    free_weight: float = 0.2  # scale factor for free-space log-odds delta

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SensorConfig:
        return cls(
            address=d["address"],
            confidence=d.get("confidence", 0.9),
            beam_angle=d.get("beam_angle", 0.0),
            aperture=d.get("aperture", 0.0),
            bin_threshold=d.get("bin_threshold", 0),
            free_weight=d.get("free_weight", 0.2),
        )


@dataclass
class ServiceConfig:
    address: str = "tcp://192.168.5.10:5010"
    contour_threshold: float = 0.65
    angular_merge_threshold_deg: float = 1.0
    outlier_window_size: int = 5
    outlier_factor: float = 3.0
    ransac_max_distance: float = 0.5
    ransac_n_iterations: int = 50
    ransac_window_size: int = 8
    ransac_vote_threshold: float = 0.5

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> ServiceConfig:
        return cls(
            address=d.get("address", cls.address),
            contour_threshold=d.get("contour_threshold", cls.contour_threshold),
            angular_merge_threshold_deg=d.get("angular_merge_threshold_deg", cls.angular_merge_threshold_deg),
            outlier_window_size=d.get("outlier_window_size", cls.outlier_window_size),
            outlier_factor=d.get("outlier_factor", cls.outlier_factor),
            ransac_max_distance=d.get("ransac_max_distance", cls.ransac_max_distance),
            ransac_n_iterations=d.get("ransac_n_iterations", cls.ransac_n_iterations),
            ransac_window_size=d.get("ransac_window_size", cls.ransac_window_size),
            ransac_vote_threshold=d.get("ransac_vote_threshold", cls.ransac_vote_threshold),
        )


@dataclass
class VisualizationConfig:
    enabled: bool = True
    window_width: int = 1024
    window_height: int = 768
    fps: int = 30
    camera_x: float = 120.0
    camera_y: float = 0.0
    render_threshold: float = 0.0  # minimum probability to render a cell (0.0 = render all)
    grid_opacity: float = 0.7
    follow_auv: bool = False

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> VisualizationConfig:
        return cls(
            enabled=d.get("enabled", True),
            window_width=d.get("window_width", 1024),
            window_height=d.get("window_height", 768),
            fps=d.get("fps", 30),
            camera_x=d.get("camera_x", 120.0),
            camera_y=d.get("camera_y", 0.0),
            render_threshold=d.get("render_threshold", 0.0),
            grid_opacity=d.get("grid_opacity", 0.7),
            follow_auv=d.get("follow_auv", False),
        )


@dataclass
class PruningConfig:
    enabled: bool = True
    keep_radius: float = 200.0   # meters — cells outside this radius from AUV are pruned
    interval: float = 5.0        # seconds between pruning cycles
    min_move: float = 10.0       # AUV must move at least this far (m) to trigger pruning
    log_interval: int = 6        # log total cell count every N pruning cycles

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> PruningConfig:
        return cls(
            enabled=d.get("enabled", True),
            keep_radius=d.get("keep_radius", 200.0),
            interval=d.get("interval", 5.0),
            min_move=d.get("min_move", 10.0),
            log_interval=d.get("log_interval", 6),
        )


@dataclass
class AppConfig:
    cell_size: float = 0.1
    odometry: SensorConfig = field(
        default_factory=lambda: SensorConfig("tcp://127.0.0.1:5001")
    )
    msis: SensorConfig = field(
        default_factory=lambda: SensorConfig("tcp://127.0.0.1:5002")
    )
    sbes_left: SensorConfig = field(
        default_factory=lambda: SensorConfig("tcp://127.0.0.1:5003", 0.85)
    )
    sbes_right: SensorConfig = field(
        default_factory=lambda: SensorConfig("tcp://127.0.0.1:5004", 0.85)
    )
    service: ServiceConfig = field(default_factory=ServiceConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    pruning: PruningConfig = field(default_factory=PruningConfig)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> AppConfig:
        sensors = d.get("sensors", {})
        return cls(
            cell_size=d.get("cell_size", 0.1),
            odometry=SensorConfig.from_dict(sensors["odometry"]) if "odometry" in sensors else cls.odometry,
            msis=SensorConfig.from_dict(sensors["msis"]) if "msis" in sensors else cls.msis,
            sbes_left=SensorConfig.from_dict(sensors["sbes_left"]) if "sbes_left" in sensors else cls.sbes_left,
            sbes_right=SensorConfig.from_dict(sensors["sbes_right"]) if "sbes_right" in sensors else cls.sbes_right,
            service=ServiceConfig.from_dict(d["service"]) if "service" in d else ServiceConfig(),
            visualization=VisualizationConfig.from_dict(d["visualization"]) if "visualization" in d else VisualizationConfig(),
            pruning=PruningConfig.from_dict(d["pruning"]) if "pruning" in d else PruningConfig(),
        )

    @classmethod
    def load(cls, path: str | Path) -> AppConfig:
        with open(path) as f:
            data = json.load(f)
        return cls.from_dict(data)
