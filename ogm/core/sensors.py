"""Sensor data structures and parsing.

Parses JSON payloads from SAM-AUV simulator publishers.

- Odometry: full pose (position + heading + timestamp)
- SBES: raw distance reading (no position — caller must attach pose)
- MSIS: raw beam data (no position — caller must attach pose)
"""

from __future__ import annotations

import base64
import json
import logging
import math
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class OdometryData:
    """AUV pose from odometry."""
    timestamp: float
    x: float          # world X (meters)
    y: float          # world Y (meters)
    heading: float    # radians, 0=East, CCW positive


@dataclass
class MSISData:
    """Mechanically Scanned Imaging Sonar scan."""
    timestamp: float
    x: float          # sonar position world X
    y: float          # sonar position world Y
    heading: float    # AUV heading at time of scan
    beam_angle: float # beam direction in body frame (radians)
    range_max: float  # maximum range (meters)
    bin_values: np.ndarray  # intensity bins, shape (N,), uint8 or float


@dataclass
class SBESData:
    """Single-Beam Echo Sounder measurement."""
    timestamp: float
    x: float          # transducer position world X
    y: float          # transducer position world Y
    heading: float    # beam direction in world frame (radians)
    range_m: float    # measured range (meters)


# --------------- Raw intermediate types (no position info) --------------------

@dataclass
class RawSBESReading:
    """Raw SBES reading before pose is attached."""
    distance: float
    detected: bool


@dataclass
class RawMSISReading:
    """Raw MSIS beam reading before pose is attached."""
    beam_angle: float      # beam direction in body frame (radians)
    range_max: float
    bin_values: np.ndarray  # uint8


# --------------- Parsing functions --------------------------------------------

def parse_odometry(payload: bytes | dict) -> OdometryData | None:
    """Parse payload into OdometryData. Returns None on failure."""
    try:
        msg = payload if isinstance(payload, dict) else json.loads(payload)
        pos = msg["position"]
        ori = msg["orientation"]
        return OdometryData(
            timestamp=msg.get("timestamp", 0.0),
            x=pos["x"],
            y=pos["y"],
            heading=ori["yaw"],
        )
    except Exception:
        logger.debug("parse_odometry: failed to parse payload", exc_info=True)
        return None


def parse_sbes(payload: bytes | dict) -> RawSBESReading | None:
    """Parse payload into RawSBESReading. Returns None on failure or no detection."""
    try:
        msg = payload if isinstance(payload, dict) else json.loads(payload)
        detected = msg.get("detected", False)
        if not detected:
            return None
        return RawSBESReading(
            distance=msg.get("distance", 0.0),
            detected=True,
        )
    except Exception:
        logger.debug("parse_sbes: failed to parse payload", exc_info=True)
        return None


def _decode_beam_data(raw: object) -> np.ndarray | None:
    """Decode beam_data from various formats into a uint8 ndarray."""
    if isinstance(raw, str):
        # Base64 encoded string
        try:
            return np.frombuffer(base64.b64decode(raw), dtype=np.uint8)
        except Exception:
            return None
    elif isinstance(raw, list):
        return np.array(raw, dtype=np.uint8)
    elif isinstance(raw, dict):
        # nlohmann::json::binary() format
        bytes_field = raw.get("bytes")
        if bytes_field is not None:
            if isinstance(bytes_field, list):
                return np.array(bytes_field, dtype=np.uint8)
            elif isinstance(bytes_field, str):
                try:
                    return np.frombuffer(base64.b64decode(bytes_field), dtype=np.uint8)
                except Exception:
                    return None
        data_field = raw.get("data")
        if data_field is not None:
            return np.array(data_field, dtype=np.uint8)
    return None


def parse_msis(payload: bytes | dict) -> RawMSISReading | None:
    """Parse payload into RawMSISReading. Returns None on failure."""
    try:
        msg = payload if isinstance(payload, dict) else json.loads(payload)
        beam_angle = math.radians(msg["current_angle_deg"])
        range_max = msg.get("range_max", 50.0)
        beam_data_raw = msg.get("beam_data")
        if beam_data_raw is None:
            return None
        bin_values = _decode_beam_data(beam_data_raw)
        if bin_values is None or len(bin_values) == 0:
            return None
        bin_values = bin_values[::-1]  # simulator sends bin[0]=far; flip to bin[0]=near
        return RawMSISReading(
            beam_angle=beam_angle,
            range_max=range_max,
            bin_values=bin_values,
        )
    except Exception:
        logger.debug("parse_msis: failed to parse payload", exc_info=True)
        return None
