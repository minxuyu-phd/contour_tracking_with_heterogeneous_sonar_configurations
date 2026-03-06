#!/usr/bin/env python3
"""
PID Controller Base Classes and Utilities for SAM-AUV

This module provides:
- PIDParams: Data class for PID parameters
- PIDController: Basic PID controller with anti-windup
- AnglePIDController: PID controller for angular values (handles wraparound)
- DataRecorder: CSV data logging utility
"""

import time
import math
import csv
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Dict, Any


@dataclass
class PIDParams:
    """PID controller parameters"""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -float('inf')
    output_max: float = float('inf')
    integral_max: float = float('inf')
    # Integral separation threshold: when |error| < threshold, reduce integral action
    integral_separation_threshold: float = 0.0  # 0 = disabled


class PIDController:
    """
    Basic PID controller with anti-windup and output limiting.

    Uses "derivative on measurement" instead of "derivative on error" to
    prevent derivative kick when setpoint changes suddenly.
    """

    def __init__(self, params: PIDParams):
        self.params = params
        self.reset()

    def reset(self):
        """Reset controller state"""
        self._integral = 0.0
        self._prev_error = None
        self._prev_feedback = None
        self._prev_time = None

    def update(self, setpoint: float, feedback: float, current_time: float) -> float:
        """
        Compute PID output

        Args:
            setpoint: Desired value
            feedback: Current measured value
            current_time: Current timestamp (seconds)

        Returns:
            Control output (clamped to output limits)
        """
        error = setpoint - feedback

        # Initialize on first call
        if self._prev_time is None:
            self._prev_time = current_time
            self._prev_error = error
            self._prev_feedback = feedback
            return self._clamp_output(self.params.kp * error)

        # Calculate dt
        dt = current_time - self._prev_time
        if dt <= 0:
            return self._clamp_output(self.params.kp * error)

        # Proportional term
        p_term = self.params.kp * error

        # Integral term with anti-windup
        self._integral += error * dt

        self._integral = max(-self.params.integral_max,
                            min(self.params.integral_max, self._integral))
        i_term = self.params.ki * self._integral

        # Derivative term - use derivative on measurement (not error)
        # This prevents "derivative kick" when setpoint changes suddenly
        # d/dt(error) = d/dt(setpoint - feedback) = -d/dt(feedback) when setpoint is constant
        # So we use negative derivative of feedback
        d_feedback = (feedback - self._prev_feedback) / dt
        d_term = -self.params.kd * d_feedback

        # Update state
        self._prev_error = error
        self._prev_feedback = feedback
        self._prev_time = current_time

        # Calculate and clamp output
        output = p_term + i_term + d_term
        return self._clamp_output(output)

    def _clamp_output(self, output: float) -> float:
        """Clamp output to configured limits"""
        return max(self.params.output_min, min(self.params.output_max, output))

    @property
    def integral(self) -> float:
        """Current integral value"""
        return self._integral

    @property
    def last_error(self) -> Optional[float]:
        """Last computed error"""
        return self._prev_error


class AnglePIDController(PIDController):
    """
    PID controller for angular values with wraparound handling.
    Angles are normalized to [-pi, pi].

    Uses "derivative on measurement" instead of "derivative on error" to
    prevent derivative kick when setpoint changes suddenly.
    """

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def angle_error(self, setpoint: float, feedback: float) -> float:
        """
        Calculate angular error handling wraparound
        Returns shortest path from feedback to setpoint
        """
        error = setpoint - feedback
        return self.normalize_angle(error)

    def angle_diff(self, angle1: float, angle2: float) -> float:
        """
        Calculate angular difference (angle1 - angle2) handling wraparound
        Returns value in [-pi, pi]
        """
        diff = angle1 - angle2
        return self.normalize_angle(diff)

    def update(self, setpoint: float, feedback: float, current_time: float) -> float:
        """
        Compute PID output for angular control
        """
        # Normalize both angles
        setpoint = self.normalize_angle(setpoint)
        feedback = self.normalize_angle(feedback)

        # Calculate error with wraparound handling
        error = self.angle_error(setpoint, feedback)

        # Initialize on first call
        if self._prev_time is None:
            self._prev_time = current_time
            self._prev_error = error
            self._prev_feedback = feedback
            return self._clamp_output(self.params.kp * error)

        # Calculate dt
        dt = current_time - self._prev_time
        if dt <= 0:
            return self._clamp_output(self.params.kp * error)

        # Proportional term
        p_term = self.params.kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = max(-self.params.integral_max,
                            min(self.params.integral_max, self._integral))
        i_term = self.params.ki * self._integral

        # Derivative term - use derivative on measurement (not error)
        # This prevents "derivative kick" when setpoint changes suddenly
        # Use angle_diff to handle wraparound correctly
        d_feedback = self.angle_diff(feedback, self._prev_feedback) / dt
        d_term = -self.params.kd * d_feedback

        # Update state
        self._prev_error = error
        self._prev_feedback = feedback
        self._prev_time = current_time

        # Calculate and clamp output
        output = p_term + i_term + d_term
        return self._clamp_output(output)


class DataRecorder:
    """
    CSV data recorder for controller logging
    """

    def __init__(self, controller_type: str, data_dir: str = None):
        """
        Initialize data recorder

        Args:
            controller_type: Type of controller (speed, heading, pitch)
            data_dir: Directory to save data files
        """
        self.controller_type = controller_type
        self.data_dir = data_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'data'
        )

        # Ensure data directory exists
        os.makedirs(self.data_dir, exist_ok=True)

        self._records = []
        self._start_time = None
        self._extra_fields = []

    def set_extra_fields(self, fields: list):
        """Set additional fields to record"""
        self._extra_fields = fields

    def record(self, setpoint: float, feedback: float, error: float,
               output: float, extra: Dict[str, Any] = None):
        """
        Record a data point

        Args:
            setpoint: Target value
            feedback: Measured value
            error: Control error
            output: Controller output
            extra: Additional data fields
        """
        timestamp = time.time()
        if self._start_time is None:
            self._start_time = timestamp

        record = {
            'timestamp': timestamp,
            'elapsed': timestamp - self._start_time,
            'setpoint': setpoint,
            'feedback': feedback,
            'error': error,
            'output': output,
        }

        if extra:
            record.update(extra)

        self._records.append(record)

    def save(self) -> str:
        """
        Save recorded data to CSV file

        Returns:
            Path to saved file
        """
        if not self._records:
            return None

        # Generate filename with timestamp
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"{self.controller_type}_{timestamp_str}.csv"
        filepath = os.path.join(self.data_dir, filename)

        # Determine all fields from first record
        fieldnames = ['timestamp', 'elapsed', 'setpoint', 'feedback', 'error', 'output']

        # Add extra fields from records
        if self._records:
            for key in self._records[0].keys():
                if key not in fieldnames:
                    fieldnames.append(key)

        # Write CSV
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._records)

        return filepath

    @property
    def record_count(self) -> int:
        """Number of recorded data points"""
        return len(self._records)

    def get_statistics(self) -> Dict[str, float]:
        """Calculate statistics from recorded data"""
        if not self._records:
            return {}

        errors = [r['error'] for r in self._records]

        # Use last 20% of data for steady state calculation
        steady_start = int(len(errors) * 0.8)
        steady_errors = errors[steady_start:] if steady_start < len(errors) else errors

        return {
            'max_error': max(abs(e) for e in errors),
            'mean_error': sum(errors) / len(errors),
            'steady_state_error': sum(abs(e) for e in steady_errors) / len(steady_errors),
            'total_records': len(self._records),
        }


