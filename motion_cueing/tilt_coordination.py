"""
sil_sim.motion_cueing.tilt_coordination — Tilt coordination for sustained accelerations.

Converts sustained (low-frequency) lateral/longitudinal acceleration into
platform tilt, so the occupant perceives a gravity-component cue instead
of requiring infinite translational travel.

The tilt angle is rate-limited to stay below the human vestibular
motion-detection threshold (~3°/s).

Output class: EXACT — pure signal processing / math.
"""

import math
import numpy as np
from typing import Dict, Any


class TiltCoordination:
    """
    Tilt coordination module.

    Maps sustained acceleration to tilt angle:
        target_tilt = gain × sustained_accel
        tilt_rate = clamp(dtilt/dt, ±rate_limit)
        tilt = clamp(tilt, ±max_tilt)

    Applied to:
        - X acceleration → pitch (nose up under forward accel)
        - Y acceleration → roll (lean into turn)
    """

    def __init__(
        self,
        gain: float,
        rate_limit_rad_s: float,
        max_tilt_rad: float,
        sample_rate_hz: float,
    ):
        """
        Args:
            gain: rad per (m/s²) — how much tilt per unit sustained accel.
            rate_limit_rad_s: Maximum tilt rate (rad/s).
            max_tilt_rad: Maximum tilt angle (rad).
            sample_rate_hz: Update rate (Hz).
        """
        self.gain = gain
        self.rate_limit = rate_limit_rad_s
        self.max_tilt = max_tilt_rad
        self.dt = 1.0 / sample_rate_hz

        # Current tilt state [pitch_from_ax, roll_from_ay]
        self._tilt = np.zeros(2)

    def reset(self) -> None:
        """Reset tilt state to zero."""
        self._tilt[:] = 0.0

    def update(self, sustained_accel_xy: np.ndarray) -> np.ndarray:
        """
        Compute tilt coordination output from sustained (low-freq) acceleration.

        Args:
            sustained_accel_xy: (2,) array [ax_sustained, ay_sustained] (m/s²).
                This should be the LOW-frequency component (total minus HPF output).

        Returns:
            (2,) array [pitch_tilt, roll_tilt] in radians. EXACT.
        """
        # Target tilt = gain × acceleration
        # X accel → pitch tilt (positive ax → negative pitch = nose up)
        # Y accel → roll tilt (positive ay → positive roll = lean left/into turn)
        target_pitch = -self.gain * sustained_accel_xy[0]
        target_roll = self.gain * sustained_accel_xy[1]
        target = np.array([target_pitch, target_roll])

        # Clamp to max tilt
        target = np.clip(target, -self.max_tilt, self.max_tilt)

        # Rate-limit
        delta = target - self._tilt
        max_delta = self.rate_limit * self.dt
        delta = np.clip(delta, -max_delta, max_delta)

        self._tilt += delta

        return self._tilt.copy()


def create_tilt_from_config(config: Dict[str, Any]) -> TiltCoordination:
    """
    Create TiltCoordination from MCA config.

    Args:
        config: Validated MCA config dict.

    Returns:
        TiltCoordination instance.
    """
    tc = config['tilt_coordination']
    return TiltCoordination(
        gain=tc['gain'],
        rate_limit_rad_s=tc['rate_limit_rad_s'],
        max_tilt_rad=tc['max_tilt_rad'],
        sample_rate_hz=config['input_rate_hz'],
    )
