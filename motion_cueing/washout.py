"""
sil_sim.motion_cueing.washout — Washout filter for return-to-neutral.

Applies exponential decay to the platform's commanded translation and rotation,
returning it smoothly to the neutral pose after transient motion cues.

This prevents the platform from drifting to its stroke limits during
extended driving.

Output class: EXACT — pure signal processing.
"""

import numpy as np
from typing import Dict, Any


class WashoutFilter:
    """
    First-order exponential washout filter.

    y[n] = α · y[n-1] + (1 - α) · x[n]
    where α = exp(-dt / τ)

    As τ → ∞, the filter passes through (no washout).
    As τ → 0, the filter snaps to zero immediately.

    Applied independently to each axis of the platform pose.
    """

    def __init__(self, time_constant_s: float, sample_rate_hz: float, channels: int = 6):
        """
        Args:
            time_constant_s: Washout time constant τ (seconds).
            sample_rate_hz: Update rate (Hz).
            channels: Number of independent channels (6 for full pose).
        """
        import math
        self.tau = time_constant_s
        self.dt = 1.0 / sample_rate_hz
        self.alpha = math.exp(-self.dt / self.tau) if self.tau > 0 else 0.0
        self.channels = channels

        # State: previous output
        self._y_prev = np.zeros(channels)

    def reset(self) -> None:
        """Reset filter state to zero (neutral pose)."""
        self._y_prev[:] = 0.0

    def update(self, x: np.ndarray) -> np.ndarray:
        """
        Apply washout filter to a pose or pose-component vector.

        The washout filter acts as a HIGH-PASS filter on the pose:
        it passes transient changes but decays sustained offsets back to zero.

        Implemented as: y[n] = α · y[n-1] + (1 - α) · x[n]
        Then the washout output is: x[n] - y[n] (the high-passed part).

        Actually, for motion cueing, the washout is typically applied as
        a simple decay on the accumulated displacement:
            pose_out[n] = α · pose_out[n-1] + (1 - α) · 0
        meaning the output decays toward zero. The input at each step
        is an incremental change added to the decaying state.

        For our pipeline, the MCA accumulates the pose, and washout decays it:
            y[n] = α · (y[n-1] + delta_x[n])

        Args:
            x: (channels,) array — current pose increment or target.

        Returns:
            (channels,) array — washout-filtered output. EXACT.
        """
        # Decay previous output and add new input
        self._y_prev = self.alpha * (self._y_prev + x)
        return self._y_prev.copy()


def create_washout_from_config(config: Dict[str, Any]) -> WashoutFilter:
    """
    Create a single WashoutFilter from MCA config (legacy interface).

    This creates one shared filter for all 6 DOF channels. For correct
    motion cueing, prefer create_split_washout_from_config() which uses
    independent time constants for translation vs rotation.

    Args:
        config: Validated MCA config dict.

    Returns:
        WashoutFilter instance (6-channel).
    """
    # Support both legacy flat and new nested washout config
    washout_cfg = config['washout']
    if isinstance(washout_cfg.get('time_constant_s', None), (int, float)):
        tau = washout_cfg['time_constant_s']
    else:
        # Fallback: use translation time constant
        tau = washout_cfg.get('translation', {}).get('time_constant_s', 2.0)

    return WashoutFilter(
        time_constant_s=tau,
        sample_rate_hz=config['input_rate_hz'],
        channels=6,  # x, y, z, roll, pitch, yaw
    )


def create_split_washout_from_config(
    config: Dict[str, Any],
) -> tuple:
    """
    Create separate translation and rotation WashoutFilters from MCA config.

    Uses independent time constants so that:
    - Translational washout decays fast (limited actuator stroke)
    - Rotational washout decays slower (preserves tilt-coordination cues
      during sustained cornering)

    This prevents the single-filter problem where tilt-coordination angles
    are washed back to neutral by the fast translational time constant
    even while the car is still turning.

    Both filters are first-order exponential washout (see WashoutFilter
    docstring). Filter order is pinned at 1 — if higher-order washout is
    needed (cleaner roll-off at the cost of more phase lag), the
    WashoutFilter class itself would need extending.

    Args:
        config: Validated MCA config dict.

    Returns:
        Tuple of (washout_translation, washout_rotation), each a
        WashoutFilter with 3 channels.
    """
    washout_cfg = config['washout']
    rate_hz = config['input_rate_hz']

    # Support both legacy flat config and new nested config
    if 'translation' in washout_cfg and 'rotation' in washout_cfg:
        tau_trans = washout_cfg['translation']['time_constant_s']
        tau_rot = washout_cfg['rotation']['time_constant_s']
    else:
        # Legacy: single time constant — use same value for both
        tau_trans = washout_cfg['time_constant_s']
        tau_rot = washout_cfg['time_constant_s']

    washout_trans = WashoutFilter(
        time_constant_s=tau_trans,
        sample_rate_hz=rate_hz,
        channels=3,  # x, y, z
    )
    washout_rot = WashoutFilter(
        time_constant_s=tau_rot,
        sample_rate_hz=rate_hz,
        channels=3,  # roll, pitch, yaw
    )

    return washout_trans, washout_rot
