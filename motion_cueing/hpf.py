"""
sil_sim.motion_cueing.hpf — Discrete high-pass filter for motion cueing.

Extracts transient (high-frequency) components from vehicle acceleration.
The HPF removes sustained accelerations that would require infinite actuator
travel, passing only the onset/offset transients that the platform can
physically reproduce.

Implements a 2nd-order Butterworth high-pass filter using bilinear transform.

Output class: EXACT — pure signal processing math.
"""

import math
import numpy as np
from typing import Dict, Any


class HighPassFilter:
    """
    Discrete 2nd-order Butterworth high-pass filter.

    Uses the bilinear (Tustin) transform to convert the continuous-time
    Butterworth prototype to discrete-time at the configured sample rate.

    Applied independently to each axis.
    """

    def __init__(self, cutoff_hz: float, sample_rate_hz: float, order: int = 2):
        """
        Args:
            cutoff_hz: Cutoff frequency (Hz). Signals below this are attenuated.
            sample_rate_hz: Sample rate (Hz).
            order: Filter order (only 2 is implemented).
        """
        if order != 2:
            raise NotImplementedError(
                "Only 2nd-order HPF is implemented, got order={}".format(order)
            )

        self.cutoff_hz = cutoff_hz
        self.sample_rate_hz = sample_rate_hz
        self.dt = 1.0 / sample_rate_hz

        # Bilinear transform: pre-warp the cutoff frequency
        wc = 2.0 * math.pi * cutoff_hz
        T = self.dt
        wc_d = 2.0 / T * math.tan(wc * T / 2.0)

        # 2nd-order Butterworth denominator polynomial:
        # s² + √2·wc·s + wc²
        # Apply bilinear transform s = (2/T)(z-1)/(z+1):
        K = 2.0 / T
        sqrt2 = math.sqrt(2.0)

        # Coefficients for the transfer function H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) /
        #                                               (1 + a1·z⁻¹ + a2·z⁻²)
        denom = K * K + sqrt2 * wc_d * K + wc_d * wc_d
        self._b0 = K * K / denom
        self._b1 = -2.0 * K * K / denom
        self._b2 = K * K / denom
        self._a1 = (2.0 * wc_d * wc_d - 2.0 * K * K) / denom
        self._a2 = (K * K - sqrt2 * wc_d * K + wc_d * wc_d) / denom

        # State: per-axis delay buffers [x[n-1], x[n-2]] and [y[n-1], y[n-2]]
        # Initialized for 3 axes (X, Y, Z)
        self._x_hist = np.zeros((3, 2))  # [axis, delay]
        self._y_hist = np.zeros((3, 2))

    def reset(self) -> None:
        """Reset filter state to zero."""
        self._x_hist[:] = 0.0
        self._y_hist[:] = 0.0

    def update(self, x: np.ndarray) -> np.ndarray:
        """
        Apply the HPF to a 3-axis input sample.

        Args:
            x: (3,) array — input signal [X, Y, Z].

        Returns:
            (3,) array — filtered output. EXACT.
        """
        y = np.zeros(3)
        for axis in range(3):
            # Direct Form II Transposed
            y[axis] = (
                self._b0 * x[axis]
                + self._b1 * self._x_hist[axis, 0]
                + self._b2 * self._x_hist[axis, 1]
                - self._a1 * self._y_hist[axis, 0]
                - self._a2 * self._y_hist[axis, 1]
            )
            # Shift delay lines
            self._x_hist[axis, 1] = self._x_hist[axis, 0]
            self._x_hist[axis, 0] = x[axis]
            self._y_hist[axis, 1] = self._y_hist[axis, 0]
            self._y_hist[axis, 0] = y[axis]

        return y


def create_hpf_from_config(config: Dict[str, Any]) -> HighPassFilter:
    """
    Create an HPF from MCA config.

    Args:
        config: Validated MCA config dict.

    Returns:
        HighPassFilter instance.
    """
    return HighPassFilter(
        cutoff_hz=config['hpf']['cutoff_freq_hz'],
        sample_rate_hz=config['input_rate_hz'],
        order=config['hpf']['order'],
    )
