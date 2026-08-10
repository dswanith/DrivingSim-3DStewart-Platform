"""
scenarios.straight_road — Straight road driving scenario.

Vehicle drives at approximately constant speed.
The Stewart platform should remain approximately neutral except for
realistic filtering/washout behaviour.

Physics:
    - Constant forward speed (default 10 m/s ≈ 36 km/h)
    - Small random road noise on vertical axis (±0.05g RMS)
    - Slight yaw wander (±0.5°)
    - No significant longitudinal or lateral acceleration
"""
import math
import numpy as np
from .base import BaseScenario
from sil_sim.telemetry.base import TelemetryFrame


class StraightRoadScenario(BaseScenario):
    """
    Straight road at constant speed.
    Generates minimal platform motion (near-neutral).
    """

    SCENARIO_NAME = "straight_road"
    DEFAULT_DURATION_S = 20.0

    def __init__(self, rate_hz: float = 60.0, duration_s: float = None, speed_ms: float = 10.0):
        super().__init__(rate_hz=rate_hz, duration_s=duration_s, speed_ms=speed_ms)
        self._rng = np.random.default_rng(seed=42)  # deterministic noise
        # Pre-generate noise arrays for the maximum expected duration
        n_samples = int(self.duration_s * self.rate_hz) + 100
        self._road_noise_z = self._rng.normal(0.0, 0.05 * 9.81, n_samples)   # ±0.05g vertical
        self._road_noise_y = self._rng.normal(0.0, 0.01 * 9.81, n_samples)   # tiny lateral
        self._yaw_wander_rate = 0.0   # accumulates
        self._yaw = 0.0
        self._pos_x = 0.0

    def _on_reset(self):
        self._rng = np.random.default_rng(seed=42)
        n_samples = int(self.duration_s * self.rate_hz) + 100
        self._road_noise_z = self._rng.normal(0.0, 0.05 * 9.81, n_samples)
        self._road_noise_y = self._rng.normal(0.0, 0.01 * 9.81, n_samples)
        self._yaw_wander_rate = 0.0
        self._yaw = 0.0
        self._pos_x = 0.0

    def _generate_frame(self, t: float, dt: float) -> TelemetryFrame:
        idx = int(t * self.rate_hz)
        idx = min(idx, len(self._road_noise_z) - 1)

        # Road surface noise (band-limited to ≈2–8 Hz)
        az = self._road_noise_z[idx]
        ay = self._road_noise_y[idx]

        # Very slow yaw wander
        wander_rate = 0.003 * math.sin(2 * math.pi * 0.1 * t)  # rad/s
        self._yaw += wander_rate * dt
        self._pos_x += self.speed_ms * dt

        # Update state for visualization
        self._state.x = self._pos_x
        self._state.y = 0.0
        self._state.speed = self.speed_ms
        self._state.ax = 0.0
        self._state.ay = ay
        self._state.az = az
        self._state.roll = 0.0
        self._state.pitch = 0.0
        self._state.yaw = self._yaw
        self._state.yaw_rate = wander_rate
        self._state.simulation_time = t
        self._state.frame_index = self._frame_index
        self._state.scenario_phase = 'cruising'

        return self._make_frame(
            t=t,
            ax=0.0, ay=ay, az=az,
            roll=0.0, pitch=0.0, yaw=self._yaw,
            yaw_rate=wander_rate,
            vx=self.speed_ms,
        )
