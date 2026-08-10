"""
scenarios.speed_hump — Speed hump (speed bump) scenario.

Vehicle travels over a raised speed hump.

Physics model:
    The hump profile is a raised cosine (smooth bump) with realistic
    dimensions for a residential speed hump:
        - Height: 75 mm
        - Width: 900 mm (half-width 450mm along travel direction)

    The vehicle body responds with:
        1. Front axle rises → nose pitches up
        2. Both axles on hump → heave up
        3. Rear axle still on hump → nose pitches down
        4. Vehicle clears hump → settling oscillation

    The excitation is split between front and rear axle, delayed by
    axle separation / vehicle speed.
"""
import math
import numpy as np
from .base import BaseScenario
from sil_sim.telemetry.base import TelemetryFrame


class SpeedHumpScenario(BaseScenario):
    """
    Speed hump: realistic heave + pitch response as both axles traverse the bump.
    """

    SCENARIO_NAME = "speed_hump"
    DEFAULT_DURATION_S = 15.0

    # Hump physical parameters
    HUMP_HEIGHT_M = 0.075         # 75 mm residential hump
    HUMP_HALF_WIDTH_M = 0.45      # 900 mm total width
    HUMP_CENTER_TIME_S = 4.0      # Time front axle reaches hump center
    AXLE_SEPARATION_M = 2.7       # Front-to-rear axle distance

    # Vehicle body response parameters
    BODY_PITCH_FREQ_HZ = 1.5      # Body pitch natural frequency
    BODY_HEAVE_FREQ_HZ = 1.2      # Body heave natural frequency
    BODY_ZETA = 0.30              # Damping ratio (slightly underdamped)

    def __init__(self, rate_hz: float = 60.0, duration_s: float = None, speed_ms: float = 8.0):
        # Default 8 m/s ≈ 29 km/h (realistic approach speed for speed hump)
        super().__init__(rate_hz=rate_hz, duration_s=duration_s, speed_ms=speed_ms)
        self._pitch = 0.0
        self._pitch_rate = 0.0
        self._heave = 0.0
        self._heave_rate = 0.0
        self._pos_x = 0.0

    def _on_reset(self):
        self._pitch = 0.0
        self._pitch_rate = 0.0
        self._heave = 0.0
        self._heave_rate = 0.0
        self._pos_x = 0.0

    def _hump_height(self, x: float) -> float:
        """Raised cosine hump profile as function of position x (m from hump center)."""
        if abs(x) > self.HUMP_HALF_WIDTH_M:
            return 0.0
        return self.HUMP_HEIGHT_M * 0.5 * (1.0 + math.cos(math.pi * x / self.HUMP_HALF_WIDTH_M))

    def _hump_slope(self, x: float) -> float:
        """Derivative dh/dx at position x (radians approximation for small angles)."""
        if abs(x) >= self.HUMP_HALF_WIDTH_M:
            return 0.0
        return -self.HUMP_HEIGHT_M * math.pi / (2.0 * self.HUMP_HALF_WIDTH_M) * math.sin(
            math.pi * x / self.HUMP_HALF_WIDTH_M
        )

    def _generate_frame(self, t: float, dt: float) -> TelemetryFrame:
        g = 9.81

        # Position of front and rear axles relative to hump center
        # x_front = 0 when front axle is at hump center
        x_front = (t - self.HUMP_CENTER_TIME_S) * self.speed_ms
        x_rear = x_front - self.AXLE_SEPARATION_M

        # Height of each axle on the hump
        h_front = self._hump_height(x_front)
        h_rear = self._hump_height(x_rear)

        # Body heave = average of front and rear axle heights
        target_heave = (h_front + h_rear) / 2.0

        # Body pitch = atan((h_front - h_rear) / axle_separation)
        # Positive pitch = nose up
        target_pitch = math.atan2(h_front - h_rear, self.AXLE_SEPARATION_M)

        # Drive heave and pitch through damped second-order systems
        omega_h = 2 * math.pi * self.BODY_HEAVE_FREQ_HZ
        omega_p = 2 * math.pi * self.BODY_PITCH_FREQ_HZ

        heave_ddot = (omega_h**2 * (target_heave - self._heave)
                      - 2 * self.BODY_ZETA * omega_h * self._heave_rate)
        pitch_ddot = (omega_p**2 * (target_pitch - self._pitch)
                      - 2 * self.BODY_ZETA * omega_p * self._pitch_rate)

        self._heave_rate += heave_ddot * dt
        self._pitch_rate += pitch_ddot * dt
        self._heave += self._heave_rate * dt
        self._pitch += self._pitch_rate * dt

        # Vertical acceleration (second derivative of heave) — this is the stimulus
        az = heave_ddot  # m/s² — what the MCA sees

        self._pos_x += self.speed_ms * dt

        # Determine phase
        if x_front < -self.HUMP_HALF_WIDTH_M:
            phase = 'approach'
        elif x_rear < 0:
            phase = 'front_on_hump'
        elif x_front > 0:
            phase = 'rear_on_hump'
        elif x_rear > self.HUMP_HALF_WIDTH_M:
            phase = 'recovery'
        else:
            phase = 'crest'

        # Update state
        self._state.x = self._pos_x
        self._state.speed = self.speed_ms
        self._state.az = az
        self._state.pitch = self._pitch
        self._state.simulation_time = t
        self._state.frame_index = self._frame_index
        self._state.scenario_phase = phase

        return self._make_frame(
            t=t,
            ax=0.0, ay=0.0, az=az,
            pitch=self._pitch,
            pitch_rate=self._pitch_rate,
            vx=self.speed_ms,
        )
