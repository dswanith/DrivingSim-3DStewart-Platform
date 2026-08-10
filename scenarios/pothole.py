"""
scenarios.pothole — Pothole impact scenario.

Vehicle travels straight and encounters a localized road depression.

Physics model:
    When a wheel drops into a pothole then rebounds, each wheel
    experiences an impulsive vertical displacement. The vehicle body
    responds with:
        - Heave (vertical)
        - Pitch (front-axle hits first)
        - Possibly roll (if pothole is offset laterally)

    The disturbance is modeled as two Gaussian pulses:
        1. Front axle impact (t = impact_time)
        2. Rear axle impact (t = impact_time + axle_separation / speed)

    Each pulse produces a vertical acceleration transient.

Parameters (realistic for a 30 cm deep, 60 cm wide pothole at 36 km/h):
    - Vertical impact: ~2-4g peak, 50ms duration
    - Pitch response: ±2-4° peak
"""
import math
import numpy as np
from .base import BaseScenario
from sil_sim.telemetry.base import TelemetryFrame


class PotholeScenario(BaseScenario):
    """
    Pothole encounter: localized vertical impact → heave + pitch on platform.
    """

    SCENARIO_NAME = "pothole"
    DEFAULT_DURATION_S = 12.0

    # Physical parameters
    IMPACT_TIME_S = 4.0           # Time of pothole encounter (s)
    AXLE_SEPARATION_M = 2.7       # Front-to-rear axle distance (m)
    POTHOLE_DEPTH_M = 0.12        # Depth of pothole (m)
    POTHOLE_WIDTH_M = 0.40        # Width of pothole (m)
    PEAK_IMPACT_G = 3.5           # Peak vertical acceleration (multiples of g)
    IMPACT_DURATION_S = 0.04      # Duration of impact pulse (seconds)
    PITCH_AMPLITUDE_RAD = math.radians(3.5)  # Peak pitch angle
    PITCH_DECAY_S = 0.8           # Time constant for pitch oscillation decay
    PITCH_FREQ_HZ = 1.8           # Body pitch natural frequency (Hz)

    def __init__(self, rate_hz: float = 60.0, duration_s: float = None, speed_ms: float = 10.0):
        super().__init__(rate_hz=rate_hz, duration_s=duration_s, speed_ms=speed_ms)
        self._pitch = 0.0
        self._pitch_rate = 0.0
        self._pos_x = 0.0

    def _on_reset(self):
        self._pitch = 0.0
        self._pitch_rate = 0.0
        self._pos_x = 0.0

    def _generate_frame(self, t: float, dt: float) -> TelemetryFrame:
        g = 9.81

        # Front axle impact time and rear axle impact time
        t_front = self.IMPACT_TIME_S
        t_rear = self.IMPACT_TIME_S + self.AXLE_SEPARATION_M / self.speed_ms

        # Vertical impulse from front axle (downward then upward rebound)
        # Model: negative Gaussian (drop) followed by positive Gaussian (rebound)
        sigma = self.IMPACT_DURATION_S / 2.0
        front_impulse = (
            -self.PEAK_IMPACT_G * g * self._gaussian(t, t_front, sigma)
            + self.PEAK_IMPACT_G * g * 0.6 * self._gaussian(t, t_front + 2*sigma, sigma * 1.5)
        )
        rear_impulse = (
            -self.PEAK_IMPACT_G * g * 0.7 * self._gaussian(t, t_rear, sigma)
            + self.PEAK_IMPACT_G * g * 0.4 * self._gaussian(t, t_rear + 2*sigma, sigma * 1.5)
        )
        az = front_impulse + rear_impulse

        # Pitch: front axle down → nose pitches down (negative pitch = nose down)
        # Excited by front impact, damped oscillation
        # Drive it with the differential axle loading
        pitch_excitation = -self.PITCH_AMPLITUDE_RAD * (
            self._gaussian(t, t_front, sigma * 2) -
            self._gaussian(t, t_rear, sigma * 2)
        )

        # Simple damped oscillator for pitch:
        # ẍ + 2ζω₀ẋ + ω₀²x = ω₀² * input
        omega0 = 2 * math.pi * self.PITCH_FREQ_HZ
        zeta = 0.35  # underdamped
        # Update pitch state via Euler integration
        pitch_ddot = (omega0**2 * (pitch_excitation - self._pitch)
                      - 2 * zeta * omega0 * self._pitch_rate)
        self._pitch_rate += pitch_ddot * dt
        self._pitch += self._pitch_rate * dt

        # Position
        self._pos_x += self.speed_ms * dt

        # Determine scenario phase
        if t < t_front - 1.0:
            phase = 'approach'
        elif t < t_front + 0.5:
            phase = 'front_impact'
        elif t < t_rear + 0.5:
            phase = 'rear_impact'
        elif t < t_rear + 3.0:
            phase = 'recovery'
        else:
            phase = 'cruise'

        # Update visualization state
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
