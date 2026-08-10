"""
scenarios.base — Base class for all synthetic driving scenarios.

Each scenario is a generator that yields TelemetryFrame objects at a
fixed rate. The scenario produces physically plausible vehicle state
signals that the MCA pipeline then converts to platform commands.

All values in SI units. Automotive axes: X-forward, Y-left, Z-up.
"""
import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Iterator, Optional
import sys
import os
import numpy as np

# Add vehicle_sim to path so we can import sil_sim
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vehicle_sim'))
from sil_sim.telemetry.base import TelemetryFrame, TelemetrySource


@dataclass
class ScenarioState:
    """
    Current vehicle state for visualization.
    All values in SI units.
    """
    # Position (metres, world frame)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # Velocity (m/s, body frame)
    speed: float = 0.0        # longitudinal speed

    # Accelerations (m/s², body frame)
    ax: float = 0.0           # longitudinal (X-forward)
    ay: float = 0.0           # lateral (Y-left)
    az: float = 0.0           # vertical disturbance (Z-up), 0 = steady level

    # Orientation (radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0

    # Scenario metadata
    scenario_name: str = ''
    scenario_phase: str = ''
    simulation_time: float = 0.0
    frame_index: int = 0


class BaseScenario(TelemetrySource, ABC):
    """
    Abstract base class for synthetic driving scenarios.

    Subclasses implement _generate_frame() to produce vehicle state
    at each timestep. The base class handles timing and state integration.
    """

    SCENARIO_NAME = "base"
    DEFAULT_DURATION_S = 15.0
    DEFAULT_RATE_HZ = 60.0

    def __init__(
        self,
        rate_hz: float = DEFAULT_RATE_HZ,
        duration_s: Optional[float] = None,
        speed_ms: float = 10.0,   # nominal vehicle speed (m/s) ≈ 36 km/h
    ):
        """
        Args:
            rate_hz: Telemetry output rate (Hz). Default 60 Hz.
            duration_s: Scenario duration (seconds). None = class default.
            speed_ms: Nominal vehicle forward speed (m/s).
        """
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.duration_s = duration_s if duration_s is not None else self.DEFAULT_DURATION_S
        self.speed_ms = speed_ms

        self._sim_time: float = 0.0
        self._frame_index: int = 0
        self._state = ScenarioState(scenario_name=self.SCENARIO_NAME)

    @property
    def source_name(self) -> str:
        return self.SCENARIO_NAME

    def is_connected(self) -> bool:
        return True

    def get_frame(self) -> Optional[TelemetryFrame]:
        return self.step()

    @property
    def simulation_time(self) -> float:
        return self._sim_time

    @property
    def frame_index(self) -> int:
        return self._frame_index

    @property
    def state(self) -> ScenarioState:
        return self._state

    def is_complete(self) -> bool:
        return self._sim_time >= self.duration_s

    def reset(self):
        """Reset scenario to initial state."""
        self._sim_time = 0.0
        self._frame_index = 0
        self._state = ScenarioState(scenario_name=self.SCENARIO_NAME)
        self._on_reset()

    def _on_reset(self):
        """Override for subclass-specific reset logic."""
        pass

    @abstractmethod
    def _generate_frame(self, t: float, dt: float) -> TelemetryFrame:
        """
        Generate one telemetry frame at simulation time t.

        Args:
            t: Current simulation time (seconds).
            dt: Time step (seconds).

        Returns:
            TelemetryFrame with all fields in SI units.
        """
        ...

    def step(self) -> Optional[TelemetryFrame]:
        """
        Advance scenario by one timestep and return a TelemetryFrame.
        Returns None when scenario is complete.
        """
        if self.is_complete():
            return None

        frame = self._generate_frame(self._sim_time, self.dt)
        self._sim_time += self.dt
        self._frame_index += 1
        return frame

    def __iter__(self) -> Iterator[TelemetryFrame]:
        """Iterate over all frames in the scenario."""
        self.reset()
        while not self.is_complete():
            frame = self.step()
            if frame is not None:
                yield frame

    @staticmethod
    def _make_frame(
        t: float,
        ax: float = 0.0, ay: float = 0.0, az: float = 0.0,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        roll_rate: float = 0.0, pitch_rate: float = 0.0, yaw_rate: float = 0.0,
        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
    ) -> TelemetryFrame:
        """
        Helper to construct a TelemetryFrame.

        UNIT CONVENTION (explicit):
            ax, ay, az:  m/s² — vehicle body acceleration (gravity NOT included)
            roll/pitch/yaw: radians — vehicle orientation
            roll_rate/pitch_rate/yaw_rate: rad/s — angular velocity
            vx/vy/vz:   m/s  — velocity in body frame

        Args:
            t: Simulation timestamp (s)
            ax: Longitudinal acceleration (m/s²), positive = forward
            ay: Lateral acceleration (m/s²), positive = left
            az: Vertical acceleration (m/s²), disturbance only (0 = level)
            roll, pitch, yaw: Orientation (radians)
            roll_rate, pitch_rate, yaw_rate: Angular velocity (rad/s)
            vx, vy, vz: Velocity (m/s)
        """
        return TelemetryFrame(
            timestamp=t,
            accel_xyz=np.array([ax, ay, az], dtype=np.float64),
            ang_vel_xyz=np.array([roll_rate, pitch_rate, yaw_rate], dtype=np.float64),
            orientation_rpy=np.array([roll, pitch, yaw], dtype=np.float64),
            velocity_xyz=np.array([vx, vy, vz], dtype=np.float64),
        )

    @staticmethod
    def _smooth_step(t: float, t0: float, t1: float) -> float:
        """Smooth Hermite step function from 0→1 over [t0, t1]."""
        if t <= t0:
            return 0.0
        if t >= t1:
            return 1.0
        x = (t - t0) / (t1 - t0)
        return x * x * (3 - 2 * x)

    @staticmethod
    def _gaussian(t: float, center: float, sigma: float) -> float:
        """Unit Gaussian pulse centred at center with std deviation sigma."""
        return math.exp(-0.5 * ((t - center) / sigma) ** 2)

    @staticmethod
    def _raised_cosine(t: float, t_start: float, width: float) -> float:
        """Raised cosine bump: 0→1→0 over [t_start, t_start+width]."""
        if t < t_start or t > t_start + width:
            return 0.0
        phase = math.pi * (t - t_start) / width
        return 0.5 * (1.0 - math.cos(2.0 * phase))
