"""
scenarios.lane_change — Lane change / S-curve scenario.

Vehicle performs a smooth left-right lane change maneuver.

Physics model:
    The lateral trajectory follows a sinusoidal profile:
        y(t) = A * sin(2π * f * t) over the maneuver window

    This produces:
        ay = -A * (2π * f)² * sin(2π * f * t)  (centripetal lateral accel)
        yaw_rate: derivative of heading angle

    The S-curve consists of:
        1. Straight approach
        2. Left lane change (or right first, depending on config)
        3. Return to center (S-shape)
        4. Straight exit

Realistic parameters for a motorway lane change at 100 km/h:
    - Lane width: 3.5 m (lateral displacement)
    - Maneuver time: 4 s
    - Peak lateral accel: ~0.3g
"""
import math
from .base import BaseScenario
from sil_sim.telemetry.base import TelemetryFrame


class LaneChangeScenario(BaseScenario):
    """
    Lane change / S-curve scenario. Produces sinusoidal lateral cues.
    """

    SCENARIO_NAME = "lane_change"
    DEFAULT_DURATION_S = 20.0

    # Maneuver parameters
    LANE_WIDTH_M = 3.5            # Lateral displacement per lane change
    MANEUVER_START_S = 4.0        # When first lane change starts
    MANEUVER_DURATION_S = 4.0     # Duration of each individual lane change
    NUM_LANE_CHANGES = 2          # Number of alternating lane changes (S-curve)
    FIRST_DIRECTION = 'left'      # Direction of first lane change

    # Body roll
    ROLL_STATIC_GAIN = 0.07       # rad per m/s² of lateral accel
    ROLL_FREQ_HZ = 2.0
    ROLL_ZETA = 0.45

    def __init__(
        self,
        rate_hz: float = 60.0,
        duration_s: float = None,
        speed_ms: float = 20.0,   # 72 km/h motorway speed
        direction: str = 'left',
    ):
        super().__init__(rate_hz=rate_hz, duration_s=duration_s, speed_ms=speed_ms)
        self.FIRST_DIRECTION = direction
        self._first_sign = +1.0 if direction == 'left' else -1.0

        self._roll = 0.0
        self._roll_rate = 0.0
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._vy_body = 0.0  # lateral velocity in body frame

    def _on_reset(self):
        self._roll = 0.0
        self._roll_rate = 0.0
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._vy_body = 0.0

    def _lateral_accel(self, t: float) -> float:
        """
        Compute lateral acceleration at time t.
        Each lane change is a half-sinusoid of lateral accel:
            ay(t) = A_peak * sin(π * (t - t_start) / T_maneuver)
        Peak lateral accel from kinematics:
            y(t) = A_lane * 0.5 * (1 - cos(π * t / T))
            ẏ(t) = A_lane * π / (2T) * sin(π * t / T)
            ÿ(t) = A_lane * π² / (2T²) * cos(π * t / T)  ← lateral accel
        """
        ay = 0.0
        for lc in range(self.NUM_LANE_CHANGES):
            t_start = self.MANEUVER_START_S + lc * self.MANEUVER_DURATION_S
            t_end = t_start + self.MANEUVER_DURATION_S
            T = self.MANEUVER_DURATION_S

            # Direction alternates: first is +sign, second is -sign
            sign = self._first_sign * ((-1.0) ** lc)

            if t_start <= t <= t_end:
                phase = (t - t_start) / T
                # ÿ = (A / T) * π * sin(π * phase) [from raised cosine profile]
                # where A is lane width
                ay_peak = self.LANE_WIDTH_M * math.pi**2 / (2.0 * T**2)
                # Actually: for y = A/2*(1-cos(πt/T)):
                #   ẏ = Aπ/(2T)*sin(πt/T)  → peak = Aπ/(2T)
                #   ÿ = Aπ²/(2T²)*cos(πt/T) → peak = Aπ²/(2T²)
                ay_contribution = -sign * ay_peak * math.cos(math.pi * phase)
                ay += ay_contribution

        return ay

    def _generate_frame(self, t: float, dt: float) -> TelemetryFrame:
        ay = self._lateral_accel(t)

        # Yaw rate from lateral acceleration and speed
        # ay ≈ v * yaw_rate (for small angles)
        self._yaw_rate = -ay / max(self.speed_ms, 1.0)  # rad/s
        self._yaw += self._yaw_rate * dt

        # Body roll
        target_roll = self.ROLL_STATIC_GAIN * ay
        omega_r = 2 * math.pi * self.ROLL_FREQ_HZ
        roll_ddot = (omega_r**2 * (target_roll - self._roll)
                     - 2 * self.ROLL_ZETA * omega_r * self._roll_rate)
        self._roll_rate += roll_ddot * dt
        self._roll += self._roll_rate * dt

        # Position
        self._pos_x += self.speed_ms * dt
        self._pos_y += (self.speed_ms * math.sin(self._yaw)) * dt

        # Phase
        phase = 'straight'
        for lc in range(self.NUM_LANE_CHANGES):
            t_start = self.MANEUVER_START_S + lc * self.MANEUVER_DURATION_S
            if t_start <= t <= t_start + self.MANEUVER_DURATION_S:
                phase = f'lane_change_{lc+1}'
                break

        # Update state
        self._state.x = self._pos_x
        self._state.y = self._pos_y
        self._state.speed = self.speed_ms
        self._state.ay = ay
        self._state.roll = self._roll
        self._state.yaw = self._yaw
        self._state.yaw_rate = self._yaw_rate
        self._state.simulation_time = t
        self._state.frame_index = self._frame_index
        self._state.scenario_phase = phase

        return self._make_frame(
            t=t,
            ax=0.0, ay=ay, az=0.0,
            roll=self._roll, yaw=self._yaw,
            roll_rate=self._roll_rate, yaw_rate=self._yaw_rate,
            vx=self.speed_ms,
            vy=self.speed_ms * math.sin(self._yaw),
        )
