"""
scenarios.turn — Cornering scenario.

Vehicle enters a left or right turn.

Physics model:
    During cornering, lateral acceleration ay = v² / R
    where R is the turn radius.

    - Vehicle starts straight, smoothly enters a constant-radius turn
    - Lateral acceleration builds up as the steering increases
    - Yaw rate = v / R
    - Vehicle body rolls toward the outside (negative roll for left turn)
    - MCA generates roll cue + sustained lateral tilt coordination

    Turn sequence:
        0–2s: Straight approach
        2–4s: Smooth steering input (transition)
        4–10s: Constant radius turn
        10–12s: Exit turn, return to straight
        12–15s: Straight exit
"""
import math
from .base import BaseScenario
from sil_sim.telemetry.base import TelemetryFrame


class TurnScenario(BaseScenario):
    """
    Constant-radius turn scenario. Produces lateral acceleration, yaw rate, and roll.
    """

    SCENARIO_NAME = "turn"
    DEFAULT_DURATION_S = 18.0

    # Turn parameters
    TURN_RADIUS_M = 40.0          # Radius of curvature (m) — tight suburban corner
    TURN_DIRECTION = 'left'       # 'left' or 'right'
    TURN_START_TIME_S = 3.0       # Time steering begins
    TURN_RAMP_DURATION_S = 2.5    # Time to fully enter the turn
    TURN_HOLD_DURATION_S = 8.0    # Duration of constant-radius phase
    TURN_EXIT_DURATION_S = 2.5    # Time to return to straight

    # Vehicle body roll parameters
    ROLL_STATIC_GAIN = 0.06       # Static roll per g of lateral accel (rad/g·g = rad/(m/s²))
    ROLL_FREQ_HZ = 2.0            # Body roll natural frequency
    ROLL_ZETA = 0.5               # Moderately damped

    def __init__(
        self,
        rate_hz: float = 60.0,
        duration_s: float = None,
        speed_ms: float = 10.0,
        direction: str = 'left',
        radius_m: float = 40.0,
    ):
        super().__init__(rate_hz=rate_hz, duration_s=duration_s, speed_ms=speed_ms)
        self.TURN_DIRECTION = direction
        self.TURN_RADIUS_M = radius_m

        # Sign: left turn → positive yaw rate (counterclockwise from above)
        #                  → negative lateral acceleration (centripetal to the right)
        # Note: automotive axes Y=left, so lateral accel during left turn is -Y
        self._sign = +1.0 if direction == 'left' else -1.0

        # State
        self._roll = 0.0
        self._roll_rate = 0.0
        self._yaw = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0

    def _on_reset(self):
        self._roll = 0.0
        self._roll_rate = 0.0
        self._yaw = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0

    def _steering_blend(self, t: float) -> float:
        """Normalised steering amount [0, 1] as a function of time."""
        t_start = self.TURN_START_TIME_S
        t_full = t_start + self.TURN_RAMP_DURATION_S
        t_exit_start = t_full + self.TURN_HOLD_DURATION_S
        t_exit_end = t_exit_start + self.TURN_EXIT_DURATION_S

        if t < t_start:
            return 0.0
        elif t < t_full:
            return self._smooth_step(t, t_start, t_full)
        elif t < t_exit_start:
            return 1.0
        elif t < t_exit_end:
            return 1.0 - self._smooth_step(t, t_exit_start, t_exit_end)
        else:
            return 0.0

    def _generate_frame(self, t: float, dt: float) -> TelemetryFrame:
        blend = self._steering_blend(t)

        # Lateral acceleration: ay = ±v² / R * blend
        # Positive Y = left, so left turn has negative ay (centripetal force pushes right)
        ay_magnitude = (self.speed_ms ** 2) / self.TURN_RADIUS_M
        ay = -self._sign * ay_magnitude * blend  # left turn: ay < 0 (force pushes right)

        # Yaw rate: positive = counterclockwise (left turn)
        yaw_rate = self._sign * (self.speed_ms / self.TURN_RADIUS_M) * blend

        # Integrate yaw
        self._yaw += yaw_rate * dt

        # Body roll: rolls outward from turn
        # Left turn → vehicle rolls right (negative roll in automotive convention)
        target_roll = -self._sign * self.ROLL_STATIC_GAIN * ay_magnitude * blend

        # Roll dynamics
        omega_r = 2 * math.pi * self.ROLL_FREQ_HZ
        roll_ddot = (omega_r**2 * (target_roll - self._roll)
                     - 2 * self.ROLL_ZETA * omega_r * self._roll_rate)
        self._roll_rate += roll_ddot * dt
        self._roll += self._roll_rate * dt

        # Position integration (approximate planar path)
        self._pos_x += self.speed_ms * math.cos(self._yaw) * dt
        self._pos_y += self.speed_ms * math.sin(self._yaw) * dt

        # Phase
        if blend == 0.0:
            phase = 'straight'
        elif blend < 1.0 and t < self.TURN_START_TIME_S + self.TURN_RAMP_DURATION_S + 0.1:
            phase = 'entering_turn'
        elif blend == 1.0:
            phase = 'cornering'
        else:
            phase = 'exiting_turn'

        # Update state
        self._state.x = self._pos_x
        self._state.y = self._pos_y
        self._state.speed = self.speed_ms
        self._state.ay = ay
        self._state.roll = self._roll
        self._state.yaw = self._yaw
        self._state.yaw_rate = yaw_rate
        self._state.simulation_time = t
        self._state.frame_index = self._frame_index
        self._state.scenario_phase = phase

        return self._make_frame(
            t=t,
            ax=0.0, ay=ay, az=0.0,
            roll=self._roll, yaw=self._yaw,
            roll_rate=self._roll_rate, yaw_rate=yaw_rate,
            vx=self.speed_ms * math.cos(self._yaw),
            vy=self.speed_ms * math.sin(self._yaw),
        )
