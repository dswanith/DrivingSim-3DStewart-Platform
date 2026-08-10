"""
sil_sim.control.pid — Discrete PID controller with anti-windup.

Implements a standard discrete-time PID controller as used on the
STM32F407 for per-actuator position control. The error is
(target_position − encoder_position), and the output is a velocity command.

Features:
    - Configurable Kp, Ki, Kd gains
    - Output saturation with configurable limits
    - Anti-windup (clamping method): stops integrator accumulation
      when output is saturated
    - Configurable sample rate

Output class: EXACT — pure math on the control law.
"""

import math
from typing import Dict, Any, Optional


class PIDController:
    """
    Discrete PID controller with anti-windup.

    Standard form:
        e[n] = setpoint - measurement
        P = Kp × e[n]
        I += Ki × e[n] × dt    (with anti-windup clamping)
        D = Kd × (e[n] - e[n-1]) / dt
        output = clamp(P + I + D, min_out, max_out)

    One instance per actuator (6 total in the simulator).
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float,
        output_max: float,
        sample_rate_hz: float,
        anti_windup: bool = True,
        derivative_filter_hz: Optional[float] = None,
    ):
        """
        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            output_min: Minimum output (velocity, m/s).
            output_max: Maximum output (velocity, m/s).
            sample_rate_hz: PID update rate (Hz).
            anti_windup: Enable anti-windup clamping.
            derivative_filter_hz: Optional LPF cutoff for D term (Hz).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.dt = 1.0 / sample_rate_hz
        self.anti_windup = anti_windup

        # Derivative filter
        self._d_filter_alpha = 1.0  # No filter by default
        if derivative_filter_hz is not None and derivative_filter_hz > 0:
            # First-order LPF: α = dt / (dt + 1/(2π·fc))
            rc = 1.0 / (2.0 * math.pi * derivative_filter_hz)
            self._d_filter_alpha = self.dt / (self.dt + rc)

        # State
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._saturated = False

    def reset(self) -> None:
        """Reset controller state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._saturated = False

    def update(self, setpoint: float, measurement: float) -> float:
        """
        Compute one PID update step.

        Args:
            setpoint: Target position (metres).
            measurement: Measured position from encoder (metres).

        Returns:
            Velocity command (m/s), clamped to output limits. EXACT.
        """
        error = setpoint - measurement

        # --- Proportional ---
        p_term = self.kp * error

        # --- Integral (with anti-windup clamping) ---
        if not (self.anti_windup and self._saturated):
            self._integral += self.ki * error * self.dt

        i_term = self._integral

        # --- Derivative (with optional low-pass filter) ---
        raw_derivative = (error - self._prev_error) / self.dt if self.dt > 0 else 0.0
        # Apply low-pass filter to derivative
        filtered_derivative = (
            self._d_filter_alpha * raw_derivative
            + (1.0 - self._d_filter_alpha) * self._prev_derivative
        )
        d_term = self.kd * filtered_derivative

        # --- Sum ---
        output_raw = p_term + i_term + d_term

        # --- Output saturation ---
        output = max(self.output_min, min(self.output_max, output_raw))
        self._saturated = (output != output_raw)

        # --- Update state ---
        self._prev_error = error
        self._prev_derivative = filtered_derivative

        return output

    @property
    def last_error(self) -> float:
        """Last computed error value."""
        return self._prev_error

    @property
    def integral_value(self) -> float:
        """Current integral accumulator value."""
        return self._integral


def create_pid_from_config(config: Dict[str, Any]) -> PIDController:
    """
    Create a PIDController from PID config.

    Args:
        config: Validated PID config dict.

    Returns:
        PIDController instance.
    """
    gains = config['gains']
    limits = config['output_limits']
    aw = config['anti_windup']

    d_filter = config.get('derivative_filter_hz', None)

    return PIDController(
        kp=gains['kp'],
        ki=gains['ki'],
        kd=gains['kd'],
        output_min=limits['min_output_m_s'],
        output_max=limits['max_output_m_s'],
        sample_rate_hz=config['control_rate_hz'],
        anti_windup=aw['enabled'],
        derivative_filter_hz=d_filter,
    )
