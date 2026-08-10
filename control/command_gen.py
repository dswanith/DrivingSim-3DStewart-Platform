"""
sil_sim.control.command_gen — Stepper motor command generator.

Takes PID velocity output and generates per-timestep STEP/DIR/ENABLE signals,
target RPM, and pulse frequency via the ScrewModel.

Generates per-timestep raw STEP/DIR/ENABLE state — replayable onto real
STM32 firmware. This is NOT just an aggregate frequency; it is the actual
control signal state at each timestep.

Output class: EXACT — pure math on the command.
"""

import math
import numpy as np
from typing import Dict, Any, NamedTuple

from sil_sim.actuator.screw_model import ScrewModel


class StepperCommand(NamedTuple):
    """
    Per-timestep stepper motor command.

    These fields represent the state of the STEP/DIR/ENABLE signals
    that the STM32 would output to the DM542 driver at this timestep.

    Output class: EXACT.
    """
    # Control signals (per-timestep state)
    step_active: bool      # True if a step pulse should be issued this timestep
    direction: int         # 1 = extend, 0 = retract
    enable: bool           # True if driver is enabled (motor energized)

    # Derived quantities
    target_rpm: float      # Target motor RPM (signed: + extend, - retract)
    step_frequency_hz: float  # Step pulse frequency (Hz, always ≥ 0)
    velocity_m_s: float    # Commanded linear velocity (m/s)

    # Pulse accumulator state
    pulses_this_step: int  # Integer pulses to issue this timestep


class CommandGenerator:
    """
    Generates stepper motor commands from PID velocity output.

    Uses the ScrewModel to convert between velocity/displacement and
    rotary motor quantities. Maintains a fractional pulse accumulator
    to correctly distribute pulses over time at any frequency.

    The per-timestep STEP/DIR/ENABLE output can be logged and later
    replayed onto real STM32 firmware.
    """

    def __init__(self, screw_model: ScrewModel, control_rate_hz: float):
        """
        Args:
            screw_model: ScrewModel instance for kinematic conversions.
            control_rate_hz: PID/command update rate (Hz).
        """
        self.screw = screw_model
        self.dt = 1.0 / control_rate_hz

        # Fractional pulse accumulator — tracks sub-pulse remainders
        # so that over many timesteps, the total pulse count is correct
        self._pulse_accumulator = 0.0

    def reset(self) -> None:
        """Reset pulse accumulator."""
        self._pulse_accumulator = 0.0

    def generate(self, velocity_cmd: float) -> StepperCommand:
        """
        Generate stepper command for one timestep.

        Args:
            velocity_cmd: Velocity command from PID (m/s).
                          Positive = extend, negative = retract.

        Returns:
            StepperCommand with all fields populated. EXACT.
        """
        # --- Direction ---
        if abs(velocity_cmd) < 1e-12:
            # Effectively stationary
            return StepperCommand(
                step_active=False,
                direction=1,
                enable=True,
                target_rpm=0.0,
                step_frequency_hz=0.0,
                velocity_m_s=0.0,
                pulses_this_step=0,
            )

        direction = 1 if velocity_cmd > 0 else 0

        # --- RPM and step frequency ---
        target_rpm = self.screw.velocity_to_rpm(velocity_cmd)
        step_freq = self.screw.rpm_to_step_frequency(target_rpm)

        # --- Pulse count for this timestep ---
        # Number of pulses = step_frequency × dt
        # Use accumulator to handle fractional pulses correctly
        exact_pulses = step_freq * self.dt
        self._pulse_accumulator += exact_pulses
        integer_pulses = int(self._pulse_accumulator)
        self._pulse_accumulator -= integer_pulses

        step_active = integer_pulses > 0

        return StepperCommand(
            step_active=step_active,
            direction=direction,
            enable=True,
            target_rpm=target_rpm,
            step_frequency_hz=step_freq,
            velocity_m_s=velocity_cmd,
            pulses_this_step=integer_pulses,
        )
