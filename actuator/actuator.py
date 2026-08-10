"""
sil_sim.actuator.actuator — Integrated actuator simulation.

Combines ScrewModel + EncoderModel + PIDController + CommandGenerator
into a single actuator simulation unit. One instance per leg (6 total).

Plant model: position += velocity × dt
    This is a KINEMATIC INTEGRATOR — no inertia, friction, or backlash.
    This is the right call for SIL validation, but it does NOT represent
    validated firmware behavior. The README documents this limitation.

The encoder reads from the simulated plant position (independent path),
NOT from the commanded position.

Output class: Mixed — kinematics are EXACT, PID tracking is EXACT math
on a simplified plant.
"""

import numpy as np
from typing import Dict, Any, Optional

from sil_sim.actuator.screw_model import ScrewModel
from sil_sim.actuator.encoder_model import EncoderModel
from sil_sim.control.pid import PIDController, create_pid_from_config
from sil_sim.control.command_gen import CommandGenerator, StepperCommand


class ActuatorState:
    """
    Complete state of one actuator at a single timestep.
    """
    __slots__ = [
        'leg_index',
        'target_position_m',
        'plant_position_m',
        'encoder_counts',
        'encoder_position_m',
        'position_error_m',
        'velocity_cmd_m_s',
        'motor_rpm',
        'step_frequency_hz',
        'step_active',
        'direction',
        'enable',
        'pulses_this_step',
        'clamped',
    ]

    def __init__(self):
        self.leg_index = 0
        self.target_position_m = 0.0
        self.plant_position_m = 0.0
        self.encoder_counts = 0
        self.encoder_position_m = 0.0
        self.position_error_m = 0.0
        self.velocity_cmd_m_s = 0.0
        self.motor_rpm = 0.0
        self.step_frequency_hz = 0.0
        self.step_active = False
        self.direction = 1
        self.enable = True
        self.pulses_this_step = 0
        self.clamped = False

    def to_dict(self) -> dict:
        """Convert to dict for logging."""
        return {k: getattr(self, k) for k in self.__slots__}


class ActuatorSim:
    """
    Single actuator simulation: screw + encoder + PID + command generator.

    Plant model: position += velocity × dt (kinematic integrator).
    Encoder reads from plant position (independent path, with quantization).
    PID error = target − encoder reading.
    """

    def __init__(
        self,
        leg_index: int,
        actuator_config: Dict[str, Any],
        pid_config: Dict[str, Any],
        initial_position_m: float = 0.0,
    ):
        """
        Args:
            leg_index: Leg number (0–5).
            actuator_config: Validated actuator config dict.
            pid_config: Validated PID config dict.
            initial_position_m: Initial actuator position (metres).
        """
        self.leg_index = leg_index

        # Sub-models
        self.screw = ScrewModel(actuator_config)
        self.encoder = EncoderModel(actuator_config)
        self.pid = create_pid_from_config(pid_config)
        self.cmd_gen = CommandGenerator(
            self.screw,
            pid_config['control_rate_hz'],
        )

        # Plant state
        self._position = initial_position_m  # True plant position (metres)
        self._dt = 1.0 / pid_config['control_rate_hz']

        # Initialize encoder to match initial position
        self.encoder.reset(initial_position_m)

        # Stroke limits (for clamping)
        self._min_stroke = 0.0  # Set from safety config
        self._max_stroke = actuator_config['ballscrew']['estimated_usable_stroke_m']
        
        # Timing
        self._sub_step_accumulator = 0.0

    def set_stroke_limits(self, min_len: float, max_len: float) -> None:
        """Set actuator stroke limits (from safety config)."""
        self._min_stroke = min_len
        self._max_stroke = max_len

    def reset(self, position_m: float = 0.0) -> None:
        """Reset actuator to a known position."""
        self._position = position_m
        self.encoder.reset(position_m)
        self.pid.reset()
        self.cmd_gen.reset()

    def update(self, target_leg_length: float, dt: Optional[float] = None) -> ActuatorState:
        """
        Run PID control cycle using sub-stepping to match the control rate.

        Args:
            target_leg_length: Target leg length from IK (metres).
            dt: The elapsed time of this MCA frame. If None, defaults to PID rate.

        Returns:
            ActuatorState with all fields populated.
        """
        state = ActuatorState()
        state.leg_index = self.leg_index
        state.target_position_m = target_leg_length

        actual_dt = dt if dt is not None else self._dt
        total_steps = (actual_dt / self._dt) + self._sub_step_accumulator
        num_steps = int(total_steps)
        self._sub_step_accumulator = total_steps - num_steps

        if num_steps <= 0:
            state.plant_position_m = self._position
            state.encoder_position_m = self.encoder.get_position()
            return state

        total_pulses = 0
        target_clamped = target_leg_length

        for _ in range(num_steps):
            # --- Step 1: Read encoder (INDEPENDENT path from command) ---
            encoder_counts = self.encoder.update(self._position)
            encoder_position = self.encoder.get_position()

            # --- Step 2 & 3: PID control ---
            # Clamp target to physical stroke limits to prevent integrator windup on infeasible IK poses
            target_clamped = max(self._min_stroke, min(self._max_stroke, target_leg_length))
            state.clamped = (target_clamped != target_leg_length)

            velocity_cmd = self.pid.update(
                setpoint=target_clamped,
                measurement=encoder_position,
            )

            # --- Step 4: Command generation (STEP/DIR/ENABLE) ---
            cmd = self.cmd_gen.generate(velocity_cmd)
            total_pulses += cmd.pulses_this_step

            # --- Step 5: Plant model (kinematic integrator) ---
            # position += velocity × dt
            # This is NOT a validated firmware model — see README
            self._position += velocity_cmd * self._dt

            # Clamp to stroke limits
            self._position = max(self._min_stroke, min(self._max_stroke, self._position))

        # Fill the state object with the final state at the end of the frame
        state.encoder_counts = encoder_counts
        state.encoder_position_m = encoder_position
        state.clamped = (target_clamped != target_leg_length)
        state.position_error_m = self.pid.last_error
        state.velocity_cmd_m_s = velocity_cmd
        state.motor_rpm = cmd.target_rpm
        state.step_frequency_hz = cmd.step_frequency_hz
        state.step_active = cmd.step_active
        state.direction = cmd.direction
        state.enable = cmd.enable
        state.pulses_this_step = total_pulses
        state.plant_position_m = self._position

        return state

    @property
    def position(self) -> float:
        """Current plant position (metres)."""
        return self._position
