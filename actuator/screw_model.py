"""
sil_sim.actuator.screw_model — Ballscrew kinematics model.

Converts between linear displacement/velocity and rotary motion
(revolutions, steps, pulses, RPM, step frequency).

Equations from §5:
    screw_rev   = ΔL / lead                           (lead = 4 mm = 0.004 m)
    motor_rev   = screw_rev × coupling_ratio           (1:1 → motor_rev = screw_rev)
    full_steps  = motor_rev × steps_per_rev            (200 steps/rev)
    pulses      = full_steps × microsteps_per_fullstep
    RPM         = (dL/dt / lead) × 60
    f_step      = (RPM / 60) × steps_per_rev × microsteps_per_fullstep

Output class: EXACT — pure kinematics, no model assumptions.
"""

import math
from typing import Dict, Any


class ScrewModel:
    """
    Ballscrew + motor + driver kinematics model.

    Converts between linear actuator domain and rotary motor domain.
    All inputs/outputs in SI units unless otherwise noted.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize from actuator config dict.

        Args:
            config: Validated actuator config dict (from loader.load_actuator()).
        """
        self.lead = config['ballscrew']['lead_m']                 # metres/rev
        self.steps_per_rev = config['motor']['steps_per_rev']     # 200
        self.coupling_ratio = config['coupling']['ratio']         # 1.0 (1:1)
        self.microsteps_per_fullstep = config['driver']['microsteps_per_fullstep']

        # Derived constants (EXACT)
        self.microsteps_per_rev = self.steps_per_rev * self.microsteps_per_fullstep
        self.metres_per_step = self.lead / self.steps_per_rev
        self.metres_per_pulse = self.lead / self.microsteps_per_rev

    def displacement_to_screw_revs(self, delta_l: float) -> float:
        """
        Convert linear displacement to screw revolutions.

        Equation (§5): screw_rev = ΔL / lead

        Args:
            delta_l: Linear displacement (metres). Positive = extension.

        Returns:
            Screw revolutions (float). EXACT.
        """
        return delta_l / self.lead

    def screw_revs_to_motor_revs(self, screw_revs: float) -> float:
        """
        Convert screw revolutions to motor revolutions.

        With 1:1 coupling: motor_rev = screw_rev × coupling_ratio

        Args:
            screw_revs: Screw revolutions.

        Returns:
            Motor revolutions. EXACT.
        """
        return screw_revs * self.coupling_ratio

    def displacement_to_motor_revs(self, delta_l: float) -> float:
        """
        Convert linear displacement to motor revolutions (combined).

        Equation (§5): motor_rev = (ΔL / lead) × coupling_ratio

        Args:
            delta_l: Linear displacement (metres).

        Returns:
            Motor revolutions. EXACT.
        """
        screw_revs = self.displacement_to_screw_revs(delta_l)
        return self.screw_revs_to_motor_revs(screw_revs)

    def motor_revs_to_full_steps(self, motor_revs: float) -> float:
        """
        Convert motor revolutions to full steps.

        Equation (§5): full_steps = motor_rev × steps_per_rev (200)

        Args:
            motor_revs: Motor revolutions.

        Returns:
            Full steps (float — may be fractional). EXACT.
        """
        return motor_revs * self.steps_per_rev

    def full_steps_to_pulses(self, full_steps: float) -> float:
        """
        Convert full steps to driver pulses (microsteps).

        Equation (§5): pulses = full_steps × microsteps_per_fullstep

        Args:
            full_steps: Full motor steps.

        Returns:
            Driver pulses (microsteps). EXACT.
        """
        return full_steps * self.microsteps_per_fullstep

    def displacement_to_pulses(self, delta_l: float) -> float:
        """
        Convert linear displacement to total driver pulses (combined chain).

        Args:
            delta_l: Linear displacement (metres).

        Returns:
            Driver pulses. EXACT.
        """
        motor_revs = self.displacement_to_motor_revs(delta_l)
        full_steps = self.motor_revs_to_full_steps(motor_revs)
        return self.full_steps_to_pulses(full_steps)

    def velocity_to_rpm(self, dl_dt: float) -> float:
        """
        Convert linear velocity to motor RPM.

        Equation (§5): RPM = (dL/dt / lead) × 60

        Args:
            dl_dt: Linear velocity (m/s). Positive = extension.

        Returns:
            Motor RPM (revolutions per minute). EXACT.
        """
        return (dl_dt / self.lead) * 60.0

    def rpm_to_velocity(self, rpm: float) -> float:
        """
        Convert motor RPM to linear velocity.

        Inverse of velocity_to_rpm: dL/dt = RPM × lead / 60

        Args:
            rpm: Motor RPM.

        Returns:
            Linear velocity (m/s). EXACT.
        """
        return rpm * self.lead / 60.0

    def rpm_to_step_frequency(self, rpm: float) -> float:
        """
        Convert motor RPM to step (pulse) frequency.

        Equation (§5): f_step = (RPM / 60) × steps_per_rev × microsteps_per_fullstep

        Args:
            rpm: Motor RPM (absolute value used for frequency).

        Returns:
            Step frequency (Hz). EXACT.
        """
        return (abs(rpm) / 60.0) * self.microsteps_per_rev

    def velocity_to_step_frequency(self, dl_dt: float) -> float:
        """
        Convert linear velocity directly to step frequency.

        Args:
            dl_dt: Linear velocity (m/s).

        Returns:
            Step frequency (Hz). EXACT.
        """
        rpm = self.velocity_to_rpm(dl_dt)
        return self.rpm_to_step_frequency(rpm)

    def velocity_to_direction(self, dl_dt: float) -> int:
        """
        Determine step direction from velocity sign.

        Returns:
            1 for extension (positive velocity), 0 for retraction, -1 for zero.
        """
        if dl_dt > 0:
            return 1
        elif dl_dt < 0:
            return 0
        else:
            return -1  # Stationary — no step direction
