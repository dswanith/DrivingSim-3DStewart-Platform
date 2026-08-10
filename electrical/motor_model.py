"""
sil_sim.electrical.motor_model — Motor torque and current estimation.

Estimates the torque required from each motor and the resulting phase current.

ALL OUTPUTS FROM THIS MODULE ARE LABELED **ESTIMATED**.
No output should ever be labeled MEASURED.

Equations from §5:
    T_required = F_axial × lead / (2π × η_screw)
    Kt_est     = T_holding_rated / I_rated  (≈ 0.55–0.75 N·m/A)
    I_phase_est = T_required / Kt_est

Config-visible assumptions (NOT hardcoded):
    - Kt_est: motor torque constant (electrical.yaml)
    - η_screw: ballscrew efficiency (electrical.yaml)
    - payload mass: total supported mass (electrical.yaml)

Output class: ESTIMATED.
"""

import math
from typing import Dict, Any


# Gravitational acceleration (SI)
GRAVITY = 9.80665  # m/s²


class MotorModel:
    """
    Motor torque and phase current estimation model.

    Estimates the torque a motor must produce to drive its ballscrew
    actuator under the configured payload, and the resulting phase current.

    ALL outputs are ESTIMATED — based on model parameters, not measurements.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Args:
            config: Validated electrical config dict.
        """
        # Config-visible model assumptions
        self.kt_est = config['motor']['kt_estimated_nm_per_a']    # N·m/A — ESTIMATED
        self.rated_current = config['motor']['rated_current_a']    # A — CONFIRMED
        self.eta_screw = config['efficiencies']['eta_screw']       # dimensionless — ESTIMATED
        self.lead = 0.004  # Will be overridden from actuator config if available

        # Payload (ESTIMATED — genuinely unknown, placeholder value)
        total_mass = config['payload']['total_payload_kg'] + config['payload']['platform_mass_kg']
        self.per_leg_mass = total_mass / 6.0  # Even distribution at neutral — ESTIMATED

        # Derived: per-leg gravity force (ESTIMATED)
        self.per_leg_force_n = self.per_leg_mass * GRAVITY  # N — ESTIMATED

    def set_lead(self, lead_m: float) -> None:
        """Set ballscrew lead from actuator config."""
        self.lead = lead_m

    def estimate_torque(self, axial_force_n: float = None) -> float:
        """
        Estimate motor torque required to drive the ballscrew.

        Equation (§5):
            T_required = F_axial × lead / (2π × η_screw)

        Args:
            axial_force_n: Axial force on the screw (N). If None, uses
                           per-leg gravity force from config.

        Returns:
            Estimated motor torque (N·m). ESTIMATED.
        """
        if axial_force_n is None:
            axial_force_n = self.per_leg_force_n

        # T_required = F × lead / (2π × η)
        torque = axial_force_n * self.lead / (2.0 * math.pi * self.eta_screw)
        return torque

    def estimate_phase_current(self, axial_force_n: float = None) -> float:
        """
        Estimate motor phase current from axial force.

        Equation (§5):
            I_phase_est = T_required / Kt_est

        Args:
            axial_force_n: Axial force (N). If None, uses per-leg gravity force.

        Returns:
            Estimated phase current (A). ESTIMATED.
        """
        torque = self.estimate_torque(axial_force_n)
        current = torque / self.kt_est
        return current

    def estimate_from_velocity(self, velocity_m_s: float, axial_force_n: float = None) -> dict:
        """
        Full motor estimation for a given velocity and force.

        Returns a dict with all ESTIMATED quantities.

        Args:
            velocity_m_s: Actuator linear velocity (m/s).
            axial_force_n: Axial force (N). If None, uses gravity load.

        Returns:
            Dict with keys (all ESTIMATED):
                torque_nm, phase_current_a, mechanical_power_w, electrical_power_w
        """
        if axial_force_n is None:
            axial_force_n = self.per_leg_force_n

        torque = self.estimate_torque(axial_force_n)
        phase_current = torque / self.kt_est

        # Mechanical power at the screw
        mech_power = axial_force_n * abs(velocity_m_s)

        # Electrical power into the motor (P = V·I approximated as T·ω for DC equiv)
        # More accurately: P_elec = P_mech / η_motor, but we don't have η_motor
        # Use power-balance: P_elec ≈ T × ω where ω = 2π × RPM/60
        rpm = abs(velocity_m_s) / self.lead * 60.0
        omega = 2.0 * math.pi * rpm / 60.0
        elec_power = torque * omega / self.eta_screw if self.eta_screw > 0 else 0.0

        return {
            'torque_nm_ESTIMATED': torque,
            'phase_current_a_ESTIMATED': phase_current,
            'mechanical_power_w_ESTIMATED': mech_power,
            'electrical_power_w_ESTIMATED': elec_power,
        }
