"""
sil_sim.electrical.power_model — Driver power estimation model.

Estimates driver input power from motor electrical power using
an explicit power-balance approach.

CRITICAL RULE (§1, Rule 2):
    Never derive 48V bus current from motor phase current directly.
    Motor phase current and driver DC-bus draw are DIFFERENT electrical
    domains. This module goes through power balance:
        P_driver_in = P_motor_elec / η_driver

Config-visible assumptions:
    - η_driver: driver efficiency (electrical.yaml) — ESTIMATED

ALL outputs are ESTIMATED.
"""

from typing import Dict, Any


class PowerModel:
    """
    Driver input power estimation using power balance.

    The DM542 driver converts DC bus power to motor phase power.
    We estimate the DC-side draw from the motor-side power:
        P_driver_in = P_motor_elec / η_driver

    This does NOT derive bus current from phase current directly —
    those are different electrical domains (motor AC phase vs DC bus).
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Args:
            config: Validated electrical config dict.
        """
        self.eta_driver = config['efficiencies']['eta_driver']  # ESTIMATED

    def estimate_driver_input_power(self, motor_electrical_power_w: float) -> float:
        """
        Estimate DC-bus power consumed by the driver.

        Equation (§5):
            P_driver_in = P_motor_elec / η_driver

        Args:
            motor_electrical_power_w: Electrical power into the motor (W). ESTIMATED.

        Returns:
            Driver DC-bus input power (W). ESTIMATED.
        """
        if self.eta_driver <= 0:
            return 0.0
        return motor_electrical_power_w / self.eta_driver
