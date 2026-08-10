"""
sil_sim.electrical.bus_model — DC bus current and power estimation.

Computes total DC bus draw from all 6 actuator drivers plus the logic rail.

Equations from §5:
    I_bus_leg   = P_driver_in / V_bus           (per leg)
    I_bus_total = Σ(I_bus_leg, 6 legs) + I_logic_5V_rail
    P_bus_total = I_bus_total × V_bus
    utilization = P_bus_total / P_psu_capacity

CRITICAL: Bus current is derived from POWER BALANCE, not from phase current.

ALL outputs are ESTIMATED.
"""

from typing import Dict, Any, List


class BusModel:
    """
    DC bus model for the 48V power supply.

    Aggregates per-leg driver power draws and the 5V logic rail draw
    to estimate total bus current, power, and PSU utilization.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Args:
            config: Validated electrical config dict.
        """
        self.bus_voltage = config['psu']['bus_voltage_v']          # 48V — CONFIRMED
        self.psu_capacity_w = config['psu']['bus_capacity_w']      # 480W — CONFIRMED
        self.psu_capacity_a = config['psu']['bus_capacity_a']      # 10A — CONFIRMED
        self.logic_draw_a = config['logic_supply']['estimated_draw_a']  # ESTIMATED
        self.logic_voltage = config['logic_supply']['voltage_v']   # 5V — CONFIRMED

    def estimate_bus_state(self, driver_powers_w: List[float]) -> dict:
        """
        Estimate total bus state from per-leg driver input powers.

        Equations (§5):
            I_bus_leg[i] = P_driver_in[i] / V_bus
            I_bus_total  = Σ(I_bus_leg) + I_logic
            P_bus_total  = Σ(P_driver_in) + P_logic
            utilization  = P_bus_total / P_psu_capacity

        Args:
            driver_powers_w: List of 6 per-leg driver input powers (W). ESTIMATED.

        Returns:
            Dict with all ESTIMATED quantities:
                per_leg_bus_current_a: list of 6 currents
                total_bus_current_a: total DC bus current
                total_bus_power_w: total DC bus power
                psu_utilization: fraction of PSU capacity used
                logic_power_w: 5V rail power
        """
        # Per-leg bus current: I = P / V (power balance, NOT phase current)
        per_leg_current = [p / self.bus_voltage if self.bus_voltage > 0 else 0.0
                           for p in driver_powers_w]

        # Logic rail power
        logic_power = self.logic_draw_a * self.logic_voltage

        # Totals
        total_driver_power = sum(driver_powers_w)
        total_power = total_driver_power + logic_power

        # Total bus current (from 48V supply)
        # Logic supply is separate (5V SMPS fed from AC mains, not from 48V bus)
        # But for PSU utilization, we track total system power
        total_bus_current = sum(per_leg_current)

        # PSU utilization (of the 48V supply only)
        utilization = total_driver_power / self.psu_capacity_w if self.psu_capacity_w > 0 else 0.0

        return {
            'bus_voltage_v': self.bus_voltage,
            'per_leg_bus_current_a_ESTIMATED': per_leg_current,
            'total_bus_current_a_ESTIMATED': total_bus_current,
            'total_driver_power_w_ESTIMATED': total_driver_power,
            'logic_power_w_ESTIMATED': logic_power,
            'total_system_power_w_ESTIMATED': total_power,
            'psu_utilization_ESTIMATED': utilization,
        }
