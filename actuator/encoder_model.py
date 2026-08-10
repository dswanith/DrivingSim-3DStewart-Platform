"""
sil_sim.actuator.encoder_model — Quadrature encoder simulation model.

Simulates the ABZ quadrature encoder mounted at the ballscrew's FREE end
(BF10 side). This is a genuinely INDEPENDENT signal path from the motor
command path — it does NOT shortcut to equal the commanded position.

The encoder reads screw rotation directly, independent of the motor-side
coupling. In simulation, this means:
    1. The "true" screw position is tracked by the actuator plant model
    2. The encoder reads that true position through quantization
    3. The PID error is (commanded − encoder_read), NOT (commanded − commanded)

Equations from §5:
    counts          = screw_rev × counts_per_rev (2400)
    measured_pos    = (counts / counts_per_rev) × lead

Output class: EXACT — the encoder math is exact; quantization models a real effect.
"""

import math
import numpy as np
from typing import Dict, Any


class EncoderModel:
    """
    Quadrature encoder model for screw-side position measurement.

    Simulates the 600 PPR ABZ encoder with ×4 quadrature decode,
    giving 2400 counts per revolution. Mounted on the BF10 (free) end
    of the ballscrew, measuring screw rotation directly.

    This model deliberately introduces quantization (discrete counts)
    to keep the encoder path genuinely independent from the command path.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize from actuator config dict.

        Args:
            config: Validated actuator config dict.
        """
        self.ppr = config['encoder']['ppr']                              # 600
        self.quadrature_multiplier = config['encoder']['quadrature_multiplier']  # 4
        self.counts_per_rev = config['encoder']['counts_per_rev']        # 2400
        self.lead = config['ballscrew']['lead_m']                        # 0.004 m

        # Resolution: smallest detectable position change
        self.resolution_m = self.lead / self.counts_per_rev  # metres per count

        # Internal state: accumulated counts (integer — this is the quantization)
        self._accumulated_counts = 0
        self._fractional_counts = 0.0  # Sub-count accumulator for smooth tracking

    def reset(self, initial_position_m: float = 0.0) -> None:
        """
        Reset encoder to a known position (e.g. after homing).

        Args:
            initial_position_m: Initial linear position (metres).
        """
        screw_revs = initial_position_m / self.lead
        exact_counts = screw_revs * self.counts_per_rev
        self._accumulated_counts = int(round(exact_counts))
        self._fractional_counts = 0.0

    def update(self, true_position_m: float) -> int:
        """
        Update encoder reading from the true screw position.

        This is the INDEPENDENT measurement path. The true position comes
        from the actuator plant model (position += velocity × dt), NOT
        from the commanded position.

        The encoder quantizes to discrete counts, so:
            counts = round(screw_rev × counts_per_rev)

        Args:
            true_position_m: True linear position of the actuator (metres).

        Returns:
            Encoder count (integer). EXACT (quantized).
        """
        # Convert true position to screw revolutions
        screw_revs = true_position_m / self.lead

        # Convert to counts — this is where quantization happens
        # Equation (§5): counts = screw_rev × 2400
        exact_counts = screw_revs * self.counts_per_rev
        self._accumulated_counts = int(round(exact_counts))

        return self._accumulated_counts

    def get_counts(self) -> int:
        """
        Get the current encoder count reading.

        Returns:
            Integer count. EXACT (quantized).
        """
        return self._accumulated_counts

    def counts_to_position(self, counts: int) -> float:
        """
        Convert encoder counts to linear position.

        Equation (§5): measured_position = (counts / 2400) × lead

        Args:
            counts: Encoder count (integer).

        Returns:
            Linear position (metres). EXACT.
        """
        screw_revs = counts / self.counts_per_rev
        return screw_revs * self.lead

    def get_position(self) -> float:
        """
        Get the current measured position from encoder counts.

        This is the encoder-derived position — it will differ slightly
        from the true position due to quantization.

        Returns:
            Measured position (metres). EXACT (quantized).
        """
        return self.counts_to_position(self._accumulated_counts)

    @staticmethod
    def position_to_counts_exact(position_m: float, lead: float,
                                  counts_per_rev: int) -> float:
        """
        Compute exact (non-quantized) counts for a given position.
        Utility for testing.

        Equation (§5): counts = (position / lead) × counts_per_rev

        Args:
            position_m: Linear position (metres).
            lead: Ballscrew lead (metres/rev).
            counts_per_rev: Encoder counts per revolution.

        Returns:
            Exact counts (float, not quantized). EXACT.
        """
        screw_revs = position_m / lead
        return screw_revs * counts_per_rev
