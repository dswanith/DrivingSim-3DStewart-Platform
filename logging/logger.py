"""
sil_sim.logging.logger — Simulation data logger (CSV + JSON).

Logs per-timestep simulation state including:
    - Telemetry (accel, velocity, angular velocity, orientation)
    - Target 6-DOF pose
    - Leg lengths L1–L6
    - Per-actuator: position, velocity, RPM, step frequency,
      STEP/DIR/ENABLE state, encoder counts, position error
    - Electrical estimates (motor current, bus current, power) — all ESTIMATED
    - Status/errors

Every electrical value is labeled ESTIMATED in column headers and JSON keys.
No value is ever labeled MEASURED.
"""

import os
import csv
import json
import time
import logging
from typing import Dict, Any, List, Optional

logger_log = logging.getLogger(__name__)


class SimLogger:
    """
    CSV + JSON logger for simulation timestep data.

    Creates two files per run:
        - {prefix}_log.csv: Flat CSV with all fields
        - {prefix}_log.jsonl: JSON Lines, one JSON object per timestep
    """

    # CSV column definitions — ESTIMATED fields explicitly labeled
    COLUMNS = [
        'timestamp_s',
        # Telemetry
        'telem_accel_x', 'telem_accel_y', 'telem_accel_z',
        'telem_ang_vel_x', 'telem_ang_vel_y', 'telem_ang_vel_z',
        'telem_roll', 'telem_pitch', 'telem_yaw',
        'telem_vel_x', 'telem_vel_y', 'telem_vel_z',
        # Target pose
        'target_x', 'target_y', 'target_z',
        'target_roll', 'target_pitch', 'target_yaw',
        # Leg lengths
        'L1_m', 'L2_m', 'L3_m', 'L4_m', 'L5_m', 'L6_m',
        # Per-actuator state (repeated for legs 1–6)
    ]

    # Per-leg column suffixes
    PER_LEG_SUFFIXES = [
        'position_m', 'velocity_m_s', 'rpm', 'step_freq_hz',
        'step_active', 'direction', 'enable', 'pulses',
        'encoder_counts', 'encoder_position_m', 'error_m',
        'phase_current_a_ESTIMATED',
    ]

    # Bus-level columns
    BUS_COLUMNS = [
        'bus_voltage_v',
        'total_bus_current_a_ESTIMATED',
        'total_driver_power_w_ESTIMATED',
        'total_system_power_w_ESTIMATED',
        'psu_utilization_ESTIMATED',
        'logic_power_w_ESTIMATED',
    ]

    # Per-leg bus current columns
    PER_LEG_BUS_SUFFIXES = ['bus_current_a_ESTIMATED']

    STATUS_COLUMNS = [
        'ik_feasible', 'workspace_status', 'safety_violations',
        'estop_state', 'sim_mode',
    ]

    def __init__(self, output_dir: str = "logs", prefix: str = "sim"):
        """
        Args:
            output_dir: Directory for log files.
            prefix: Filename prefix.
        """
        self.output_dir = output_dir
        self.prefix = prefix
        os.makedirs(output_dir, exist_ok=True)

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(output_dir, "{}_{}_log.csv".format(prefix, timestamp_str))
        self._json_path = os.path.join(output_dir, "{}_{}_log.jsonl".format(prefix, timestamp_str))

        # Build full column list
        self._columns = list(self.COLUMNS)
        for leg in range(1, 7):
            for suffix in self.PER_LEG_SUFFIXES:
                self._columns.append("leg{}_{}".format(leg, suffix))
        self._columns.extend(self.BUS_COLUMNS)
        for leg in range(1, 7):
            for suffix in self.PER_LEG_BUS_SUFFIXES:
                self._columns.append("leg{}_{}".format(leg, suffix))
        self._columns.extend(self.STATUS_COLUMNS)

        # Open files
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=self._columns,
                                           extrasaction='ignore')
        self._csv_writer.writeheader()

        self._json_file = open(self._json_path, 'w')

        self._row_count = 0
        logger_log.info("SimLogger initialized: CSV=%s, JSONL=%s",
                        self._csv_path, self._json_path)

    def log_timestep(self, data: Dict[str, Any]) -> None:
        """
        Log one timestep of simulation data.

        Args:
            data: Dict with keys matching column names.
                  Missing keys are written as empty strings in CSV.
        """
        # CSV
        self._csv_writer.writerow(data)

        # JSONL
        json_line = json.dumps(data, default=str)
        self._json_file.write(json_line + '\n')

        self._row_count += 1

        # Flush periodically
        if self._row_count % 100 == 0:
            self._csv_file.flush()
            self._json_file.flush()

    def close(self) -> None:
        """Close log files."""
        self._csv_file.close()
        self._json_file.close()
        logger_log.info("SimLogger closed: %d rows written to %s",
                        self._row_count, self._csv_path)

    @property
    def csv_path(self) -> str:
        return self._csv_path

    @property
    def json_path(self) -> str:
        return self._json_path

    @property
    def row_count(self) -> int:
        return self._row_count

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
