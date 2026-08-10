"""
integration.protocol — WebSocket message protocol.

Defines the structured JSON message sent from the Python SIL simulator
to the browser at ~60 Hz.

UNIT CONVENTION AT THE INTERFACE:
    Python → WebSocket: SI units (metres, radians)
    WebSocket message: Contains BOTH SI and display units explicitly labelled
    Browser receives: Uses mm/degrees for display

Every field has an explicit unit suffix in its key name to prevent
hidden unit conversions. The browser must NOT assume any units.

Message schema version: 1.0
"""
import json
import math
import numpy as np
from typing import Dict, Any, List, Optional


# Protocol version for forward compatibility
PROTOCOL_VERSION = "1.0"


def build_ws_message(
    timestamp: float,
    frame_index: int,
    scenario_name: str,
    mode: str,
    # Vehicle state (SI units)
    vehicle_accel_xyz_ms2: np.ndarray,    # [ax, ay, az] m/s²
    vehicle_ang_vel_xyz_rads: np.ndarray, # [wx, wy, wz] rad/s
    vehicle_orientation_rpy_rad: np.ndarray,  # [roll, pitch, yaw] rad
    vehicle_velocity_xyz_ms: np.ndarray,  # [vx, vy, vz] m/s
    # Motion cueing output (SI units)
    platform_pose_m_rad: np.ndarray,      # [x, y, z, roll, pitch, yaw] m, rad
    # Stewart IK output (SI units)
    leg_lengths_m: np.ndarray,            # [L1..L6] metres
    # Platform joint positions (SI units) for direct 3D rendering
    base_joints_m: np.ndarray,            # (6, 3) base joints in metres
    top_joints_m: np.ndarray,             # (6, 3) top joints in metres
    # Actuator states (SI units)
    actuator_positions_m: List[float],    # [pos1..pos6] metres
    actuator_velocities_ms: List[float],  # [vel1..vel6] m/s
    actuator_errors_m: List[float],       # [err1..err6] metres
    encoder_counts: List[int],            # raw counts
    # Safety
    safety_valid: bool,
    safety_warnings: List[str],
    safety_estop: bool,
    # Simulation metadata
    sim_freq_hz: float,
    carla_connected: bool,
    phase: str = '',
) -> str:
    """
    Build a JSON-encoded WebSocket message.

    All values are serialized with explicit unit labels.
    Browser must read these unit labels and not assume any convention.

    Returns:
        JSON string ready to send over WebSocket.

    UNITS SUMMARY:
        Positions:   metres (m) in Python, mm in browser display
        Angles:      radians (rad) in Python, degrees (deg) in browser display
        Velocities:  m/s in Python, mm/s in browser display
        Accelerations: m/s² everywhere
        Leg lengths: metres in Python, mm in browser display
        Encoder:     raw counts (dimensionless)
    """
    # Neutral leg length in metres (1672.425 mm → 1.672425 m neutral height)
    # Neutral lengths are computed by the geometry module; for protocol we include them
    # Browser uses mm display: multiply metres by 1000

    r2d = 180.0 / math.pi  # radians to degrees conversion

    msg = {
        "protocol_version": PROTOCOL_VERSION,
        "timestamp_s": float(timestamp),
        "frame_index": int(frame_index),
        "scenario": scenario_name,
        "mode": mode,                   # "synthetic" | "carla" | "manual"
        "phase": phase,
        "carla_connected": carla_connected,

        # --- Vehicle state ---
        # SI units. Browser converts for display only.
        "vehicle": {
            "accel_x_ms2": float(vehicle_accel_xyz_ms2[0]),   # longitudinal (X-fwd)
            "accel_y_ms2": float(vehicle_accel_xyz_ms2[1]),   # lateral (Y-left)
            "accel_z_ms2": float(vehicle_accel_xyz_ms2[2]),   # vertical (Z-up)
            "ang_vel_x_rads": float(vehicle_ang_vel_xyz_rads[0]),  # roll rate
            "ang_vel_y_rads": float(vehicle_ang_vel_xyz_rads[1]),  # pitch rate
            "ang_vel_z_rads": float(vehicle_ang_vel_xyz_rads[2]),  # yaw rate
            "roll_rad": float(vehicle_orientation_rpy_rad[0]),
            "pitch_rad": float(vehicle_orientation_rpy_rad[1]),
            "yaw_rad": float(vehicle_orientation_rpy_rad[2]),
            "vel_x_ms": float(vehicle_velocity_xyz_ms[0]),    # forward speed
            "vel_y_ms": float(vehicle_velocity_xyz_ms[1]),    # lateral speed
            "vel_z_ms": float(vehicle_velocity_xyz_ms[2]),    # vertical speed
            # Convenience display values (converted, labelled)
            "speed_kmh": float(vehicle_velocity_xyz_ms[0] * 3.6),
            "roll_deg": float(vehicle_orientation_rpy_rad[0] * r2d),
            "pitch_deg": float(vehicle_orientation_rpy_rad[1] * r2d),
            "yaw_deg": float(vehicle_orientation_rpy_rad[2] * r2d),
            "ang_vel_z_degs": float(vehicle_ang_vel_xyz_rads[2] * r2d),
        },

        # --- Motion cueing output (desired platform pose) ---
        # SI units. This is the TARGET, not the actual actuator position.
        "motion_cueing": {
            "x_m": float(platform_pose_m_rad[0]),      # lateral surge (m)
            "y_m": float(platform_pose_m_rad[1]),      # sway (m)
            "z_m": float(platform_pose_m_rad[2]),      # heave above neutral (m)
            "roll_rad": float(platform_pose_m_rad[3]),
            "pitch_rad": float(platform_pose_m_rad[4]),
            "yaw_rad": float(platform_pose_m_rad[5]),
            # Display conversions (explicit labels)
            "x_mm": float(platform_pose_m_rad[0] * 1000.0),
            "y_mm": float(platform_pose_m_rad[1] * 1000.0),
            "z_mm": float(platform_pose_m_rad[2] * 1000.0),
            "roll_deg": float(platform_pose_m_rad[3] * r2d),
            "pitch_deg": float(platform_pose_m_rad[4] * r2d),
            "yaw_deg": float(platform_pose_m_rad[5] * r2d),
        },

        # --- Stewart platform IK output ---
        # leg_lengths are ABSOLUTE lengths (neutral + displacement)
        # The browser adds NEUTRAL_HEIGHT_M to get absolute leg positions for rendering.
        "stewart": {
            "L1_m": float(leg_lengths_m[0]),
            "L2_m": float(leg_lengths_m[1]),
            "L3_m": float(leg_lengths_m[2]),
            "L4_m": float(leg_lengths_m[3]),
            "L5_m": float(leg_lengths_m[4]),
            "L6_m": float(leg_lengths_m[5]),
            # Display conversions
            "L1_mm": float(leg_lengths_m[0] * 1000.0),
            "L2_mm": float(leg_lengths_m[1] * 1000.0),
            "L3_mm": float(leg_lengths_m[2] * 1000.0),
            "L4_mm": float(leg_lengths_m[3] * 1000.0),
            "L5_mm": float(leg_lengths_m[4] * 1000.0),
            "L6_mm": float(leg_lengths_m[5] * 1000.0),
        },

        # --- 3D joint positions for browser rendering ---
        # These allow the browser to directly position actuator endpoints
        # WITHOUT running IK. Python IK is the single source of truth.
        # Units: metres (browser multiplies by 1000 to get mm)
        "joints": {
            "base_m": [list(map(float, pt)) for pt in base_joints_m],   # (6, 3)
            "top_m": [list(map(float, pt)) for pt in top_joints_m],     # (6, 3)
            # Also provide mm for convenience
            "base_mm": [[float(v * 1000.0) for v in pt] for pt in base_joints_m],
            "top_mm": [[float(v * 1000.0) for v in pt] for pt in top_joints_m],
        },

        # --- Actuator SIL states ---
        "actuators": {
            "positions_m": [float(p) for p in actuator_positions_m],
            "velocities_ms": [float(v) for v in actuator_velocities_ms],
            "position_errors_m": [float(e) for e in actuator_errors_m],
            "encoder_counts": [int(c) for c in encoder_counts],
            # Display
            "positions_mm": [float(p * 1000.0) for p in actuator_positions_m],
            "velocities_mms": [float(v * 1000.0) for v in actuator_velocities_ms],
            "position_errors_mm": [float(e * 1000.0) for e in actuator_errors_m],
        },

        # --- Safety ---
        "safety": {
            "valid": bool(safety_valid),
            "estop": bool(safety_estop),
            "warnings": list(safety_warnings),
            "status": "ESTOP" if safety_estop else ("WARNING" if safety_warnings else "NORMAL"),
        },

        # --- Simulation metadata ---
        "sim": {
            "rate_hz": float(sim_freq_hz),
            "realtime": True,
        },
    }

    return json.dumps(msg, allow_nan=False)


def validate_message(msg: Dict[str, Any]) -> List[str]:
    """
    Validate a decoded WebSocket message dict.

    Returns:
        List of error strings. Empty = valid.
    """
    errors = []
    required_keys = ['timestamp_s', 'vehicle', 'motion_cueing', 'stewart', 'joints', 'safety']
    for key in required_keys:
        if key not in msg:
            errors.append(f"Missing required key: {key}")

    # Check for NaN/Infinity
    def check_numeric(obj, path=''):
        if isinstance(obj, float):
            if math.isnan(obj) or math.isinf(obj):
                errors.append(f"NaN or Inf at {path}")
        elif isinstance(obj, dict):
            for k, v in obj.items():
                check_numeric(v, f"{path}.{k}")
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                check_numeric(v, f"{path}[{i}]")

    check_numeric(msg)

    return errors
