"""
sil_sim.config.loader — Configuration loading and validation.

Loads YAML config files and validates that all required fields are present
and have the correct types. Fails loudly on missing or malformed parameters.

All numeric values are expected in SI units (metres, seconds, radians, amps, etc.).
"""

import os
import yaml
import math
from typing import Any, Dict, List, Optional


# Path to the config directory (relative to this file)
_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))


class ConfigError(Exception):
    """Raised when a config file is missing, malformed, or has invalid values."""
    pass


def _load_yaml(filename: str) -> Dict[str, Any]:
    """Load a YAML file from the config directory."""
    filepath = os.path.join(_CONFIG_DIR, filename)
    if not os.path.exists(filepath):
        raise ConfigError(
            "Config file not found: {}. Expected at: {}".format(filename, filepath)
        )
    with open(filepath, 'r') as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ConfigError(
                "Malformed YAML in {}: {}".format(filename, e)
            )
    if data is None:
        raise ConfigError("Config file {} is empty".format(filename))
    return data


def _require(data: Dict, keys: List[str], filename: str) -> None:
    """Validate that all required keys exist in the config dict (supports dotted paths)."""
    for key_path in keys:
        parts = key_path.split('.')
        current = data
        for part in parts:
            if not isinstance(current, dict) or part not in current:
                raise ConfigError(
                    "Missing required config key '{}' in {}".format(key_path, filename)
                )
            current = current[part]
        if current is None:
            raise ConfigError(
                "Config key '{}' in {} is null — must have a value".format(
                    key_path, filename
                )
            )


def _require_positive(data: Dict, key_path: str, filename: str) -> None:
    """Validate a numeric config value is positive."""
    parts = key_path.split('.')
    current = data
    for part in parts:
        current = current[part]
    if not isinstance(current, (int, float)) or current <= 0:
        raise ConfigError(
            "Config key '{}' in {} must be a positive number, got: {}".format(
                key_path, filename, current
            )
        )


def _require_non_negative(data: Dict, key_path: str, filename: str) -> None:
    """Validate a numeric config value is non-negative."""
    parts = key_path.split('.')
    current = data
    for part in parts:
        current = current[part]
    if not isinstance(current, (int, float)) or current < 0:
        raise ConfigError(
            "Config key '{}' in {} must be non-negative, got: {}".format(
                key_path, filename, current
            )
        )


def load_geometry() -> Dict[str, Any]:
    """Load and validate geometry.yaml."""
    filename = "geometry.yaml"
    data = _load_yaml(filename)
    _require(data, [
        'euler_convention',
        'base.plate_diameter_m',
        'base.joints_xy_m',
        'platform.plate_diameter_m',
        'platform.joints_xy_m',
        'neutral_height_m',
        'joint_hardware_limit_deg',
        'joint_operating_margin_deg',
        'workspace_limits.max_translation_x_m',
        'workspace_limits.max_translation_y_m',
        'workspace_limits.max_translation_z_m',
        'workspace_limits.max_roll_deg',
        'workspace_limits.max_pitch_deg',
        'workspace_limits.max_yaw_deg',
    ], filename)

    # Validate joints_xy_m: must be 6 pairs of [x, y] coordinates
    for key in ['base.joints_xy_m', 'platform.joints_xy_m']:
        parts = key.split('.')
        joints = data[parts[0]][parts[1]]
        if not isinstance(joints, list) or len(joints) != 6:
            raise ConfigError(
                f"'{key}' in {filename} must be a list of 6 coordinate pairs"
            )
        for i, pair in enumerate(joints):
            if not isinstance(pair, list) or len(pair) != 2:
                raise ConfigError(
                    f"'{key}[{i}]' in {filename} must be a list of 2 coordinates [x, y]"
                )

    _require_positive(data, 'base.plate_diameter_m', filename)
    _require_positive(data, 'platform.plate_diameter_m', filename)
    _require_positive(data, 'neutral_height_m', filename)
    _require_positive(data, 'joint_hardware_limit_deg', filename)
    _require_non_negative(data, 'joint_operating_margin_deg', filename)

    return data


def load_actuator() -> Dict[str, Any]:
    """Load and validate actuator.yaml."""
    filename = "actuator.yaml"
    data = _load_yaml(filename)
    _require(data, [
        'ballscrew.lead_m',
        'ballscrew.overall_length_m',
        'ballscrew.estimated_usable_stroke_m',
        'motor.steps_per_rev',
        'motor.rated_current_a',
        'driver.microsteps_per_fullstep',
        'coupling.ratio',
        'encoder.ppr',
        'encoder.quadrature_multiplier',
        'encoder.counts_per_rev',
    ], filename)

    _require_positive(data, 'ballscrew.lead_m', filename)
    _require_positive(data, 'motor.steps_per_rev', filename)
    _require_positive(data, 'driver.microsteps_per_fullstep', filename)
    _require_positive(data, 'encoder.counts_per_rev', filename)

    # Validate coupling ratio is 1:1 (confirmed spec)
    ratio = data['coupling']['ratio']
    if ratio != 1.0:
        raise ConfigError(
            "Coupling ratio in {} must be 1.0 (confirmed 1:1), got: {}".format(
                filename, ratio
            )
        )

    # Validate counts_per_rev consistency
    expected_counts = data['encoder']['ppr'] * data['encoder']['quadrature_multiplier']
    actual_counts = data['encoder']['counts_per_rev']
    if expected_counts != actual_counts:
        raise ConfigError(
            "Encoder counts_per_rev ({}) != ppr ({}) × quadrature_multiplier ({}) = {} in {}".format(
                actual_counts, data['encoder']['ppr'],
                data['encoder']['quadrature_multiplier'],
                expected_counts, filename
            )
        )

    return data


def load_mca() -> Dict[str, Any]:
    """Load and validate mca.yaml."""
    filename = "mca.yaml"
    data = _load_yaml(filename)
    _require(data, [
        'gains.surge',
        'gains.sway',
        'gains.heave',
        'gains.roll',
        'gains.pitch',
        'gains.yaw',
        'hpf.cutoff_freq_hz',
        'hpf.order',
        'hpf_rotation.cutoff_freq_hz',
        'hpf_rotation.order',
        'tilt_coordination.gain',
        'tilt_coordination.rate_limit_rad_s',
        'tilt_coordination.max_tilt_deg',
        'dead_zone.acceleration_m_s2',
        'dead_zone.angular_velocity_rad_s',
        'pose_rate_limits.max_translation_rate_m_s',
        'pose_rate_limits.max_rotation_rate_rad_s',
        'washout',
        'input_rate_hz',
        'output_rate_hz',
    ], filename)

    _require_positive(data, 'hpf.cutoff_freq_hz', filename)
    _require_positive(data, 'hpf_rotation.cutoff_freq_hz', filename)
    _require_positive(data, 'input_rate_hz', filename)
    _require_positive(data, 'tilt_coordination.max_tilt_deg', filename)
    _require_non_negative(data, 'dead_zone.acceleration_m_s2', filename)
    _require_non_negative(data, 'dead_zone.angular_velocity_rad_s', filename)
    
    # Compute max tilt rad for internal use
    data['tilt_coordination']['max_tilt_rad'] = math.radians(data['tilt_coordination']['max_tilt_deg'])

    # Validate washout config: support both legacy flat and new split format
    washout_cfg = data['washout']
    if 'translation' in washout_cfg and 'rotation' in washout_cfg:
        # New split format
        _require(data, [
            'washout.translation.time_constant_s',
            'washout.rotation.time_constant_s',
        ], filename)
        _require_positive(data, 'washout.translation.time_constant_s', filename)
        _require_positive(data, 'washout.rotation.time_constant_s', filename)
    elif 'time_constant_s' in washout_cfg:
        # Legacy flat format
        _require_positive(data, 'washout.time_constant_s', filename)
    else:
        raise ConfigError(
            "washout config in {} must have either "
            "'time_constant_s' (legacy) or 'translation'+'rotation' (split) "
            "sub-keys".format(filename)
        )

    # Validate body_frame_rotation mode (optional, defaults to "full")
    mode = data.get('body_frame_rotation', 'full')
    if mode not in ('full', 'yaw_only'):
        raise ConfigError(
            "body_frame_rotation in {} must be 'full' or 'yaw_only', "
            "got: '{}'".format(filename, mode)
        )

    return data


def load_pid() -> Dict[str, Any]:
    """Load and validate pid.yaml."""
    filename = "pid.yaml"
    data = _load_yaml(filename)
    _require(data, [
        'gains.kp',
        'gains.ki',
        'gains.kd',
        'output_limits.min_output_m_s',
        'output_limits.max_output_m_s',
        'anti_windup.enabled',
        'anti_windup.method',
        'control_rate_hz',
    ], filename)

    _require_positive(data, 'control_rate_hz', filename)

    # Validate output limits are ordered
    lo = data['output_limits']['min_output_m_s']
    hi = data['output_limits']['max_output_m_s']
    if lo >= hi:
        raise ConfigError(
            "PID output_limits: min ({}) must be < max ({}) in {}".format(
                lo, hi, filename
            )
        )

    return data


def load_electrical() -> Dict[str, Any]:
    """Load and validate electrical.yaml."""
    filename = "electrical.yaml"
    data = _load_yaml(filename)
    _require(data, [
        'psu.bus_voltage_v',
        'psu.bus_capacity_a',
        'psu.bus_capacity_w',
        'logic_supply.voltage_v',
        'logic_supply.estimated_draw_a',
        'motor.kt_estimated_nm_per_a',
        'motor.rated_current_a',
        'efficiencies.eta_screw',
        'efficiencies.eta_driver',
        'payload.total_payload_kg',
        'payload.platform_mass_kg',
        'limits.max_phase_current_a',
        'limits.max_bus_current_total_a',
        'limits.max_bus_power_w',
    ], filename)

    _require_positive(data, 'psu.bus_voltage_v', filename)
    _require_positive(data, 'motor.kt_estimated_nm_per_a', filename)
    _require_positive(data, 'payload.total_payload_kg', filename)

    # Validate efficiencies are in (0, 1]
    for key in ['efficiencies.eta_screw', 'efficiencies.eta_driver']:
        parts = key.split('.')
        val = data[parts[0]][parts[1]]
        if not isinstance(val, (int, float)) or val <= 0 or val > 1.0:
            raise ConfigError(
                "'{}' in {} must be in (0, 1.0], got: {}".format(key, filename, val)
            )

    return data


def load_safety() -> Dict[str, Any]:
    """Load and validate safety.yaml."""
    filename = "safety.yaml"
    data = _load_yaml(filename)
    _require(data, [
        'actuator.estimated_usable_stroke_m',
        'actuator.stroke_midpoint_ratio',
        'workspace.max_translation_m',
        'workspace.max_rotation_deg',
        'power.max_bus_current_a',
        'power.max_bus_power_w',
        'power.max_phase_current_a',
    ], filename)

    _require_positive(data, 'actuator.estimated_usable_stroke_m', filename)
    ratio = data['actuator']['stroke_midpoint_ratio']
    if ratio < 0.0 or ratio > 1.0:
        raise ConfigError(f"'actuator.stroke_midpoint_ratio' must be in [0.0, 1.0], got {ratio}")

    return data


def load_carla() -> Dict[str, Any]:
    """Load and validate carla.yaml."""
    filename = "carla.yaml"
    data = _load_yaml(filename)
    _require(data, [
        'connection.host',
        'connection.port',
        'connection.timeout_s',
        'telemetry.rate_hz',
        'telemetry.convert_to_specific_force',
        'fallback.mode',
    ], filename)
    return data


def load_all() -> Dict[str, Dict[str, Any]]:
    """Load and validate all config files. Returns a dict of config dicts."""
    configs = {
        'geometry': load_geometry(),
        'actuator': load_actuator(),
        'mca': load_mca(),
        'pid': load_pid(),
        'electrical': load_electrical(),
        'safety': load_safety(),
        'carla': load_carla(),
    }
    
    # Cross-file derivation
    # Compute neutral leg lengths from geometry
    from sil_sim.kinematics.geometry import StewartGeometry
    geom = StewartGeometry(configs['geometry'])
    
    # Inject computed lengths into actuator config (as a list for per-leg homing)
    configs['actuator']['home_leg_lengths_m'] = [float(l) for l in geom.home_leg_lengths]
    
    # Compute and inject dynamic per-leg safety limits
    stroke = configs['safety']['actuator']['estimated_usable_stroke_m']
    midpoint_ratio = configs['safety']['actuator']['stroke_midpoint_ratio']
    
    min_lengths = []
    max_lengths = []
    for home_len in geom.home_leg_lengths:
        min_lengths.append(float(home_len - (stroke * midpoint_ratio)))
        max_lengths.append(float(home_len + (stroke * (1.0 - midpoint_ratio))))
        
    configs['safety']['actuator']['min_leg_lengths_m'] = min_lengths
    configs['safety']['actuator']['max_leg_lengths_m'] = max_lengths
    
    return configs
