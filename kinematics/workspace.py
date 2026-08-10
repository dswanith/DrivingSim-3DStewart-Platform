"""
sil_sim.kinematics.workspace — Workspace feasibility checking.

Checks whether a given platform pose is within the kinematic workspace
of the Stewart platform, considering:
    1. Actuator stroke limits (leg length min/max)
    2. Joint articulation limits (rod-end ±25°)
    3. Platform translation limits (saturation)
    4. Platform rotation limits (saturation)

Returns which specific constraint(s) failed.

Output class: EXACT — geometric checks only.
"""

import math
import numpy as np
from typing import Dict, Any, List, Tuple

from sil_sim.kinematics.geometry import StewartGeometry


class WorkspaceViolation:
    """Describes a single workspace constraint violation."""

    def __init__(self, constraint: str, detail: str, value: float, limit: float):
        self.constraint = constraint
        self.detail = detail
        self.value = value
        self.limit = limit

    def __repr__(self):
        return "WorkspaceViolation({}: {} — value={:.6f}, limit={:.6f})".format(
            self.constraint, self.detail, self.value, self.limit
        )

    def __str__(self):
        return self.__repr__()


def check_workspace(
    pose: np.ndarray,
    leg_lengths: np.ndarray,
    geometry: StewartGeometry,
    safety_config: Dict[str, Any],
) -> Tuple[bool, List[WorkspaceViolation]]:
    """
    Check if a pose is within the Stewart platform's workspace.

    Args:
        pose: [x, y, z, roll, pitch, yaw] in SI (m, rad).
        leg_lengths: (6,) array of computed leg lengths (m).
        geometry: StewartGeometry instance.
        safety_config: Validated safety config dict.

    Returns:
        (feasible, violations):
            feasible: True if all constraints pass.
            violations: List of WorkspaceViolation objects (empty if feasible).
    """
    violations = []

    # --- 1. Actuator stroke limits ---
    min_lens = safety_config['actuator']['min_leg_lengths_m']
    max_lens = safety_config['actuator']['max_leg_lengths_m']
    for i in range(6):
        if leg_lengths[i] < min_lens[i]:
            violations.append(WorkspaceViolation(
                constraint="actuator_stroke",
                detail="Leg {} length {:.6f} m < min {:.6f} m".format(
                    i + 1, leg_lengths[i], min_lens[i]
                ),
                value=leg_lengths[i],
                limit=min_lens[i],
            ))
        if leg_lengths[i] > max_lens[i]:
            violations.append(WorkspaceViolation(
                constraint="actuator_stroke",
                detail="Leg {} length {:.6f} m > max {:.6f} m".format(
                    i + 1, leg_lengths[i], max_lens[i]
                ),
                value=leg_lengths[i],
                limit=max_lens[i],
            ))

    # --- 2. Joint articulation limits (rod-end ±25°) ---
    # The rod-end allows ±25° from its NEUTRAL (mounted) position.
    # Compute the joint angles at neutral as the baseline, then check
    # how far the current pose deviates from that baseline.
    neutral_pose = np.array([0.0, 0.0, geometry.neutral_height, 0.0, 0.0, 0.0])
    neutral_angles = geometry.compute_joint_angles(neutral_pose)
    current_angles = geometry.compute_joint_angles(pose)
    max_joint_deviation = geometry.joint_max_angle_rad
    for i in range(6):
        deviation = abs(current_angles[i] - neutral_angles[i])
        if deviation > max_joint_deviation:
            violations.append(WorkspaceViolation(
                constraint="joint_articulation",
                detail="Leg {} joint deviation {:.2f}° > max ±{:.2f}°".format(
                    i + 1,
                    math.degrees(deviation),
                    math.degrees(max_joint_deviation),
                ),
                value=deviation,
                limit=max_joint_deviation,
            ))

    # --- 3. Translation limits ---
    # Check displacement from NEUTRAL pose, not absolute coordinates.
    # X and Y neutral are 0; Z neutral is geometry.neutral_height.
    wl = geometry.workspace_limits
    displacements = [
        ('max_tx', 'X translation', pose[0] - 0.0),           # X displacement from neutral (0)
        ('max_ty', 'Y translation', pose[1] - 0.0),           # Y displacement from neutral (0)
        ('max_tz', 'Z translation', pose[2] - geometry.neutral_height),  # Z displacement from neutral
    ]
    for key, label, disp in displacements:
        val = abs(disp)
        limit = wl[key]
        if val > limit:
            violations.append(WorkspaceViolation(
                constraint="translation_limit",
                detail="{} |{:.6f}| m > max {:.6f} m".format(label, disp, limit),
                value=val,
                limit=limit,
            ))

    # --- 4. Rotation limits ---
    # Note: pose[2] is Z translation; pose[3:6] are roll/pitch/yaw
    labels_r = [
        ('max_roll', 'Roll', 3),
        ('max_pitch', 'Pitch', 4),
        ('max_yaw', 'Yaw', 5),
    ]
    for key, label, idx in labels_r:
        val = abs(pose[idx])
        limit = wl[key]
        if val > limit:
            violations.append(WorkspaceViolation(
                constraint="rotation_limit",
                detail="{} |{:.4f}| rad ({:.2f}°) > max {:.4f} rad ({:.2f}°)".format(
                    label, pose[idx], math.degrees(pose[idx]),
                    limit, math.degrees(limit),
                ),
                value=val,
                limit=limit,
            ))

    feasible = len(violations) == 0
    return feasible, violations
