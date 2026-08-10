"""
sil_sim.motion_cueing.mca_pipeline — Motion Cueing Algorithm pipeline.

Chains: world→body rotation → HPF (transient extraction) → tilt coordination
        → split washout → target pose.

Input:  TelemetryFrame (acceleration, angular velocity, orientation)
Output: 6-DOF target platform pose [x, y, z, roll, pitch, yaw] in SI units.

The MCA converts vehicle motion signals into physically-realisable platform
commands that stay within the Stewart platform's limited workspace.

Output class: EXACT — pure signal processing.
"""

import numpy as np
import math
from typing import Dict, Any

from sil_sim.telemetry.base import TelemetryFrame
from sil_sim.motion_cueing.hpf import HighPassFilter, create_hpf_from_config
from sil_sim.motion_cueing.tilt_coordination import TiltCoordination, create_tilt_from_config
from sil_sim.motion_cueing.washout import WashoutFilter, create_split_washout_from_config
from sil_sim.motion_cueing.frame_transform import rotate_world_to_body


class MCAPipeline:
    """
    Motion Cueing Algorithm pipeline.

    Processing flow:
        0. Rotate world-frame telemetry into vehicle body frame
        1. HPF on translational acceleration → extract transients
        2. Sustained accel (input − HPF output) → tilt coordination
        3. Transient accel → double-integrate → translational displacement
        4. Angular velocity → integrate → rotational displacement
        5. Add tilt coordination angles to rotational displacement
        6. Split washout filters on translation/rotation → target pose
           (separate time constants so tilt-coordination cues persist
           longer than translational transients)

    The output target pose is in the platform's workspace frame,
    relative to the neutral pose.

    NOTE: At t=0, a SafetyViolation(actuator_acceleration) may fire
    due to the cold-start transient (step from 0 to initial velocity).
    This is expected initial-condition behavior, not a systemic bug —
    it clears after the first frame.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Args:
            config: Validated MCA config dict (from loader.load_mca()).
        """
        self._config = config
        self.dt = 1.0 / config['input_rate_hz']

        # Body-frame rotation mode: "full" (default) or "yaw_only" (debug)
        self._rotation_mode = config.get('body_frame_rotation', 'full')

        # Sub-modules
        self.hpf = create_hpf_from_config(config)
        # Create a second HPF for rotational channels using the same factory
        rot_config = {'hpf': config['hpf_rotation'], 'input_rate_hz': config['input_rate_hz']}
        self.hpf_rot = create_hpf_from_config(rot_config)
        
        self.tilt = create_tilt_from_config(config)
        
        # Scaling gains
        gains = config['gains']
        self.gains_trans = np.array([gains['surge'], gains['sway'], gains['heave']])
        self.gains_rot = np.array([gains['roll'], gains['pitch'], gains['yaw']])
        
        # Dead zones
        self.dead_zone_accel = config['dead_zone']['acceleration_m_s2']
        self.dead_zone_omega = config['dead_zone']['angular_velocity_rad_s']
        
        # Rate limits
        self.max_trans_rate = config['pose_rate_limits']['max_translation_rate_m_s']
        self.max_rot_rate = config['pose_rate_limits']['max_rotation_rate_rad_s']
        self._prev_target_pose = np.zeros(6)

        # Split washout: independent time constants for translation vs rotation.
        # This prevents the fast translational washout from decaying
        # tilt-coordination signals that should persist during sustained turns.
        self.washout_trans, self.washout_rot = create_split_washout_from_config(config)

        # Integration state for translational displacement
        self._velocity_transient = np.zeros(3)  # Integrated transient accel → velocity
        self._displacement = np.zeros(3)         # Integrated velocity → displacement

        # Integration state for rotational displacement
        self._rotation = np.zeros(3)  # Integrated angular velocity

        # Previous timestamp for actual per-frame dt computation
        self._prev_timestamp = None

    def reset(self) -> None:
        """Reset all MCA state to zero."""
        self.hpf.reset()
        self.tilt.reset()
        self.washout_trans.reset()
        self.washout_rot.reset()
        self._velocity_transient[:] = 0.0
        self._displacement[:] = 0.0
        self._rotation[:] = 0.0
        self._prev_timestamp = None
        self._prev_target_pose[:] = 0.0

    def update(self, frame: TelemetryFrame) -> np.ndarray:
        """
        Process one telemetry frame through the MCA pipeline.

        Args:
            frame: TelemetryFrame with acceleration, angular velocity,
                   orientation (all in world frame from CARLA/CSV).

        Returns:
            (6,) array — target platform pose [x, y, z, roll, pitch, yaw]
            relative to neutral, in SI units (m, rad). EXACT.
        """
        # --- Compute actual per-frame dt ---
        # Use actual timestamp deltas when available (handles jittery CSV
        # timing, e.g. ~10.3–10.8 ms instead of a fixed 1/input_rate_hz).
        # This prevents drift in the integrators (velocity/displacement).
        #
        # NOTE: The HPF and Washout filters are initialized once with fixed
        # coefficients derived from the config `input_rate_hz`. While their
        # effective cutoff frequencies will jitter slightly (~5%) relative to
        # this true per-frame dt, it's acceptable for signal filtering. The
        # integrators, however, MUST use this true dt to avoid unbounded drift.
        #
        # Falls back to config dt for the first frame or if timestamps
        # are non-monotonic.
        if self._prev_timestamp is not None and frame.timestamp > self._prev_timestamp:
            dt = frame.timestamp - self._prev_timestamp
        else:
            dt = self.dt
        self._prev_timestamp = frame.timestamp

        # --- Step 0: Rotate world-frame telemetry into vehicle body frame ---
        # CARLA (and CSV recordings of CARLA data) report acceleration and
        # angular velocity in the world/global coordinate frame. The MCA
        # needs body-frame quantities: acc_x = forward, acc_y = lateral,
        # acc_z = vertical relative to the car, not the compass.
        # Without this rotation, tilt coordination and translational cues
        # reflect compass heading instead of what the driver's body feels.
        accel, ang_vel = rotate_world_to_body(
            frame.accel_xyz,
            frame.ang_vel_xyz,
            frame.orientation_rpy,
            mode=self._rotation_mode,
        )
        
        # Dead zone
        accel = np.where(np.abs(accel) < self.dead_zone_accel, 0.0, accel)
        ang_vel = np.where(np.abs(ang_vel) < self.dead_zone_omega, 0.0, ang_vel)
        
        # Scaling gains
        accel = accel * self.gains_trans
        ang_vel = ang_vel * self.gains_rot

        # --- Step 1: HPF on translational acceleration ---
        accel_transient = self.hpf.update(accel)

        # --- Step 2: Sustained acceleration (for tilt coordination) ---
        accel_sustained = accel - accel_transient
        # Only X and Y are used for tilt (Z is vertical — no tilt mapping)
        tilt_angles = self.tilt.update(accel_sustained[:2])
        # tilt_angles = [pitch_from_ax, roll_from_ay]

        # --- Step 3: Double-integrate transient accel → displacement ---
        # Pure double integration without leaky alpha.
        # The translational washout filter will decay this accumulated displacement back to neutral.
        self._velocity_transient += accel_transient * dt
        displacement_delta = self._velocity_transient * dt
        
        # --- Step 4: HPF and Integrate angular velocity → rotation ---
        ang_vel_transient = self.hpf_rot.update(ang_vel)
        rotation_delta = ang_vel_transient * dt

        # --- Step 5: Combine into 6-DOF pose ---
        # Translation: [x, y, z]
        trans_delta = displacement_delta

        # Rotation: [roll, pitch, yaw]
        rot_delta = rotation_delta

        # --- Step 6: Split washout filters → decay toward neutral ---
        # Translation and rotation use INDEPENDENT time constants.
        # Translational washout decays fast (limited actuator stroke).
        # Rotational washout decays slower (preserves tilt-coordination
        # cues during sustained cornering — the whole reason tilt
        # coordination exists).
        trans_washed = self.washout_trans.update(trans_delta)
        rot_washed = self.washout_rot.update(rot_delta)

        # --- Step 7: Add tilt coordination ---
        # Tilt coordination adds a sustained absolute angle (not a delta)
        # to simulate sustained g-forces. It should not be washed out.
        rot_washed[0] += tilt_angles[1]  # Roll + tilt from ay
        rot_washed[1] += tilt_angles[0]  # Pitch + tilt from ax

        target_pose = np.concatenate([trans_washed, rot_washed])

        # --- Step 8: Rate limits and Workspace Clamping ---
        # 8a: Rate limiting
        pose_rate = (target_pose - self._prev_target_pose) / dt if dt > 0 else np.zeros(6)
        
        # Clip rates
        trans_rate_clipped = np.clip(pose_rate[:3], -self.max_trans_rate, self.max_trans_rate)
        rot_rate_clipped = np.clip(pose_rate[3:], -self.max_rot_rate, self.max_rot_rate)
        
        target_pose[:3] = self._prev_target_pose[:3] + trans_rate_clipped * dt
        target_pose[3:] = self._prev_target_pose[3:] + rot_rate_clipped * dt
        
        # 8b: Workspace clamping (limits passed via config or injected dynamically)
        if 'workspace_limits' in getattr(self, '_config', {}):
            limits = self._config['workspace_limits']
            max_trans = limits['max_translation_m']
            max_rot = math.radians(limits['max_rotation_deg'])
            target_pose[:3] = np.clip(target_pose[:3], -max_trans, max_trans)
            target_pose[3:] = np.clip(target_pose[3:], -max_rot, max_rot)

        self._prev_target_pose = target_pose.copy()

        # Add neutral height (Z offset from base)
        # This is handled by the IK module (neutral pose),
        # so the MCA output is RELATIVE to neutral.

        return target_pose
