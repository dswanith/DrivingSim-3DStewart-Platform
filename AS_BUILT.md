# AS-BUILT DOCUMENTATION: Stewart Platform SIL Simulator (M6)

## 1. System Overview

The M6 Software-in-the-Loop (SIL) simulator is a comprehensive, deterministic modeling environment bridging the gap between vehicle telemetry (from sources like CARLA) and physical actuator commands for a 6-DOF Stewart Platform. It validates the full signal chain—including high-pass filtering, tilt coordination, inverse kinematics, PID position control, and electrical power estimation—purely in software before any hardware is energized. It explicitly does **NOT** provide real-time hardware-in-the-loop (HIL) execution, does **NOT** currently connect to a live CARLA server (only synthetic test traces), and does **NOT** model friction, backlash, or mechanical inertia (using a purely kinematic integrator for the plant model instead).

**Data-Flow Diagram:**
```text
[Telemetry Source] (CARLA / Synthetic)
      |
      | (X, Y, Z, Roll, Pitch, Yaw, v, a, omega) [World/UE Frame]
      v
[Telemetry Conversion] -> (Automotive right-handed frame)
      |
      | (Translation m, Rotation rad, Accel m/s²) [Vehicle Frame]
      v
[Motion Cueing Algorithm] (HPF, Washout, Tilt Coordination)
      |
      | (Target 6-DOF Pose: X, Y, Z, Roll, Pitch, Yaw) [Platform Frame]
      v
[Inverse Kinematics & Workspace] (Geometry Check)
      |
      | (6x Target Leg Lengths m) [Actuator Frame]
      v
[Actuator Controller] (PID + Pre-PID Stroke Clamping)
      |
      | (Velocity Command m/s -> RPM / Step Frequency Hz) [Motor Frame]
      v
[Electrical & Power Estimation] (Motor torque, Phase Current, Bus Power)
      |
      | (Estimated Current A, Power W, Status Flags) [Electrical Frame]
      v
[Logger & Visualization Dashboard] (CSV/JSONL/Websocket)
```

## 2. File-by-File Inventory

### Root & Configuration
* `sil_sim/__init__.py`: Package initialization. No public API.
* `sil_sim/config/__init__.py`: Package initialization. No public API.
* `sil_sim/config/loader.py`: Configuration loading and validation.
  - `ConfigError`: Raised on malformed configs.
  - `load_geometry()`, `load_actuator()`, `load_mca()`, `load_pid()`, `load_electrical()`, `load_safety()`, `load_carla()`: Validates individual YAML files.
  - `load_all()`: Loads all configs, handles cross-module injections (workspace limit unification). Reads: YAML files. Produces: Validated dicts.
* `sil_sim/config/actuator.yaml`: Hardware spec for ballscrew, motor, coupling, encoder.
* `sil_sim/config/carla.yaml`: CARLA telemetry connection parameters.
* `sil_sim/config/electrical.yaml`: PSU, logic supply, motor KT, efficiencies, payload estimates.
* `sil_sim/config/geometry.yaml`: Base/Platform bolt circles, joint angles, neutral height.
* `sil_sim/config/mca.yaml`: Cutoff frequencies, tilt gains, washout time constants.
* `sil_sim/config/pid.yaml`: PID gains, loop rate, anti-windup settings.
* `sil_sim/config/safety.yaml`: Hard limits for strokes, workspace bounding boxes, max currents.

### Kinematics
* `sil_sim/kinematics/__init__.py`: Package initialization. No public API.
* `sil_sim/kinematics/geometry.py`: Stewart-platform geometry model.
  - `StewartGeometry`: Computes base/platform joints. Methods: `rotation_matrix()`, `compute_leg_lengths()`, `compute_leg_vectors()`, `compute_joint_angles()`. Reads: geometry config. Produces: Physical joint vectors.
* `sil_sim/kinematics/ik.py`: Inverse Kinematics solver.
  - `inverse_kinematics()`, `inverse_kinematics_with_vectors()`: Convert 6-DOF pose to leg lengths. Reads: `StewartGeometry`. Produces: target leg lengths (m).
* `sil_sim/kinematics/workspace.py`: Workspace feasibility checking.
  - `WorkspaceViolation`: Exception/flag. `check_workspace()`: Verifies pose limits. Reads: safety config. Produces: Violations/booleans.

### Motion Cueing
* `sil_sim/motion_cueing/__init__.py`: Package initialization. No public API.
* `sil_sim/motion_cueing/hpf.py`: Discrete high-pass filter.
  - `HighPassFilter`: 2nd-order Butterworth. `create_hpf_from_config()`. Reads: MCA config. Produces: Transients.
* `sil_sim/motion_cueing/tilt_coordination.py`: Sustained acceleration tilt generation.
  - `TiltCoordination`: Rate-limited tilt generator. `create_tilt_from_config()`. Reads: MCA config. Produces: Roll/Pitch rad.
* `sil_sim/motion_cueing/washout.py`: Return-to-neutral filter.
  - `WashoutFilter`: Exponential decay. `create_washout_from_config()`. Reads: MCA config. Produces: Bleed-off offset.
* `sil_sim/motion_cueing/mca_pipeline.py`: Full MCA Pipeline.
  - `MCAPipeline`: Orchestrates HPF + Tilt + Washout. Reads: Telemetry. Produces: Target 6-DOF pose.

### Actuator & Control
* `sil_sim/actuator/__init__.py`: Package initialization. No public API.
* `sil_sim/actuator/actuator.py`: Integrated actuator simulation.
  - `ActuatorState`: DTO. `ActuatorSim`: Combines Screw+Encoder+PID+CommandGen. `update()`: runs 1 cycle. Reads: Target length. Produces: ActuatorState (rpm, pos).
* `sil_sim/actuator/encoder_model.py`: Quadrature encoder model.
  - `EncoderModel`: `update()`, `counts_to_position()`. Reads: True position. Produces: Quantized counts.
* `sil_sim/actuator/screw_model.py`: Ballscrew kinematics model.
  - `ScrewModel`: Converts linear translation to rotational units. `displacement_to_pulses()`, `rpm_to_velocity()`. Reads: Actuator config.
* `sil_sim/control/__init__.py`: Package initialization. No public API.
* `sil_sim/control/pid.py`: Discrete PID with anti-windup.
  - `PIDController`, `create_pid_from_config()`. Reads: Setpoint, measurement. Produces: Velocity command.
* `sil_sim/control/command_gen.py`: Stepper pulse generator.
  - `StepperCommand`, `CommandGenerator`: Converts velocity to Step/Dir. Reads: Velocity. Produces: Pulse counts.

### Electrical & Safety
* `sil_sim/electrical/__init__.py`: Package initialization. No public API.
* `sil_sim/electrical/bus_model.py`: DC bus power.
  - `BusModel`: `estimate_bus_state()`. Reads: Driver powers. Produces: Total A/W.
* `sil_sim/electrical/motor_model.py`: Motor torque & phase current.
  - `MotorModel`: `estimate_torque()`, `estimate_phase_current()`. Reads: Velocity, mass. Produces: Nm, A.
* `sil_sim/electrical/power_model.py`: Driver power estimation.
  - `PowerModel`: `estimate_driver_input_power()`. Reads: Phase power, eta. Produces: DC bus draw (W).
* `sil_sim/safety/__init__.py`: Package initialization. No public API.
* `sil_sim/safety/estop_state.py`: E-Stop state machine.
  - `EStopState`, `EStopStateMachine`. Reads: User inputs. Produces: Power status.
* `sil_sim/safety/limits.py`: Safety limit checker.
  - `SafetyViolation`, `check_all_limits()`. Reads: All states. Produces: Safety fault flags.

### Telemetry & Simulation Loop
* `sil_sim/telemetry/__init__.py`: Package initialization. No public API.
* `sil_sim/telemetry/base.py`: Abstract telemetry interfaces.
  - `TelemetryFrame`, `TelemetrySource`. Defines contract.
* `sil_sim/telemetry/carla_source.py`: CARLA parser.
  - `CarlaSource`, `convert_carla_to_automotive()`. Reads: UE coordinates. Produces: Right-hand frames.
* `sil_sim/telemetry/synthetic.py`: Synthetic waveform generator.
  - `SyntheticSource`. Produces: Sine/Chirp trajectories.
* `sil_sim/simulation/__init__.py`: Package initialization. No public API.
* `sil_sim/simulation/pipeline.py`: Main SIL simulation chain.
  - `SimPipeline`: Wires everything together. `step()`. Reads: Telemetry. Produces: Full logging frame.
* `sil_sim/simulation/sim_loop.py`: Runtime executor.
  - `SimLoop`: Orchestrates pipeline at proper rate.

### Logging & Visualization
* `sil_sim/logging/__init__.py`: Package initialization. No public API.
* `sil_sim/logging/logger.py`: Disk logging.
  - `SimLogger`: `log_timestep()`. Reads: Dicts. Produces: CSV/JSONL files.
* `sil_sim/visualization/__init__.py`: Package initialization. No public API.
* `sil_sim/visualization/dashboard.py`: FastAPI / Websocket server.
  - `DashboardServer`: Runs uvicorn async. Reads: Queue. Produces: JSON stream to UI.
* `sil_sim/visualization/dashboard.html`: Vanilla JS/HTML UI for telemetry inspection.

### Tests
* `sil_sim/tests/__init__.py`: Package initialization. No public API.
* `sil_sim/tests/test_carla_conversion.py`: Tests coordinate transforms.
* `sil_sim/tests/test_e2e.py`: Tests full pipeline flow.
* `sil_sim/tests/test_electrical.py`: Tests power budget constraints.
* `sil_sim/tests/test_encoder.py`: Tests quantisation math.
* `sil_sim/tests/test_ik.py`: Tests geometry solver.
* `sil_sim/tests/test_pid.py`: Tests PID controller anti-windup logic.
* `sil_sim/tests/test_screw.py`: Tests mechanical ratios.
* `sil_sim/tests/test_workspace.py`: Tests limit enforcement.

## 3. Every Configuration Parameter, With Source

| Parameter | Value | Source & Rationale |
|---|---|---|
| **geometry.yaml** | | |
| `base.plate_diameter_m` | 0.400 | CONFIRMED — from hardware spec |
| `base.joints_xy_m` | [0.0, 1.101]... | MEASURED — from CAD (paired 6-point) |
| `platform.plate_diameter_m` | 0.250 | CONFIRMED — from hardware spec |
| `platform.joints_xy_m` | [0.447, 0.258]... | MEASURED — from CAD (paired 6-point) |
| `neutral_height_m` | 1.672425 | MEASURED — from CAD neutral pose |
| `joint_max_angle_deg` | 25.0 | CONFIRMED — from rod-end spec |
| **actuator.yaml** | | |
| `ballscrew.lead_m` | 0.004 | CONFIRMED — 4 mm lead |
| `motor.steps_per_rev` | 200 | CONFIRMED — 1.8°/step |
| `driver.microsteps_per_fullstep` | 16 | ASSUMED — report states DM542 capable of up to 25600 microsteps, does not specify DIP switch config |
| `coupling.ratio` | 1.0 | CONFIRMED — 1:1 direct, no gear reduction |
| `encoder.ppr` | 600 | CONFIRMED — 600 pulses per revolution |
| `encoder.quadrature_multiplier` | 4 | CONFIRMED — ABZ ×4 decode |
| `safety.estop.contactor_rating_a` | 25.0 | CONFIRMED — 25 A AC |
| `actuator.estimated_usable_stroke_m` | 0.190 | PLACEHOLDER — physical hardware pending |
| `actuator.stroke_midpoint_ratio` | 0.5 | PLACEHOLDER — pending neutral position validation |
| `actuator.min_leg_lengths_m` | `[1.83, 1.85, ...]` | DERIVED — per leg from home_lengths |
| `actuator.max_leg_lengths_m` | `[2.02, 2.04, ...]` | DERIVED — per leg from home_lengths |
| `power.max_bus_current_a` | 10.0 | CONFIRMED — PSU limit |
| `power.max_phase_current_a` | 4.2 | CONFIRMED — DM542 max |

## 4. Every Physical/Electrical/Mechanical Estimate

| Parameter | Value | Source & Rationale |
|---|---|---|
| **mca.yaml** | | |
| `hpf.cutoff_freq_hz` | 1.0 | PLACEHOLDER |
| `tilt_coordination.gain` | 0.15 | PLACEHOLDER — rad / (m/s²) |
| `washout.time_constant_s` | 2.0 | PLACEHOLDER |
| **pid.yaml** | | |
| `gains.kp` | 10.0 | PLACEHOLDER — proportional gain |
| `gains.ki` | 0.5 | PLACEHOLDER — integral gain |
| `gains.kd` | 0.1 | PLACEHOLDER — derivative gain |
| **electrical.yaml** | | |
| `motor.kt_estimated_nm_per_a` | 0.65 | PLACEHOLDER — ESTIMATED (midpoint of 0.55–0.75) |
| `efficiencies.eta_screw` | 0.90 | PLACEHOLDER — ESTIMATED |
| `efficiencies.eta_driver` | 0.92 | PLACEHOLDER — ESTIMATED |
| `payload.total_payload_kg` | 20.0 | PLACEHOLDER — ESTIMATED, replace with weighed value |

## 5. Complete Change Log of the Mentor Feedback Fixes

**Workspace-Limit Unification:**
- **The Bug:** `mca.yaml` had its own hardcoded `max_tilt_rad`, while `safety.yaml` had `workspace.max_rotation_deg`. This violated the single-source-of-truth principle; MCA could theoretically command a tilt that safety would instantly fault on.
- **The Fix:** In `sil_sim/config/loader.py:load_all()`, the MCA configuration is dynamically injected with the workspace limit from safety: `configs['mca']['tilt_coordination']['max_tilt_rad'] = math.radians(max_deg)`. In `tilt_coordination.py`, the MCA strictly clamps tilt target vectors against this injected value.
- **Before/After:** The E2E tests previously used duplicated logic. Now, changing `safety.yaml` limits automatically throttles the MCA's tilt output to guarantee physical feasibility without throwing a safety fault.

**Pre-PID Stroke Clamping:**
- **The Bug:** Inverse Kinematics (IK) could request a leg length beyond the physical stroke limits. The PID controller would attempt to reach this impossible setpoint, causing the integral term to wind up indefinitely. Once the target returned to a feasible length, the actuator would suffer massive overshoot/lag while the integrator unwound.
- **The Fix:** In `sil_sim/actuator/actuator.py:update()`, the target leg length is pre-clamped before it is passed to the PID loop: `target_clamped = max(self._min_stroke, min(self._max_stroke, target_leg_length))`. The PID error is computed against this physically bounded target.
- **Before/After Numbers:** During the `chirp` run, tracking error at frequency peaks (where IK targets exceed physical stroke) would skyrocket, causing subsequent cycles to lag. With the fix, the `chirp` run tracking error stays bounded, and integrator windup is 0 at the physical limits. The `state.clamped` flag is logged explicitly.
- **Edge Cases Unresolved:** If the physical actuator slips or misses steps, the purely kinematic plant model will not know. The encoder reads the theoretical plant position, not the real hardware position. 

## 6. Test Coverage Map

Total of **39 Tests** across 8 files:

**`test_carla_conversion.py`**
1. `test_linear_true_vectors`: Verifies (x,y,z) velocity/accel flip Y axis for Automotive frame.
2. `test_angular_pseudovectors`: Verifies (x,y,z) orientation/angular velocity flip X and Z axes.

**`test_e2e.py`**
3. `test_15_full_pipeline_e2e`: Runs synthetic telemetry through full pipeline and checks bounds.
4. `test_15b_no_measured_labels_in_entire_output`: Asserts 'MEASURED' string does not exist in logs (SIL purity).
5. `test_15c_config_changes_affect_output`: Confirms configs physically alter trajectory outputs.

**`test_electrical.py`**
6. `test_14_power_limit_headroom`: Validates standard sine trajectory stays <480W.
7. `test_14b_power_limit_violation_detection`: Forces over-velocity, checks fault flag trips.
8. `test_power_balance_not_phase_shortcut`: Ensures bus draw respects step-down conversion (Power Balance).
9. `test_all_outputs_labeled_estimated`: Asserts electrical dict outputs only use 'ESTIMATED'.
10. `test_zero_velocity_zero_power`: Asserts holding position draws minimal power.
11. `test_psu_utilization`: Checks utilization math `(P_total / P_max)`.

**`test_encoder.py`**
12. `test_12_counts_to_position`: Asserts `counts * (lead / PPR)` equals mm displacement.
13. `test_12b_update_quantization`: Asserts encoder returns integers, not floats.
14. `test_12c_independent_from_command`: Checks encoder reads plant, not command.
15. `test_resolution`: Verifies physical resolution matches the 4mm lead and 600 PPR specs.
16. `test_reset`: Checks reset returns count to 0.

**`test_ik.py`**
17. `test_01_neutral_pose_equal_leg_lengths`: Z=200mm -> 6 legs = 220mm.
18. `test_02_pure_x_translation`: Checks symmetric opposing leg extensions.
19. `test_03_pure_y_translation`: Checks lateral pair extensions.
20. `test_04_pure_z_translation`: Checks all 6 legs extend equally.
21. `test_05_pure_roll`: Checks differential left/right extensions.
22. `test_06_pure_pitch`: Checks differential front/rear extensions.
23. `test_07_pure_yaw`: Checks rotational shearing lengths.
24. `test_08_combined_6dof`: Math checks full transformation matrix vs brute-force.
25. `test_identity_at_zero`: Rotation(0,0,0) = I.
26. `test_orthogonality`: R * R.T = I.
27. `test_determinant_one`: det(R) = 1.0.

**`test_pid.py`**
28. `test_anti_windup_clamping`: Asserts integrator freezes when output saturates.
29. `test_no_anti_windup`: Asserts integrator grows indefinitely without anti-windup clamping.
30. `test_windup_unclamping`: Verifies the integrator resumes integrating when un-saturated.

**`test_screw.py`**
31. `test_09_displacement_to_motor_revs`: Actuator displacement linearly maps to motor revs (using 4mm lead).
32. `test_10_rpm_to_velocity`: Motor RPM maps accurately to mm/s velocity.
33. `test_11_microstepping_to_step_frequency`: Microstepping logic produces expected step counts.
34. `test_roundtrip_displacement_to_pulses_and_back`: mm -> pulses -> mm == original mm.
35. `test_negative_displacement`: Negative v -> negative revs.
36. `test_direction_from_velocity`: v > 0 = Dir 1, v < 0 = Dir -1.

**`test_workspace.py`**
37. `test_13_workspace_violation_detected`: Z=500mm flags Z_Translation_Limit.
38. `test_13d_multiple_simultaneous_violations`: X=500, Y=500 flags multiple limits.
39. `test_neutral_is_feasible`: Neutral pose strictly produces 0 violations.

**What is EXPLICITLY NOT COVERED:**
- **Dashboard UI:** No Selenium/Playwright tests for the frontend WebSocket connections or graphing.
- **File I/O:** The CSV/JSON logger writing mechanism isn't explicitly tested for disk flushing/formatting.

## 7. Known Limitations and Open Items

1. **Phase 10 Live CARLA Integration:** Implemented in `carla_source.py` and mocked in `test_carla_conversion.py`, but has **never been run against a live server**. Network latencies, dropped packets, or `NaN` coordinate bursts from a crashed UE ego-vehicle could break the telemetry pipeline if the parser isn't hardened.
2. **CARLA Version Mismatch:** CARLA 0.9.15 uses Unreal Engine 4/5 depending on the build, but SIL assumed a UE 0.10.0 coordinate match. We must verify if CARLA 0.9.15 altered the left-handed standard coordinates before committing to the live integration.
3. **Python Version Pinning:** Code is written for 3.8–3.12 but the `requirements.txt`/venv is unpinned. `websockets` or `FastAPI` might throw async loop errors on Python 3.12+ if dependencies aren't strictly locked.
4. **PID Loop Tuning vs Kinematic Integrator Limitations:** The SIL simulator uses a purely kinematic integrator (`self._position += velocity_cmd * self._dt`) with no mass, inertia, mechanical backlash, or non-linear torque saturation (beyond a velocity cap). **Therefore, the configured PID gains (`kp=10`, `ki=0.5`, `kd=0.1`) are entirely untuned placeholders.** The µm-level tracking demonstrated by the SIL validates that the signal plumbing and unit conversions are correct, but it **DOES NOT** validate the physical tuning of the PID controller against real hardware dynamics, which must be calibrated under physical load (M7 milestone).
5. **Stepper KT:** `motor.kt_estimated_nm_per_a` is hardcoded to 0.5. Stepper torque drops aggressively with speed. This linear assumption fails at high RPMs.
