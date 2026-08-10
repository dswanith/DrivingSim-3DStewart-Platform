# Stewart-Platform 6-DOF Software-in-the-Loop Simulator

A Software-in-the-Loop (SIL) simulator for a 1/1-scale 6-DOF Stewart-platform motion
simulator integrated with the CARLA driving simulator. This software takes vehicle
telemetry and propagates it through the full control pipeline — motion cueing, inverse
kinematics, actuator control, and electrical estimation — producing every value the
real STM32 firmware would eventually produce.

**No physical hardware is actuated.** This is a validation tool for the control pipeline.

## Quick Start

### Installation

```bash
# Create virtual environment (recommended)
python -m venv venv
venv\Scripts\activate          # Windows
# source venv/bin/activate     # Linux/macOS

# Install dependencies
pip install -r requirements.txt
```

### Run Synthetic Mode (default — no CARLA needed)

```bash
python run.py --mode synthetic
python run.py --mode synthetic --duration 10 --signal sine
python run.py --mode synthetic --signal chirp --dashboard
python run.py --mode synthetic --realtime --dashboard
```

### Run with Dashboard

```bash
python run.py --mode synthetic --dashboard
# Open http://localhost:8080 in a browser
```

### Run Tests

```bash
python -m pytest sil_sim/tests/ -v
```

### Run CARLA Mode (requires CARLA server + Python API)

```bash
pip install carla==0.9.15
python run.py --mode carla
```

**CARLA integration is deferred until the synthetic E2E gate passes.**
If CARLA is unavailable, the simulator falls back to synthetic mode automatically.

## Command-Line Options

| Flag | Description | Default |
|------|-------------|---------|
| `--mode` | `synthetic` or `carla` | `synthetic` |
| `--dashboard` | Launch live browser dashboard | Off |
| `--realtime` | Pace to ~60 Hz (otherwise runs fast) | Off |
| `--duration` | Synthetic mode duration (seconds) | 10 |
| `--signal` | Synthetic signal type: `sine`, `step`, `ramp`, `chirp`, `combined` | `sine` |
| `--no-log` | Disable CSV/JSONL logging | Off |
| `--http-port` | Dashboard HTTP port | 8080 |
| `--ws-port` | Dashboard WebSocket port | 8081 |

## Architecture

```
CARLA / Synthetic Telemetry (~60 Hz)
  → Motion Cueing Algorithm (HPF + tilt coordination + washout)
  → Target platform pose [x, y, z, roll, pitch, yaw]
  → Stewart-platform Inverse Kinematics (L_i = ‖T + R·p_i − b_i‖)
  → 6 actuator leg lengths L1..L6
  → Per-actuator PID position control loop
  → Command generation (STEP/DIR/ENABLE, RPM, pulse frequency)
  → Electrical estimation (torque, current, power via power balance)
  → Safety checks (workspace, current, power limits)
  → Logging (CSV/JSONL) + Dashboard (WebSocket + Chart.js)
```

### Module Structure

```
sil_sim/
├── config/             7 YAML config files + loader with validation
├── telemetry/          TelemetrySource ABC, SyntheticSource, CarlaSource (scaffold)
├── motion_cueing/      HPF, tilt coordination, washout, MCA pipeline
├── kinematics/         Geometry (parameterized), IK, workspace checker
├── actuator/           Screw model, encoder model, integrated actuator sim
├── control/            PID controller, stepper command generator
├── electrical/         Motor model, power model (power balance), bus model
├── safety/             Limit checker, E-stop state machine
├── simulation/         Pipeline (wires everything), sim loop
├── logging/            CSV + JSONL logger
├── visualization/      Dashboard server + HTML/JS page
└── tests/              15 required tests (IK, screw, encoder, workspace, electrical, E2E)
```

### Output Class Discipline

Every output in logs, dashboard, and code is labeled with one of three classes:

| Class | Meaning | Used For |
|-------|---------|----------|
| **EXACT** | Pure geometry/kinematics/math | IK, leg lengths, encoder counts, screw kinematics |
| **ESTIMATED** | Model-based, depends on assumed parameters | Motor current, bus power, torque |
| **MEASURED** | Real sensor data | **NEVER produced by this simulator** |

## Configuration Reference

All config files are in `sil_sim/config/`. Values marked **CONFIRMED** come from the
hardware spec. Values marked **PLACEHOLDER** are defaults that must be updated.

### geometry.yaml — Joint Geometry

| Parameter | Value | Status |
|-----------|-------|--------|
| Base plate diameter | 0.400 m | CONFIRMED |
| Platform plate diameter | 0.250 m | CONFIRMED |
| Base bolt-circle radius | 0.170 m | PLACEHOLDER |
| Platform bolt-circle radius | 0.100 m | PLACEHOLDER |
| Joint-pair clocking angles (base) | [0°,60°], [120°,180°], [240°,300°] | PLACEHOLDER |
| Joint-pair clocking angles (platform) | [30°,90°], [150°,210°], [270°,330°] | PLACEHOLDER |
| Neutral height | 0.200 m | PLACEHOLDER |
| Rod-end articulation | ±25° | CONFIRMED |
| Workspace translation limits | ±30 mm (X,Y,Z) | PLACEHOLDER |
| Workspace rotation limits | ±15° (R,P,Y) | PLACEHOLDER |
| Euler convention | Intrinsic Z-Y-X, automotive axes | CONFIRMED |

### actuator.yaml — Ballscrew / Motor / Encoder

| Parameter | Value | Status |
|-----------|-------|--------|
| Ballscrew lead | 4 mm | CONFIRMED |
| Ballscrew OAL | 250 mm | CONFIRMED |
| Estimated usable stroke | 190 mm | ESTIMATED |
| Motor steps/rev | 200 (1.8°/step) | CONFIRMED |
| Rated current | 4 A/phase | CONFIRMED |
| DM542 microstep setting | 16 µsteps/step | PLACEHOLDER |
| Coupling ratio | 1:1 | CONFIRMED |
| Encoder PPR | 600 | CONFIRMED |
| Quadrature multiplier | ×4 | CONFIRMED |
| Encoder counts/rev | 2400 | CONFIRMED |
| Home leg length | 0.220 m | PLACEHOLDER |

### pid.yaml — PID Controller

| Parameter | Value | Status |
|-----------|-------|--------|
| Kp | 10.0 | PLACEHOLDER |
| Ki | 0.5 | PLACEHOLDER |
| Kd | 0.1 | PLACEHOLDER |
| Output limits | ±50 mm/s | PLACEHOLDER |
| Control rate | 1000 Hz | PLACEHOLDER |
| Anti-windup | Clamping, enabled | Default |

### electrical.yaml — Electrical Model

| Parameter | Value | Status |
|-----------|-------|--------|
| Bus voltage | 48 V | CONFIRMED |
| PSU capacity | 480 W / 10 A | CONFIRMED |
| Logic supply | 5 V / 3 A | CONFIRMED |
| Kt (torque constant) | 0.65 N·m/A | ESTIMATED |
| η_screw (ballscrew efficiency) | 0.90 | ESTIMATED |
| η_driver (DM542 efficiency) | 0.92 | ESTIMATED |
| Total payload mass | 20 kg | ESTIMATED (genuinely unknown) |
| Platform mass | 1.1 kg | ESTIMATED (from geometry) |

### mca.yaml — Motion Cueing Algorithm

| Parameter | Value | Status |
|-----------|-------|--------|
| HPF cutoff | 1.0 Hz | PLACEHOLDER |
| HPF order | 2 (Butterworth) | Default |
| Tilt coordination gain | 0.15 rad/(m/s²) | PLACEHOLDER |
| Tilt rate limit | 0.05 rad/s (~3°/s) | PLACEHOLDER |
| Washout time constant | 2.0 s | PLACEHOLDER |

### safety.yaml — Safety Limits

| Parameter | Value | Status |
|-----------|-------|--------|
| Min leg length | 0.125 m | PLACEHOLDER |
| Max leg length | 0.315 m | PLACEHOLDER |
| Max bus current | 10 A | CONFIRMED |
| Max bus power | 480 W | CONFIRMED |
| Max phase current | 4.2 A | CONFIRMED |

### carla.yaml — CARLA Connection

| Parameter | Value | Status |
|-----------|-------|--------|
| Host | localhost | PLACEHOLDER |
| Port | 2000 | PLACEHOLDER |
| Timeout | 5 s | PLACEHOLDER |
| API version | 0.9.15 | PLACEHOLDER |

## PID Plant Model — Known Limitation

The actuator plant model used for PID closed-loop simulation is a **kinematic integrator**:

```
position += velocity × dt
```

This has **no inertia, friction, or backlash**. This is the correct choice for SIL
validation (verifying the pipeline math), but the closed-loop tracking numbers from
this simulator **do NOT represent validated firmware behavior**. When the real STM32
firmware runs on physical hardware, PID gains will need re-tuning against the actual
plant dynamics.

## Electrical Model — Power Balance Approach

The electrical model uses an explicit **power-balance** approach to estimate DC bus current:

```
T_required   = F_axial × lead / (2π × η_screw)
I_phase_est  = T_required / Kt_est
P_motor_elec = T × ω / η_screw
P_driver_in  = P_motor_elec / η_driver
I_bus_leg    = P_driver_in / V_bus        ← power balance, NOT I_phase
I_bus_total  = Σ(I_bus_leg) + I_logic
```

Bus current is **never** derived from motor phase current directly — they are different
electrical domains (motor AC phase vs. DC bus).

All electrical outputs are labeled **ESTIMATED** throughout the system.

## Encoder Path Independence

The encoder model is a **genuinely independent** signal path from the motor command path.
The encoder reads the simulated plant position (with quantization to discrete counts),
NOT the commanded position. The PID error is computed as:

```
error = target_position − encoder_read_position
```

This prevents the simulator from trivially showing zero tracking error by shortcutting
the measurement to equal the command.

## CARLA Integration

CARLA integration is **deferred to Phase 10** and will not be implemented until the
synthetic E2E test (Test 15) passes. The `carla_source.py` module is scaffolded with
the `TelemetrySource` ABC interface.

> [!WARNING]
> **Python Version Constraint**: The CARLA 0.9.15 Python API provides published wheels only up to Python 3.12. Do not attempt to run Phase 10 using Python 3.13+. Pin your virtual environment to Python 3.8–3.12 before starting CARLA integration.

When implemented:
- Connects to CARLA Python API (verify `pip install carla==0.9.15` matches your server)
- Configurable host/port via `carla.yaml`
- Automatically locates the player vehicle by checking for `role_name='hero'` or `role_name='ego_vehicle'`. **Make sure your CARLA spawn script sets this attribute**, otherwise it will fall back to grabbing the first vehicle it finds.
- Graceful disconnect handling → falls back to synthetic mode with a logged warning
- Windows-compatible, no Docker dependencies

## Logging Output

Each run produces two log files in the `logs/` directory:
- `sil_YYYYMMDD_HHMMSS_log.csv` — flat CSV, one row per timestep
- `sil_YYYYMMDD_HHMMSS_log.jsonl` — JSON Lines, one JSON object per timestep

Fields include: timestamp, telemetry, target pose, L1–L6, per-actuator state
(position, velocity, RPM, step frequency, **STEP/DIR/ENABLE state**, encoder counts,
position error), electrical estimates (all labeled ESTIMATED), status/errors.

## Tests

15 required tests covering all equations in §5:

| # | Test | Module |
|---|------|--------|
| 1 | Neutral pose → equal leg lengths | test_ik.py |
| 2 | Pure X translation | test_ik.py |
| 3 | Pure Y translation | test_ik.py |
| 4 | Pure Z translation | test_ik.py |
| 5 | Pure roll | test_ik.py |
| 6 | Pure pitch | test_ik.py |
| 7 | Pure yaw | test_ik.py |
| 8 | Combined 6-DOF pose | test_ik.py |
| 9 | Displacement → motor revolutions | test_screw.py |
| 10 | RPM → linear velocity | test_screw.py |
| 11 | Microstepping → pulse frequency | test_screw.py |
| 12 | Encoder counts → actuator position | test_encoder.py |
| 13 | Workspace violation detection | test_workspace.py |
| 14 | Power-limit violation detection | test_electrical.py |
| 15 | Full E2E pipeline (gate test) | test_e2e.py |
