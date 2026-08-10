# DrivingSim 6-DOF Stewart Platform Simulator & Digital Twin

An integrated vehicle-to-Stewart-platform simulation system that merges a Three.js 3D visualizer with a high-fidelity Python Software-in-the-Loop (SIL) simulation engineering model.

This system simulates synthetic driving scenarios, parses the vehicle acceleration signals through a Motion Cueing Algorithm (MCA) pipeline, computes actuator lengths via Inverse Kinematics, simulates actuator PID motor controllers, runs safety limit validations, and streams the digital twin state over WebSockets to a split-screen browser dashboard at 60 Hz.

---

## 🛠️ System Architecture

```
                       ┌───────────────────────────────┐
                       │   SYNTHETIC ROAD SCENARIOS    │
                       │ (pothole, hump, turn, S-curve)│
                       └───────────────┬───────────────┘
                                       │ Telemetry Frame
                                       ▼
                       ┌───────────────────────────────┐
                       │    MOTION CUEING PIPELINE     │
                       │ (filters, tilt-coordination)  │
                       └───────────────┬───────────────┘
                                       │ Target Pose
                                       ▼
                       ┌───────────────────────────────┐
                       │      INVERSE KINEMATICS       │
                       │   (6 Leg Lengths, Geometry)   │
                       └───────────────┬───────────────┘
                                       │ Leg Lengths
                                       ▼
                       ┌───────────────────────────────┐
                       │      ACTUATOR SIM (x6)        │
                       │ (PID Controller, Ball-screws) │
                       └───────────────┬───────────────┘
                                       │ Current / Voltage
                                       ▼
                       ┌───────────────────────────────┐
                       │     SAFETY & ESTOP CHECKS     │
                       │ (workspace, current, strokes) │
                       └───────────────┬───────────────┘
                                       │ State JSON
                                       ▼
                       ┌───────────────────────────────┐
                       │       WEBSOCKET BRIDGE        │
                       │        (ws://:8765)           │
                       └───────────────┬───────────────┘
                                       │
                ┌──────────────────────┴──────────────────────┐
                ▼                                             ▼
┌───────────────────────────────┐             ┌───────────────────────────────┐
│     3D STEWART PLATFORM       │             │     2D CANVAS VEHICLE TRACK   │
│     (Three.js Viewport)       │             │     (Steering & shockwaves)   │
└───────────────────────────────┘             └───────────────────────────────┘
```

---

## 🚀 Getting Started

### 1. Prerequisites
Ensure you have Python 3.8 to 3.12 and dependencies installed:
```bash
pip3 install numpy pyyaml websockets pytest
```

### 2. Run the Simulation Server
Start the Python backend simulation server. Specify the scenario to load on startup:
```bash
# Available scenarios: straight, pothole, speed_hump, turn, lane_change
python3 run_sim.py --scenario pothole
```

You can also run recording or replay commands:
```bash
# Record telemetry frames to recordings/ folder
python3 run_sim.py --scenario turn --record

# Replay a recorded JSONL file
python3 run_sim.py --replay recordings/turn_170756789.jsonl
```

### 3. Open the Dashboard
Open the HTML5 dashboard in any modern web browser:
`dashboard/index.html`

The connection status indicator at the top right will change to **CONNECTED** (green), and real-time telemetry will begin streaming.

---

## 🏎️ Driving Scenarios

- **Pothole Impact**: Vehicle travels at constant speed and hits a localized road depression. Front and rear wheels trigger delayed Gaussian vertical acceleration impulses, forcing the vehicle body to pitch and heave under a damped second-order dynamic response.
- **Speed Hump**: Simulates crossing a speed hump. Excites heave and pitch states sequentially as front and rear wheels traverse a raised-cosine bump profile.
- **Turn**: Enters a left or right turn ($a_y = v^2/R$), displaying lateral roll tilt-coordination cues on the platform.
- **Lane Change**: Sinusoidal lane shift (S-curve trajectory) producing alternating lateral acceleration forces and yaw rates.
- **Straight**: Cruising on a straight road with realistic vertical road-surface noise.

---

## 🎛️ Manual Controls (Python Integration)

Toggle to **MANUAL SLIDERS** in the browser dashboard. 

Dragging manual sliders (Surge, Sway, Heave, Roll, Pitch, Yaw) sends the target pose to the Python backend over WebSockets. The Python server runs the target through the **actual** inverse kinematics, PID actuator, electrical, and safety modules, and streams the states back. This ensures that safety E-Stop protections and motor current estimations are active even during manual debugging!

---

## 🔬 Running Unit Tests

Run the test suites to verify physical constraints and kinematics calculation correctness:
```bash
# Run sil_sim tests
python3 -m pytest vehicle_sim/sil_sim/tests/ -v

# Run scenario-specific tests
python3 -m pytest tests/test_scenarios.py -v
```
