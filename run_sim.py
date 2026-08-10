"""
run_sim.py — Main entry point for the integrated DrivingSim-3DStewart-Platform.

Usage:
    python run_sim.py --scenario speed_hump
    python run_sim.py --scenario turn
    python run_sim.py --scenario pothole
    python run_sim.py --scenario straight
    python run_sim.py --replay recordings/pothole.jsonl

The simulation:
  1. Serves a WebSocket on ws://localhost:8765
  2. Broadcasts ~95 Hz telemetry frames (platform pose + 6 actuator lengths)
  3. Receives control commands from the browser dashboard
  4. Auto-loops scenarios when they complete
"""
import argparse
import asyncio
import json
import logging
import os
import sys
import threading
import time
import queue
import numpy as np
from typing import Dict, Any, Optional

# Add vehicle_sim to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'vehicle_sim'))

from vehicle_sim.sil_sim.config.loader import load_all
from vehicle_sim.sil_sim.simulation.pipeline import SimPipeline
from integration.sim_clock import SimulationClock
from integration.websocket_server import WebSocketPublisher
from integration.protocol import build_ws_message
from scenarios import SCENARIOS

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger('run_sim')


class InteractiveTelemetrySource:
    """Wraps scenario to allow feeding dummy frames in manual mode."""
    def __init__(self, scenario):
        self.scenario = scenario
        self.dummy_frame = None

    def get_frame(self):
        if self.dummy_frame is not None:
            return self.dummy_frame
        return self.scenario.step()

    def is_connected(self):
        return True

    def reset(self):
        self.scenario.reset()
        self.dummy_frame = None

    @property
    def source_name(self):
        return self.scenario.source_name


class IntegrationLoop:
    """Wires the scenario, pipeline, and WebSocket publisher together."""

    def __init__(self, config_dir: str, scenario_name: str,
                 record_file: Optional[str] = None, auto_start: bool = True,
                 auto_loop: bool = True):
        self.scenario_name = scenario_name
        self.record_file = record_file
        self.recording_f = None

        # State machine
        self.mode = "synthetic"           # "synthetic" or "manual"
        self.is_running = auto_start      # Auto-start in demo mode
        self.auto_loop = auto_loop        # Loop scenario when complete
        self.manual_pose = np.zeros(6)
        self.cmd_queue = queue.Queue()
        self._loop_count = 0

        # 1. Load configuration
        logger.info("Loading configuration from sil_sim/config")
        self.configs = load_all()
        self.configs['mca']['workspace_limits'] = self.configs['safety']['workspace']

        # 2. Initialize Scenario
        if scenario_name not in SCENARIOS:
            raise ValueError(
                f"Unknown scenario: '{scenario_name}'. Available: {list(SCENARIOS.keys())}"
            )
        self.scenario = SCENARIOS[scenario_name]()
        self.telemetry_source = InteractiveTelemetrySource(self.scenario)
        logger.info(f"Initialized scenario: {scenario_name}")

        # 3. Initialize Pipeline
        self.pipeline = SimPipeline(self.configs, telemetry_source=self.telemetry_source)

        # Patch MCA update for manual mode support
        self.original_mca_update = self.pipeline.mca.update
        self.pipeline.mca.update = self._patched_mca_update

        # 4. Initialize Clock
        self.clock = SimulationClock(
            rate_hz=self.configs['mca']['input_rate_hz'],
            realtime=True
        )

        # 5. Initialize WebSocket Server
        self.ws_pub = WebSocketPublisher(port=8765, incoming_queue=self.cmd_queue)

    def _patched_mca_update(self, frame):
        if self.mode == "manual":
            return self.manual_pose
        return self.original_mca_update(frame)

    def _setup_asyncio(self):
        """Run asyncio event loop in background thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.ws_pub.start(self.loop)
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    def _process_commands(self):
        """Drain and execute all pending client commands."""
        while not self.cmd_queue.empty():
            try:
                data_str = self.cmd_queue.get_nowait()
                msg = json.loads(data_str)
                cmd = msg.get("command")

                if cmd == "start":
                    self.is_running = True
                    logger.info("▶  Simulation STARTED")

                elif cmd == "pause":
                    self.is_running = False
                    logger.info("⏸  Simulation PAUSED")

                elif cmd == "reset":
                    self.is_running = False
                    self._reset_scenario()
                    logger.info("⟳  Simulation RESET")

                elif cmd == "set_mode":
                    new_mode = msg.get("mode")
                    if new_mode in ["synthetic", "manual"]:
                        self.mode = new_mode
                        logger.info(f"Mode → {new_mode}")
                        if new_mode == "manual":
                            self.manual_pose = np.zeros(6)

                elif cmd == "select_scenario":
                    scen = msg.get("scenario")
                    if scen in SCENARIOS:
                        self.scenario_name = scen
                        self.scenario = SCENARIOS[scen]()
                        self.telemetry_source.scenario = self.scenario
                        self._reset_scenario()
                        logger.info(f"Scenario → {scen}")

                elif cmd == "set_manual_pose":
                    pose_arr = msg.get("pose")
                    if isinstance(pose_arr, list) and len(pose_arr) == 6:
                        self.manual_pose = np.array(pose_arr, dtype=np.float64)

            except queue.Empty:
                break
            except Exception as e:
                logger.error(f"Command error: {e}")

    def _reset_scenario(self):
        """Reset scenario, pipeline, and clock."""
        self.scenario.reset()
        self.pipeline.reset()
        self.clock.reset()
        self._loop_count = 0

    def _build_and_broadcast(self, data: Dict[str, Any]):
        """Build WS packet from pipeline data and broadcast."""
        target_pose_abs = np.array([
            data['target_x'],
            data['target_y'],
            data['target_z']
        ])
        target_rot = np.array([
            data['target_roll'],
            data['target_pitch'],
            data['target_yaw']
        ])

        # Relative pose for motion_cueing display (z relative to neutral)
        target_pose_rel = np.array([
            data['target_x'],
            data['target_y'],
            data['target_z'] - self.pipeline.geometry.neutral_height,
            data['target_roll'],
            data['target_pitch'],
            data['target_yaw']
        ])

        # Compute top joints in world frame using Python IK geometry
        base_joints = self.pipeline.geometry.base_joints
        R = self.pipeline.geometry.rotation_matrix(
            target_rot[0], target_rot[1], target_rot[2]
        )
        top_joints = np.zeros((6, 3))
        for i in range(6):
            top_joints[i] = target_pose_abs + R @ self.pipeline.geometry.platform_joints[i]

        leg_lengths = np.array([data[f'L{i}_m'] for i in range(1, 7)])
        act_positions = [data[f'leg{i}_position_m'] for i in range(1, 7)]
        act_velocities = [data[f'leg{i}_velocity_m_s'] for i in range(1, 7)]
        act_errors = [data[f'leg{i}_error_m'] for i in range(1, 7)]
        encoder_counts = [data[f'leg{i}_encoder_counts'] for i in range(1, 7)]
        safety_warnings = ([data['safety_violations']]
                           if data['safety_violations'] else [])

        msg = build_ws_message(
            timestamp=data['timestamp_s'],
            frame_index=self.pipeline.frame_count,
            scenario_name=self.scenario_name,
            mode=self.mode,
            vehicle_accel_xyz_ms2=np.array([
                data['telem_accel_x'],
                data['telem_accel_y'],
                data['telem_accel_z']
            ]),
            vehicle_ang_vel_xyz_rads=np.array([
                data['telem_ang_vel_x'],
                data['telem_ang_vel_y'],
                data['telem_ang_vel_z']
            ]),
            vehicle_orientation_rpy_rad=np.array([
                data['telem_roll'],
                data['telem_pitch'],
                data['telem_yaw']
            ]),
            vehicle_velocity_xyz_ms=np.array([
                data['telem_vel_x'],
                data['telem_vel_y'],
                data['telem_vel_z']
            ]),
            platform_pose_m_rad=target_pose_rel,
            leg_lengths_m=leg_lengths,
            base_joints_m=base_joints,
            top_joints_m=top_joints,
            actuator_positions_m=act_positions,
            actuator_velocities_ms=act_velocities,
            actuator_errors_m=act_errors,
            encoder_counts=encoder_counts,
            safety_valid=(data['workspace_status'] == 'OK'),
            safety_warnings=safety_warnings,
            safety_estop=(data['estop_state'] != 'NORMAL'),
            sim_freq_hz=self.configs['mca']['input_rate_hz'],
            carla_connected=False,
            phase=self.scenario.state.scenario_phase,
        )

        self.ws_pub.broadcast(msg)

        if self.recording_f and self.is_running:
            self.recording_f.write(msg + '\n')

    def run(self):
        """Main simulation loop."""
        # Start WebSocket thread
        ws_thread = threading.Thread(target=self._setup_asyncio, daemon=True)
        ws_thread.start()

        if self.record_file:
            os.makedirs(os.path.dirname(self.record_file), exist_ok=True)
            self.recording_f = open(self.record_file, 'w')
            logger.info(f"Recording to {self.record_file}")

        logger.info("=" * 60)
        logger.info(f"  DrivingSim 6-DOF Stewart Platform")
        logger.info(f"  WebSocket: ws://0.0.0.0:8765")
        logger.info(f"  Scenario : {self.scenario_name}")
        logger.info(f"  Auto-loop: {self.auto_loop}")
        logger.info("=" * 60)
        logger.info("Waiting for browser connection at ws://localhost:8765 ...")

        try:
            while True:
                # 1. Process incoming commands from browser
                self._process_commands()

                # 2. Decide whether to step
                if not self.is_running and self.mode == "synthetic":
                    time.sleep(0.01)
                    continue

                # 3. Tick the clock
                tick = self.clock.tick()

                if self.mode == "manual":
                    # Feed dummy neutral frame; MCA is patched to return manual_pose
                    self.telemetry_source.dummy_frame = self.scenario._make_frame(
                        t=tick.simulation_time,
                        vx=0.0
                    )
                else:
                    self.telemetry_source.dummy_frame = None

                # 4. Step the pipeline
                data = self.pipeline.step()

                if data is None:
                    # Scenario completed
                    if self.auto_loop:
                        self._loop_count += 1
                        logger.info(
                            f"Scenario '{self.scenario_name}' completed "
                            f"(loop {self._loop_count}). Restarting..."
                        )
                        self.scenario.reset()
                        self.pipeline.reset()
                        self.clock.reset()
                        continue
                    else:
                        self.is_running = False
                        logger.info("Scenario complete. Paused.")
                        continue

                # 5. Build and broadcast WebSocket message
                self._build_and_broadcast(data)

        except KeyboardInterrupt:
            logger.info("Interrupted by user.")
        finally:
            if self.recording_f:
                self.recording_f.close()
            if hasattr(self, 'loop') and self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)
            ws_thread.join(timeout=2.0)


class ReplayLoop:
    """Replays a recorded JSONL file over WebSockets."""

    def __init__(self, record_file: str):
        self.record_file = record_file
        self.ws_pub = WebSocketPublisher(port=8765)

    def _setup_asyncio(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.ws_pub.start(self.loop)
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    def run(self):
        ws_thread = threading.Thread(target=self._setup_asyncio, daemon=True)
        ws_thread.start()

        logger.info(f"Replaying {self.record_file} ...")
        try:
            with open(self.record_file, 'r') as f:
                last_time = None
                for line in f:
                    msg_obj = json.loads(line)
                    t = msg_obj['timestamp_s']
                    if last_time is not None:
                        dt = t - last_time
                        if dt > 0:
                            time.sleep(dt)
                    last_time = t
                    self.ws_pub.broadcast(json.dumps(msg_obj))
            logger.info("Replay finished.")
        except KeyboardInterrupt:
            logger.info("Interrupted.")
        except Exception as e:
            logger.error(f"Replay error: {e}")
        finally:
            if hasattr(self, 'loop') and self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)
            ws_thread.join(timeout=2.0)


def main():
    parser = argparse.ArgumentParser(
        description="DrivingSim 6-DOF Stewart Platform WebSocket Server"
    )
    parser.add_argument(
        '--scenario',
        type=str,
        choices=list(SCENARIOS.keys()),
        default='speed_hump',
        help='Synthetic driving scenario to run (default: speed_hump)'
    )
    parser.add_argument(
        '--record',
        action='store_true',
        help='Record simulation frames to recordings/ directory'
    )
    parser.add_argument(
        '--replay',
        type=str,
        help='Path to a JSONL recording file to replay'
    )
    parser.add_argument(
        '--no-loop',
        action='store_true',
        help='Do not auto-loop when scenario ends (stop instead)'
    )
    parser.add_argument(
        '--no-autostart',
        action='store_true',
        help='Wait for START command from browser before running'
    )
    parser.add_argument(
        '--config',
        type=str,
        default='vehicle_sim/sil_sim/config',
        help='Path to config directory (informational only)'
    )

    args = parser.parse_args()

    if args.replay:
        loop = ReplayLoop(args.replay)
        loop.run()
    else:
        record_file = None
        if args.record:
            os.makedirs('recordings', exist_ok=True)
            record_file = f"recordings/{args.scenario}_{int(time.time())}.jsonl"

        loop = IntegrationLoop(
            config_dir=args.config,
            scenario_name=args.scenario,
            record_file=record_file,
            auto_start=not args.no_autostart,
            auto_loop=not args.no_loop,
        )
        loop.run()


if __name__ == '__main__':
    main()
