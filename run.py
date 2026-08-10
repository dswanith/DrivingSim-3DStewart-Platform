#!/usr/bin/env python
"""
Stewart-Platform 6-DOF SIL Simulator — Entrypoint.

Usage:
    python run.py --mode synthetic         # Run with synthetic telemetry (default)
    python run.py --mode carla             # Run with CARLA telemetry
    python run.py --mode synthetic --dashboard   # With live dashboard
    python run.py --mode synthetic --no-log      # Without logging
    python run.py --mode synthetic --realtime    # Pace to 60 Hz (default: fast)

The simulator runs fully in synthetic mode with CARLA absent.
CARLA integration requires a running CARLA server and the carla Python package.
"""

import argparse
import logging
import sys
import os

# Ensure the project root is on the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sil_sim.config.loader import load_all, ConfigError
from sil_sim.telemetry.synthetic import SyntheticSource
from sil_sim.simulation.pipeline import SimPipeline
from sil_sim.simulation.sim_loop import SimLoop
from sil_sim.logging.logger import SimLogger
from sil_sim.visualization.dashboard import DashboardServer


def main():
    parser = argparse.ArgumentParser(
        description="Stewart-Platform 6-DOF SIL Simulator",
    )
    parser.add_argument(
        '--mode', choices=['synthetic', 'carla'], default='synthetic',
        help='Telemetry source mode (default: synthetic)',
    )
    parser.add_argument(
        '--dashboard', action='store_true',
        help='Launch the live engineering dashboard',
    )
    parser.add_argument(
        '--no-log', action='store_true',
        help='Disable CSV/JSONL logging',
    )
    parser.add_argument(
        '--realtime', action='store_true',
        help='Pace simulation to ~60 Hz (default: run as fast as possible)',
    )
    parser.add_argument(
        '--duration', type=float, default=10.0,
        help='Synthetic mode duration in seconds (default: 10)',
    )
    parser.add_argument(
        '--signal', default='sine',
        choices=['sine', 'step', 'ramp', 'chirp', 'combined'],
        help='Synthetic signal type (default: sine)',
    )
    parser.add_argument(
        '--http-port', type=int, default=8080,
        help='Dashboard HTTP port (default: 8080)',
    )
    parser.add_argument(
        '--ws-port', type=int, default=8081,
        help='Dashboard WebSocket port (default: 8081)',
    )
    args = parser.parse_args()

    # --- Logging setup ---
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        datefmt='%H:%M:%S',
    )
    log = logging.getLogger('sil_sim')

    # --- Load configs ---
    log.info("Loading configuration...")
    try:
        configs = load_all()
    except ConfigError as e:
        log.error("Configuration error: %s", e)
        sys.exit(1)
    log.info("Configuration loaded and validated successfully.")

    # --- Create telemetry source ---
    if args.mode == 'synthetic':
        telemetry = SyntheticSource(
            rate_hz=configs['mca']['input_rate_hz'],
            signal_type=args.signal,
            duration_s=args.duration,
        )
        log.info("Synthetic telemetry: signal=%s, duration=%.1fs, rate=%.0fHz",
                 args.signal, args.duration, configs['mca']['input_rate_hz'])
    elif args.mode == 'carla':
        try:
            from sil_sim.telemetry.carla_source import CarlaSource
            telemetry = CarlaSource(configs['carla'])
            if not telemetry.is_connected():
                log.warning("CARLA source failed to connect or is unavailable — falling back to synthetic")
                telemetry = SyntheticSource(
                    rate_hz=configs['mca']['input_rate_hz'],
                    signal_type=args.signal,
                    duration_s=args.duration,
                )
        except NotImplementedError:
            log.warning("CARLA source not yet implemented — falling back to synthetic")
            telemetry = SyntheticSource(
                rate_hz=configs['mca']['input_rate_hz'],
                signal_type=args.signal,
                duration_s=args.duration,
            )
        except Exception as e:
            log.warning("CARLA connection failed: %s — falling back to synthetic", e)
            telemetry = SyntheticSource(
                rate_hz=configs['mca']['input_rate_hz'],
                signal_type=args.signal,
                duration_s=args.duration,
            )

    # --- Create pipeline ---
    pipeline = SimPipeline(configs, telemetry)

    # --- Logger ---
    sim_logger = None
    if not args.no_log:
        sim_logger = SimLogger(output_dir="logs", prefix="sil")
        log.info("Logging to: %s", sim_logger.csv_path)

    # --- Dashboard ---
    dashboard = None
    if args.dashboard:
        dashboard = DashboardServer(
            http_port=args.http_port,
            ws_port=args.ws_port,
        )
        dashboard.start()
        log.info("Dashboard: http://localhost:%d", args.http_port)

    # --- Run ---
    loop = SimLoop(
        pipeline=pipeline,
        sim_logger=sim_logger,
        dashboard=dashboard,
        realtime=args.realtime,
        target_rate_hz=configs['mca']['input_rate_hz'],
    )

    try:
        total = loop.run()
        log.info("Simulation complete: %d frames processed", total)
    finally:
        if sim_logger:
            sim_logger.close()
        if dashboard:
            dashboard.stop()

    return 0


if __name__ == '__main__':
    sys.exit(main())
