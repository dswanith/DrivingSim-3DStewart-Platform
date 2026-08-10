#!/usr/bin/env python
import os
import sys
import csv
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sil_sim.config.loader import load_all
from sil_sim.telemetry.base import TelemetrySource, TelemetryFrame
from sil_sim.simulation.pipeline import SimPipeline
from sil_sim.simulation.sim_loop import SimLoop
from sil_sim.visualization.dashboard import DashboardServer

class CsvSource(TelemetrySource):
    """
    CSV telemetry source for offline playback of CARLA recordings.

    IMPORTANT: The CSV data (acc_x/y/z, angular_x/y/z, vel_x/y/z) is in
    CARLA's WORLD (global) coordinate frame, NOT the vehicle's body frame.
    The world→body frame rotation is handled inside the MCA pipeline
    (Step 0: frame_transform.py) using the vehicle's own orientation
    (roll, pitch, yaw from the same CSV row). Do NOT pre-rotate the data
    here — the MCA expects world-frame input.
    """
    def __init__(self, csv_path):
        self.rows = []
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.rows.append(row)
        self.idx = 0
        
    def get_frame(self):
        if self.idx >= len(self.rows):
            return None
        row = self.rows[self.idx]
        self.idx += 1
        
        timestamp = float(row['time'])
        accel_xyz = np.array([float(row['acc_x']), float(row['acc_y']), float(row['acc_z'])])
        ang_vel_xyz = np.array([float(row['angular_x']), float(row['angular_y']), float(row['angular_z'])])
        
        # CARLA orientation is typically in degrees, converting to radians
        roll_rad = np.deg2rad(float(row['roll']))
        pitch_rad = np.deg2rad(float(row['pitch']))
        yaw_rad = np.deg2rad(float(row['yaw']))
        orientation_rpy = np.array([roll_rad, pitch_rad, yaw_rad])
        
        velocity_xyz = np.array([float(row['vel_x']), float(row['vel_y']), float(row['vel_z'])])
        
        return TelemetryFrame(
            timestamp=timestamp,
            accel_xyz=accel_xyz,
            ang_vel_xyz=ang_vel_xyz,
            orientation_rpy=orientation_rpy,
            velocity_xyz=velocity_xyz
        )
        
    def is_connected(self): return True
    def reset(self): self.idx = 0
    @property
    def source_name(self): return "csv"

def main():
    configs = load_all()
    csv_file = "telemetry_20260808_141112.csv"
    if not os.path.exists(csv_file):
        print(f"Error: Could not find {csv_file}")
        return
        
    telemetry = CsvSource(csv_file)
    pipeline = SimPipeline(configs, telemetry)
    
    dashboard = DashboardServer(http_port=8080, ws_port=8081)
    dashboard.start()
    
    print("--------------------------------------------------")
    print("Dashboard running at http://localhost:8080")
    print(f"Playing back {csv_file} in real-time...")
    print("--------------------------------------------------")
    
    loop = SimLoop(
        pipeline=pipeline,
        sim_logger=None,
        dashboard=dashboard,
        realtime=True,
        target_rate_hz=configs['mca']['input_rate_hz']
    )
    
    try:
        loop.run()
    except KeyboardInterrupt:
        print("Playback stopped.")
    finally:
        dashboard.stop()

if __name__ == '__main__':
    main()
