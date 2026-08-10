import pytest
import math
import numpy as np

from scenarios.straight_road import StraightRoadScenario
from scenarios.pothole import PotholeScenario
from scenarios.speed_hump import SpeedHumpScenario
from scenarios.turn import TurnScenario
from scenarios.lane_change import LaneChangeScenario


def test_straight_road():
    scenario = StraightRoadScenario(duration_s=2.0)
    frames = list(scenario)
    assert len(frames) in [120, 121]  # 60 Hz * 2s
    
    # Check that straight road produces small vertical noise but no lateral/longitudinal accel
    max_az = max(abs(f.accel_xyz[2]) for f in frames)
    max_ay = max(abs(f.accel_xyz[1]) for f in frames)
    max_ax = max(abs(f.accel_xyz[0]) for f in frames)
    
    assert max_az > 0.1  # There is some noise
    assert max_az < 2.0  # Not too much noise
    assert max_ax < 1e-5
    assert max_ay < 0.5  # Only tiny lateral wander

def test_pothole():
    scenario = PotholeScenario(duration_s=6.0)
    # The pothole impact is at 4.0s
    frames = list(scenario)
    
    az_max = max(f.accel_xyz[2] for f in frames)
    az_min = min(f.accel_xyz[2] for f in frames)
    
    # Impact should produce significant vertical acceleration
    assert az_max > 10.0  # m/s2
    assert az_min < -10.0 # m/s2
    
    # Should pitch down then oscillate
    pitch_max = max(f.orientation_rpy[1] for f in frames)
    pitch_min = min(f.orientation_rpy[1] for f in frames)
    assert pitch_min < -0.01  # Pitches down significantly
    assert pitch_max > 0.005  # Rebounds

def test_speed_hump():
    scenario = SpeedHumpScenario(duration_s=6.0)
    frames = list(scenario)
    
    az_max = max(f.accel_xyz[2] for f in frames)
    pitch_max = max(f.orientation_rpy[1] for f in frames)
    
    assert az_max > 1.5  # Actually ~1.89 m/s2
    assert pitch_max > 0.005 # Nose pitches up slightly

def test_turn():
    scenario = TurnScenario(duration_s=15.0, direction='left', radius_m=40.0)
    frames = list(scenario)
    
    # Left turn: lateral acceleration is negative (pushes right in vehicle frame)
    ay_min = min(f.accel_xyz[1] for f in frames)
    assert ay_min < -1.0
    
    # Vehicle rolls right (negative roll)
    roll_min = min(f.orientation_rpy[0] for f in frames)
    assert roll_min < -0.01
    
    # Yaw rate is positive (counter-clockwise)
    yaw_rate_max = max(f.ang_vel_xyz[2] for f in frames)
    assert yaw_rate_max > 0.1

def test_lane_change():
    scenario = LaneChangeScenario(duration_s=12.0)
    frames = list(scenario)
    
    ay_max = max(f.accel_xyz[1] for f in frames)
    ay_min = min(f.accel_xyz[1] for f in frames)
    
    assert ay_max > 1.0
    assert ay_min < -1.0
