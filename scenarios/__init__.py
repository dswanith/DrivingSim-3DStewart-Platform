"""
scenarios — Synthetic driving scenario generators.

Each scenario generates realistic vehicle telemetry that passes through
the full SIL simulation pipeline:
    Scenario → TelemetryFrame → MCA → IK → ActuatorSim → WebSocket

All signals are in SI units (m, m/s, m/s², rad, rad/s).
Automotive body frame: X-forward, Y-left, Z-up.
"""
from .base import BaseScenario, ScenarioState
from .straight_road import StraightRoadScenario
from .pothole import PotholeScenario
from .speed_hump import SpeedHumpScenario
from .turn import TurnScenario
from .lane_change import LaneChangeScenario

SCENARIOS = {
    'straight': StraightRoadScenario,
    'pothole': PotholeScenario,
    'speed_hump': SpeedHumpScenario,
    'turn': TurnScenario,
    'lane_change': LaneChangeScenario,
}

__all__ = [
    'BaseScenario', 'ScenarioState',
    'StraightRoadScenario', 'PotholeScenario', 'SpeedHumpScenario',
    'TurnScenario', 'LaneChangeScenario',
    'SCENARIOS',
]
