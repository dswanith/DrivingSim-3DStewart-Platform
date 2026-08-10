"""
integration — Bridge between the Python SIL simulator and the browser visualizer.

Modules:
    sim_clock.py      — SimulationClock: single source of truth for time
    websocket_server.py — Async WebSocket publisher at ~60 Hz
    protocol.py       — WebSocket message schema and serialization
"""
from .sim_clock import SimulationClock
from .protocol import build_ws_message, validate_message

__all__ = ['SimulationClock', 'build_ws_message', 'validate_message']
