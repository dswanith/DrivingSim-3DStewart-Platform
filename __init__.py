# sil_sim — Stewart-Platform 6-DOF Software-in-the-Loop Simulator
"""
Top-level package for the SIL simulator.

Output-class discipline:
    EXACT      — pure geometry / kinematics / math (no model assumptions)
    ESTIMATED  — model-based (electrical, friction, efficiency estimates)
    MEASURED   — real sensor data (NEVER produced by this simulator)
"""

__version__ = "0.1.0"
