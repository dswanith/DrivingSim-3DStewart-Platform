"""
integration.sim_clock — Single simulation clock.

The SimulationClock is the single source of truth for:
    - simulation_time   (seconds, starts at 0.0)
    - dt                (seconds, fixed timestep)
    - frame_index       (integer, monotonically increasing)
    - wall_clock_time   (seconds, actual wall time at each tick)

All pipeline modules (MCA, IK, actuator, logger, WebSocket) receive
the SimulationClock and use its time, not system time.

This ensures that vehicle telemetry, platform pose, actuator states,
and browser visualization all share a synchronized timestamp.
"""
import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ClockTick:
    """Data class representing one simulation clock tick."""
    simulation_time: float      # seconds (simulation time)
    dt: float                   # seconds (fixed timestep)
    frame_index: int            # monotonically increasing frame counter
    wall_time: float            # seconds (actual wall clock)


class SimulationClock:
    """
    Deterministic simulation clock.

    Usage:
        clock = SimulationClock(rate_hz=60.0)
        for tick in clock:          # iterates in real time if realtime=True
            pipeline.step(tick)
    """

    def __init__(
        self,
        rate_hz: float = 60.0,
        realtime: bool = True,
        duration_s: Optional[float] = None,
    ):
        """
        Args:
            rate_hz: Simulation frequency (Hz). Default 60 Hz.
            realtime: If True, pace to real-world time. If False, run as fast as possible.
            duration_s: Total simulation duration (seconds). None = run indefinitely.
        """
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.realtime = realtime
        self.duration_s = duration_s

        self._sim_time: float = 0.0
        self._frame_index: int = 0
        self._start_wall: Optional[float] = None
        self._running: bool = False

    @property
    def simulation_time(self) -> float:
        return self._sim_time

    @property
    def frame_index(self) -> int:
        return self._frame_index

    @property
    def is_running(self) -> bool:
        return self._running

    def reset(self):
        """Reset clock to t=0."""
        self._sim_time = 0.0
        self._frame_index = 0
        self._start_wall = None
        self._running = False

    def tick(self) -> ClockTick:
        """
        Advance clock by one dt and return a ClockTick.
        If realtime=True, sleeps to maintain pace.
        """
        if self._start_wall is None:
            self._start_wall = time.monotonic()
            self._running = True

        # Build tick
        tick = ClockTick(
            simulation_time=self._sim_time,
            dt=self.dt,
            frame_index=self._frame_index,
            wall_time=time.monotonic() - self._start_wall,
        )

        # Advance
        self._sim_time += self.dt
        self._frame_index += 1

        # Real-time pacing
        if self.realtime:
            target_wall = self._frame_index * self.dt
            actual_wall = time.monotonic() - self._start_wall
            sleep_time = target_wall - actual_wall
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        return tick

    def is_complete(self) -> bool:
        """Return True if duration has been reached."""
        if self.duration_s is None:
            return False
        return self._sim_time >= self.duration_s

    def __iter__(self):
        """Iterate, yielding ClockTick objects until complete."""
        self.reset()
        while not self.is_complete():
            yield self.tick()
