"""Wall-clock anchored replay timing, independent of ROS timer jitter."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Optional

import numpy as np


@dataclass(frozen=True)
class ReplaySnapshot:
    index: int
    replay_time: float
    paused: bool
    reached_end: bool


class ReplayController:
    """Map monotonic wall time to the nearest recorded state timestamp."""

    def __init__(
        self,
        sample_times,
        playback_speed: float = 1.0,
        loop: bool = False,
        start_paused: bool = False,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.times = np.asarray(sample_times, dtype=float)
        if self.times.ndim != 1 or len(self.times) == 0:
            raise ValueError("ReplayController requires at least one timestamp")
        if not np.all(np.isfinite(self.times)) or np.any(np.diff(self.times) < 0):
            raise ValueError("Replay timestamps must be finite and monotonic")
        if playback_speed <= 0.0:
            raise ValueError("playback_speed must be greater than zero")
        self.start_time = float(self.times[0])
        self.end_time = float(self.times[-1])
        self.duration = max(0.0, self.end_time - self.start_time)
        self.speed = float(playback_speed)
        self.loop = bool(loop)
        self.paused = bool(start_paused)
        self._clock = clock
        self._anchor_wall = self._clock()
        self._anchor_log = self.start_time
        self._reached_end = False

    def _log_time_at(self, now: float, update_end_state: bool = True) -> float:
        if self.paused:
            return self._anchor_log
        target = self._anchor_log + (now - self._anchor_wall) * self.speed
        if self.loop and self.duration > 0.0:
            return self.start_time + ((target - self.start_time) % self.duration)
        if target >= self.end_time:
            if update_end_state:
                self._anchor_log = self.end_time
                self._anchor_wall = now
                self.paused = True
                self._reached_end = True
            return self.end_time
        return max(self.start_time, target)

    def _nearest_index(self, replay_time: float) -> int:
        right = int(np.searchsorted(self.times, replay_time, side="left"))
        if right <= 0:
            return 0
        if right >= len(self.times):
            return len(self.times) - 1
        left = right - 1
        return left if replay_time - self.times[left] <= self.times[right] - replay_time else right

    def snapshot(self, now: Optional[float] = None) -> ReplaySnapshot:
        wall = self._clock() if now is None else float(now)
        replay_time = self._log_time_at(wall)
        return ReplaySnapshot(self._nearest_index(replay_time), replay_time, self.paused, self._reached_end)

    def set_paused(self, paused: bool, now: Optional[float] = None) -> None:
        wall = self._clock() if now is None else float(now)
        if bool(paused) == self.paused:
            return
        if paused:
            self._anchor_log = self._log_time_at(wall, update_end_state=False)
            self._anchor_wall = wall
            self.paused = True
        else:
            self._anchor_wall = wall
            self.paused = False
            self._reached_end = False

    def set_speed(self, speed: float, now: Optional[float] = None) -> None:
        if speed <= 0.0 or not np.isfinite(speed):
            raise ValueError("playback_speed must be finite and greater than zero")
        wall = self._clock() if now is None else float(now)
        current = self._log_time_at(wall, update_end_state=False)
        self._anchor_log = current
        self._anchor_wall = wall
        self.speed = float(speed)

    def set_loop(self, loop: bool, now: Optional[float] = None) -> None:
        wall = self._clock() if now is None else float(now)
        current = self._log_time_at(wall, update_end_state=False)
        self._anchor_log = current
        self._anchor_wall = wall
        self.loop = bool(loop)

    def reset(self, resume: bool = True, now: Optional[float] = None) -> None:
        wall = self._clock() if now is None else float(now)
        self._anchor_log = self.start_time
        self._anchor_wall = wall
        self.paused = not resume
        self._reached_end = False
