"""Matplotlib 3D animation for aligned EKF states."""

from __future__ import annotations

import itertools
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation

from state_model import quaternion_body_axes_enu


class EKFReplayVisualizer:
    def __init__(
        self,
        states: pd.DataFrame,
        fps: float = 30.0,
        speed: float = 1.0,
        trail_seconds: Optional[float] = 30.0,
        show_health: bool = True,
        use_quaternion_conjugate: bool = False,
    ) -> None:
        if states.empty:
            raise ValueError("표시할 상태 샘플이 없습니다.")
        self.states = states
        self.times = states["time_s"].to_numpy(float)
        self.positions = states[["x_east_m", "y_north_m", "z_up_m"]].to_numpy(float)
        self.velocities = states[["vx_east_mps", "vy_north_mps", "vz_up_mps"]].to_numpy(float)
        self.quaternions = states[["qw", "qx", "qy", "qz"]].to_numpy(float)
        self.fps = max(1.0, float(fps))
        self.speed = max(0.01, float(speed))
        self.trail_seconds = trail_seconds if trail_seconds is None else max(0.0, float(trail_seconds))
        self.show_health = show_health and states["vel_test_ratio"].notna().any()
        self.use_quaternion_conjugate = use_quaternion_conjugate
        self.playhead_s = float(self.times[0])
        self.paused = False
        self.current_index = 0
        self._last_wall_time: Optional[float] = None

        self.fig = plt.figure(figsize=(13, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlabel("East x [m]")
        self.ax.set_ylabel("North y [m]")
        self.ax.set_zlabel("Up z [m]")
        self.ax.set_title("ArduPilot EKF3 DataFlash Replay")
        self._set_equal_limits()

        valid_position = np.all(np.isfinite(self.positions), axis=1)
        full = self.positions[valid_position]
        self.full_line, = self.ax.plot(full[:, 0], full[:, 1], full[:, 2], color="0.7", lw=1.0, alpha=0.55, label="full path")
        self.trail_line, = self.ax.plot([], [], [], color="tab:blue", lw=2.0, label="trail")
        self.point, = self.ax.plot([], [], [], marker="o", color="black", markersize=6, linestyle="None", label="current")
        colors = ("tab:red", "tab:green", "tab:blue")
        self.body_lines = [self.ax.plot([], [], [], color=color, lw=2.5)[0] for color in colors]
        self.body_labels = [self.ax.text(0, 0, 0, label, color=color) for label, color in zip(("body X", "body Y", "body Z"), colors)]
        self.velocity_line, = self.ax.plot([], [], [], color="tab:orange", lw=2.0, label="velocity")
        self.info_text = self.ax.text2D(0.02, 0.98, "", transform=self.ax.transAxes, va="top", family="monospace")
        self.health_text = self.ax.text2D(0.68, 0.98, "", transform=self.ax.transAxes, va="top", family="monospace")
        self.speed_text = self.ax.text2D(0.02, 0.03, "", transform=self.ax.transAxes, family="monospace")
        self.ax.legend(loc="lower right")

        self.axis_length = self._display_scale() * 0.08
        self.velocity_scale = max(0.25, self.axis_length / max(self._finite_speed_percentile(), 0.1))
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.animation: Optional[FuncAnimation] = None

    def _display_scale(self) -> float:
        finite = self.positions[np.all(np.isfinite(self.positions), axis=1)]
        if len(finite) == 0:
            return 1.0
        return max(float(np.max(np.ptp(finite, axis=0))), 1.0)

    def _finite_speed_percentile(self) -> float:
        speeds = np.linalg.norm(self.velocities, axis=1)
        speeds = speeds[np.isfinite(speeds)]
        return float(np.percentile(speeds, 90)) if len(speeds) else 1.0

    def _set_equal_limits(self) -> None:
        finite = self.positions[np.all(np.isfinite(self.positions), axis=1)]
        if len(finite) == 0:
            self.ax.set_xlim(-1, 1); self.ax.set_ylim(-1, 1); self.ax.set_zlim(-1, 1)
            return
        low, high = np.min(finite, axis=0), np.max(finite, axis=0)
        center = (low + high) * 0.5
        radius = max(float(np.max(high - low)) * 0.55, 1.0)
        self.ax.set_xlim(center[0] - radius, center[0] + radius)
        self.ax.set_ylim(center[1] - radius, center[1] + radius)
        self.ax.set_zlim(center[2] - radius, center[2] + radius)
        try:
            self.ax.set_box_aspect((1, 1, 1))
        except AttributeError:
            pass

    @staticmethod
    def _set_line3d(line, points: np.ndarray) -> None:
        line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])

    @staticmethod
    def _value(row: pd.Series, name: str) -> str:
        value = row.get(name, np.nan)
        return "n/a" if not np.isfinite(value) else f"{value:.3f}"

    def _update_artists(self, index: int) -> tuple:
        row = self.states.iloc[index]
        position = self.positions[index]
        if self.trail_seconds is None:
            start = 0
        else:
            start_time = self.times[index] - self.trail_seconds
            start = int(np.searchsorted(self.times, start_time, side="left"))
        trail = self.positions[start:index + 1]
        valid_trail = trail[np.all(np.isfinite(trail), axis=1)]
        if len(valid_trail):
            self._set_line3d(self.trail_line, valid_trail)
        if np.all(np.isfinite(position)):
            self._set_line3d(self.point, position.reshape(1, 3))
            axes = quaternion_body_axes_enu(self.quaternions[index], self.use_quaternion_conjugate)
            for axis_index, (line, label) in enumerate(zip(self.body_lines, self.body_labels)):
                endpoint = position + axes[:, axis_index] * self.axis_length
                self._set_line3d(line, np.vstack((position, endpoint)))
                label.set_position_3d((endpoint[0], endpoint[1], endpoint[2]))
            velocity = self.velocities[index]
            velocity_endpoint = position + velocity * self.velocity_scale if np.all(np.isfinite(velocity)) else position
            self._set_line3d(self.velocity_line, np.vstack((position, velocity_endpoint)))

        self.info_text.set_text(
            f"time {row['time_s']:.3f} s\n"
            f"PN PE PD  {self._value(row, 'pn_m')}  {self._value(row, 'pe_m')}  {self._value(row, 'pd_m')} m\n"
            f"VN VE VD  {self._value(row, 'vn_mps')}  {self._value(row, 've_mps')}  {self._value(row, 'vd_mps')} m/s\n"
            f"RPY       {self._value(row, 'roll_deg')}  {self._value(row, 'pitch_deg')}  {self._value(row, 'yaw_deg')} deg"
        )
        if self.show_health:
            self.health_text.set_text(
                "XKF4 health\n"
                f"SV SP SH SM  {self._value(row, 'vel_test_ratio')}  {self._value(row, 'pos_test_ratio')}  "
                f"{self._value(row, 'height_test_ratio')}  {self._value(row, 'mag_test_ratio')}\n"
                f"FS TS SS     {self._value(row, 'fault_status')}  {self._value(row, 'timeout_status')}  {self._value(row, 'solution_status')}"
            )
        else:
            self.health_text.set_text("XKF4 health: unavailable/disabled")
        state = "PAUSED" if self.paused else "PLAY"
        self.speed_text.set_text(f"{state}  speed {self.speed:.3g}x   [Space pause, arrows seek/speed, R restart, Esc quit]")
        return (self.trail_line, self.point, self.velocity_line, self.info_text, self.health_text, self.speed_text, *self.body_lines, *self.body_labels)

    def _animate(self, _frame: int) -> tuple:
        import time
        now = time.monotonic()
        if self._last_wall_time is None:
            elapsed = 1.0 / self.fps
        else:
            elapsed = min(now - self._last_wall_time, 0.5)
        self._last_wall_time = now
        if not self.paused:
            self.playhead_s += elapsed * self.speed
            if self.playhead_s >= self.times[-1]:
                self.playhead_s = float(self.times[-1])
                self.paused = True
        self.current_index = min(int(np.searchsorted(self.times, self.playhead_s, side="right") - 1), len(self.times) - 1)
        self.current_index = max(0, self.current_index)
        return self._update_artists(self.current_index)

    def _on_key(self, event) -> None:
        key = (event.key or "").lower()
        if key in (" ", "space"):
            self.paused = not self.paused
            self._last_wall_time = None
        elif key == "r":
            self.playhead_s = float(self.times[0])
            self.current_index = 0
            self.paused = False
            self._last_wall_time = None
        elif key == "right":
            self.playhead_s = min(float(self.times[-1]), self.playhead_s + 1.0)
        elif key == "left":
            self.playhead_s = max(float(self.times[0]), self.playhead_s - 1.0)
        elif key == "up":
            self.speed = min(64.0, self.speed * 2.0)
        elif key == "down":
            self.speed = max(0.03125, self.speed / 2.0)
        elif key in ("escape", "esc"):
            plt.close(self.fig)
        self._update_artists(self.current_index)
        self.fig.canvas.draw_idle()

    def show(self) -> None:
        self.animation = FuncAnimation(
            self.fig,
            self._animate,
            frames=itertools.count(),
            interval=1000.0 / self.fps,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()


def replay_states(
    states: pd.DataFrame,
    fps: float,
    speed: float,
    trail_seconds: Optional[float],
    show_health: bool,
    use_quaternion_conjugate: bool,
) -> None:
    EKFReplayVisualizer(states, fps, speed, trail_seconds, show_health, use_quaternion_conjugate).show()
