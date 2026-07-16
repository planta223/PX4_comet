"""RViz marker construction for one aligned EKF state."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _point(values) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(values[0]), float(values[1]), float(values[2]))
    return point


def _color(red: float, green: float, blue: float, alpha: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(red), g=float(green), b=float(blue), a=float(alpha))


def _finite_text(row, name: str, precision: int = 3) -> str:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError):
        return "n/a"
    return f"{value:.{precision}f}" if math.isfinite(value) else "n/a"


def _status_text(row, name: str) -> str:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError):
        return "n/a"
    return f"0x{int(value):X}" if math.isfinite(value) else "n/a"


class MarkerBuilder:
    def __init__(self, frame_id: str = "map", axis_length: float = 2.0, velocity_scale: float = 1.0) -> None:
        self.frame_id = frame_id
        self.axis_length = max(0.01, float(axis_length))
        self.velocity_scale = max(0.0, float(velocity_scale))

    def _base(self, stamp, namespace: str, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def current_position(self, stamp, state) -> Marker:
        marker = self._base(stamp, "current_position", 0, Marker.SPHERE)
        marker.pose.position = _point(state.position_enu)
        marker.scale.x = marker.scale.y = marker.scale.z = self.axis_length * 0.18
        marker.color = _color(1.0, 0.75, 0.05, 1.0)
        return marker

    def velocity(self, stamp, state) -> Marker:
        marker = self._base(stamp, "velocity", 0, Marker.ARROW)
        marker.points = [
            _point(state.position_enu),
            _point(state.position_enu + state.velocity_enu * self.velocity_scale),
        ]
        marker.scale.x = self.axis_length * 0.06
        marker.scale.y = self.axis_length * 0.12
        marker.scale.z = self.axis_length * 0.16
        marker.color = _color(1.0, 0.55, 0.0, 1.0)
        return marker

    def body_axes(self, stamp, state) -> MarkerArray:
        result = MarkerArray()
        colors = (_color(1.0, 0.0, 0.0), _color(0.0, 1.0, 0.0), _color(0.0, 0.35, 1.0))
        for axis_index, color in enumerate(colors):
            marker = self._base(stamp, "body_axes", axis_index, Marker.ARROW)
            marker.points = [
                _point(state.position_enu),
                _point(state.position_enu + state.body_axes_enu[:, axis_index] * self.axis_length),
            ]
            marker.scale.x = self.axis_length * 0.045
            marker.scale.y = self.axis_length * 0.09
            marker.scale.z = self.axis_length * 0.12
            marker.color = color
            result.markers.append(marker)
        return result

    def status_string(self, row, replay_time: float, playback_speed: float, paused: bool) -> str:
        mode = "PAUSED" if paused else "PLAY"
        return (
            f"{mode}  t={replay_time:.3f}s  speed={playback_speed:.3g}x\n"
            f"PN PE PD: {_finite_text(row, 'pn_m')}  {_finite_text(row, 'pe_m')}  {_finite_text(row, 'pd_m')} m\n"
            f"VN VE VD: {_finite_text(row, 'vn_mps')}  {_finite_text(row, 've_mps')}  {_finite_text(row, 'vd_mps')} m/s\n"
            f"RPY: {_finite_text(row, 'roll_deg')}  {_finite_text(row, 'pitch_deg')}  {_finite_text(row, 'yaw_deg')} deg\n"
            f"SV SP SH SM: {_finite_text(row, 'vel_test_ratio')}  {_finite_text(row, 'pos_test_ratio')}  "
            f"{_finite_text(row, 'height_test_ratio')}  {_finite_text(row, 'mag_test_ratio')}\n"
            f"FS TS SS: {_status_text(row, 'fault_status')}  {_status_text(row, 'timeout_status')}  {_status_text(row, 'solution_status')}"
        )

    def status_marker(self, stamp, state, text: str) -> Marker:
        marker = self._base(stamp, "status", 0, Marker.TEXT_VIEW_FACING)
        marker.pose.position = _point(state.position_enu + [0.0, 0.0, self.axis_length * 1.6])
        marker.scale.z = self.axis_length * 0.22
        marker.color = _color(1.0, 1.0, 1.0, 1.0)
        marker.text = text
        return marker
