"""Convert the aligned DataFlash state schema to ROS ENU pose values."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from bin_ekf_replay.state_model import quaternion_body_axes_enu


@dataclass(frozen=True)
class RosState:
    position_enu: np.ndarray
    velocity_enu: np.ndarray
    quaternion_xyzw: np.ndarray
    body_axes_enu: np.ndarray


def model_axis_offset_matrix(rpy_degrees: Sequence[float]) -> np.ndarray:
    values = np.asarray(rpy_degrees, dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("model_axis_offset_rpy_deg must contain three finite values")
    return Rotation.from_euler("xyz", values, degrees=True).as_matrix()


def state_row_to_ros(
    row,
    use_quaternion_conjugate: bool = False,
    model_axis_offset_rpy_deg: Sequence[float] = (0.0, 0.0, 0.0),
) -> RosState:
    """Return ENU pose values, with ROS quaternion ordered x,y,z,w."""
    position = np.array([row["x_east_m"], row["y_north_m"], row["z_up_m"]], dtype=float)
    velocity = np.array([row["vx_east_mps"], row["vy_north_mps"], row["vz_up_mps"]], dtype=float)
    quaternion_wxyz = np.array([row["qw"], row["qx"], row["qy"], row["qz"]], dtype=float)
    offset = model_axis_offset_matrix(model_axis_offset_rpy_deg)
    axes = quaternion_body_axes_enu(
        quaternion_wxyz,
        use_conjugate=use_quaternion_conjugate,
        model_axis_offset=offset,
    )
    quaternion_xyzw = Rotation.from_matrix(axes).as_quat()
    quaternion_xyzw /= np.linalg.norm(quaternion_xyzw)
    return RosState(position, velocity, quaternion_xyzw, axes)
