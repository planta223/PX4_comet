#!/usr/bin/env python3
"""
03_reconstruct_trajectory.py

Trajectory analysis pipeline step 3.
Read the prepared CSV files from 02_prepare_trajectory_csv.py and reconstruct
the trajectory in one of three modes:

- gps-only:
    Use GPS local coordinates directly for x/y/z.
- gps-baro:
    Use GPS local coordinates for x/y and BARO relative altitude for z.
- imu-gps:
    Keep the existing RK4 IMU integration with GPS position correction.
    This mode is experimental.

Inputs:
    output/02_trajectory_inputs_<log_id>/imu.csv
    output/02_trajectory_inputs_<log_id>/gps.csv
    output/02_trajectory_inputs_<log_id>/baro.csv

Outputs:
    output/03_trajectory_<log_id>/trajectory.csv
    output/03_trajectory_<log_id>/gps_local.csv
    output/03_trajectory_<log_id>/reconstruct_summary.md

Notes:
    - The default recommended mode is gps-baro.
    - 04_plot_trajectory.py reads the generated trajectory.csv directly.
    - Raw ArduPilot IMU AccX/AccY/AccZ are likely body-frame values, so
      imu-gps must not be treated as a precision trajectory estimator unless
      attitude rotation is added first.
"""

# =====================================================================
# Usage examples
# ---------------------------------------------------------------------
# Recommended default:
#
#   python scripts/03_reconstruct_trajectory.py \
#     --mode gps-baro \
#     --imu output/02_trajectory_inputs_00000197/imu.csv \
#     --gps output/02_trajectory_inputs_00000197/gps.csv \
#     --baro output/02_trajectory_inputs_00000197/baro.csv \
#     --out output/03_trajectory_00000197/trajectory.csv \
#     --gps-local-out output/03_trajectory_00000197/gps_local.csv \
#     --summary-out output/03_trajectory_00000197/reconstruct_summary.md
#
# GPS-only:
#
#   python scripts/03_reconstruct_trajectory.py \
#     --mode gps-only \
#     --imu output/02_trajectory_inputs_00000197/imu.csv \
#     --gps output/02_trajectory_inputs_00000197/gps.csv \
#     --out output/03_trajectory_00000197/trajectory.csv \
#     --gps-local-out output/03_trajectory_00000197/gps_local.csv \
#     --summary-out output/03_trajectory_00000197/reconstruct_summary.md
#
# IMU-GPS experimental:
#
#   python scripts/03_reconstruct_trajectory.py \
#     --mode imu-gps \
#     --imu output/02_trajectory_inputs_00000197/imu.csv \
#     --gps output/02_trajectory_inputs_00000197/gps.csv \
#     --baro output/02_trajectory_inputs_00000197/baro.csv \
#     --dt 0.02 \
#     --gps-blend 1.0 \
#     --out output/03_trajectory_00000197/trajectory.csv \
#     --gps-local-out output/03_trajectory_00000197/gps_local.csv \
#     --summary-out output/03_trajectory_00000197/reconstruct_summary.md
# =====================================================================

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd


EARTH_RADIUS_M = 6378137.0
TRAJECTORY_BASE_COLUMNS = ["time_s", "x", "y", "z", "vx", "vy", "vz", "lat", "lng"]
TRAJECTORY_METADATA_COLUMNS = ["source_mode", "alt_source"]
MODE_CHOICES = ("gps-only", "gps-baro", "imu-gps")
EXPERIMENTAL_WARNING = (
    "WARNING: imu-gps mode assumes IMU acceleration is already expressed in an "
    "earth-fixed frame. Raw ArduPilot IMU AccX/AccY/AccZ are likely body-frame, "
    "so this mode is experimental and not valid for precision trajectory "
    "estimation without attitude rotation."
)


# =====================================================================
# [1] Common utilities
# =====================================================================

def require_columns(df: pd.DataFrame, required: set[str], name: str) -> None:
    """
    Validate that required columns exist in the DataFrame.
    """
    missing = required - set(df.columns)

    if missing:
        raise ValueError(
            f"{name} CSV is missing required columns: {sorted(missing)}\n"
            f"Actual columns: {list(df.columns)}"
        )


def validate_file(path: Path, name: str) -> None:
    """
    Validate that an input file exists and is a regular file.
    """
    if not path.exists():
        raise FileNotFoundError(f"{name} file not found: {path}")

    if not path.is_file():
        raise ValueError(f"{name} path is not a file: {path}")


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    Clamp a value to the given range.
    """
    return max(min_value, min(max_value, value))


def describe_path(path: Path | None) -> str:
    """
    Render a path for summary output.
    """
    return str(path) if path is not None else "N/A"


def validate_time_axis(time_s: np.ndarray, name: str) -> None:
    """
    Validate that time values are finite and strictly increasing.
    """
    if len(time_s) < 2:
        raise ValueError(f"{name} requires at least 2 time samples.")

    if not np.isfinite(time_s).all():
        raise ValueError(f"{name} contains non-finite time values.")

    dt = np.diff(time_s)

    if np.any(dt <= 0):
        raise ValueError(
            f"{name} time_s must be strictly increasing after preprocessing."
        )


def safe_gradient(values: np.ndarray, time_s: np.ndarray, name: str) -> np.ndarray:
    """
    Compute d(values)/d(time) with validation for irregular time steps.
    """
    values = np.asarray(values, dtype=float)
    time_s = np.asarray(time_s, dtype=float)

    if len(values) != len(time_s):
        raise ValueError(
            f"{name} gradient length mismatch: values={len(values)}, time={len(time_s)}"
        )

    if not np.isfinite(values).all():
        raise ValueError(f"{name} contains non-finite values.")

    validate_time_axis(time_s, name=f"{name} gradient")

    edge_order = 1 if len(values) < 3 else 2

    return np.gradient(values, time_s, edge_order=edge_order)


def build_gradient_velocity(
    time_s: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Estimate vx/vy/vz from position time series.
    """
    vx = safe_gradient(x, time_s, name="x")
    vy = safe_gradient(y, time_s, name="y")
    vz = safe_gradient(z, time_s, name="z")

    return vx, vy, vz


def apply_trajectory_metadata(
    trajectory_df: pd.DataFrame,
    source_mode: str,
    alt_source: str,
) -> pd.DataFrame:
    """
    Attach metadata columns while keeping the expected column order.
    """
    out = trajectory_df.copy()
    out["source_mode"] = source_mode
    out["alt_source"] = alt_source

    return out[TRAJECTORY_BASE_COLUMNS + TRAJECTORY_METADATA_COLUMNS]


# =====================================================================
# [2] CSV loading
# =====================================================================

def load_prepared_imu_csv(path: Path | None) -> pd.DataFrame | None:
    """
    Read imu.csv generated by 02_prepare_trajectory_csv.py.

    Required columns:
        time_s, ax, ay, az

    Optional columns:
        gx, gy, gz
    """
    if path is None:
        return None

    validate_file(path, "IMU")

    df = pd.read_csv(path)

    require_columns(df, {"time_s", "ax", "ay", "az"}, "IMU")

    keep_cols = ["time_s", "ax", "ay", "az"]

    for col in ["gx", "gy", "gz"]:
        if col in df.columns:
            keep_cols.append(col)

    df = df[keep_cols].copy()
    df = df.sort_values("time_s").dropna(subset=["time_s", "ax", "ay", "az"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 2:
        raise ValueError("IMU data is too short. At least 2 rows are required.")

    return df


def load_prepared_gps_csv(path: Path) -> pd.DataFrame:
    """
    Read gps.csv generated by 02_prepare_trajectory_csv.py.

    Required columns:
        time_s, lat, lng, alt

    Optional columns:
        spd, vz, nsats, hdop, status
    """
    validate_file(path, "GPS")

    df = pd.read_csv(path)

    require_columns(df, {"time_s", "lat", "lng", "alt"}, "GPS")

    keep_cols = ["time_s", "lat", "lng", "alt"]

    for col in ["spd", "vz", "nsats", "hdop", "status"]:
        if col in df.columns:
            keep_cols.append(col)

    df = df[keep_cols].copy()
    df = df.sort_values("time_s").dropna(subset=["time_s", "lat", "lng", "alt"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 2:
        raise ValueError(
            "GPS data is too short. At least 2 rows are required for reconstruction."
        )

    return df


def load_prepared_baro_csv(path: Path | None) -> pd.DataFrame | None:
    """
    Read baro.csv generated by 02_prepare_trajectory_csv.py.

    Required columns:
        time_s, baro_alt
    """
    if path is None:
        return None

    validate_file(path, "BARO")

    df = pd.read_csv(path)

    require_columns(df, {"time_s", "baro_alt"}, "BARO")

    df = df.sort_values("time_s").dropna(subset=["time_s", "baro_alt"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 1:
        raise ValueError("BARO data is empty.")

    return df


# =====================================================================
# [3] Coordinate conversion
# =====================================================================

def latlng_to_local_xy(
    lat: float,
    lng: float,
    lat0: float,
    lng0: float,
) -> tuple[float, float]:
    """
    Convert latitude/longitude to local ENU planar coordinates.

    Returns:
        x = East [m]
        y = North [m]
    """
    lat0_rad = math.radians(lat0)

    dlat = math.radians(lat - lat0)
    dlng = math.radians(lng - lng0)

    y = dlat * EARTH_RADIUS_M
    x = dlng * EARTH_RADIUS_M * math.cos(lat0_rad)

    return x, y


def local_xy_to_latlng(
    x: float,
    y: float,
    lat0: float,
    lng0: float,
) -> tuple[float, float]:
    """
    Convert local ENU planar coordinates back to latitude/longitude.
    """
    lat0_rad = math.radians(lat0)

    dlat = y / EARTH_RADIUS_M
    dlng = x / (EARTH_RADIUS_M * math.cos(lat0_rad))

    lat = lat0 + math.degrees(dlat)
    lng = lng0 + math.degrees(dlng)

    return lat, lng


def add_gps_local_coordinates(gps_df: pd.DataFrame) -> pd.DataFrame:
    """
    Add gx/gy/gz local coordinates using the first GPS sample as the origin.
    """
    gps_local = gps_df.copy()

    lat0 = float(gps_local["lat"].iloc[0])
    lng0 = float(gps_local["lng"].iloc[0])
    alt0 = float(gps_local["alt"].iloc[0])

    xy = gps_local.apply(
        lambda row: latlng_to_local_xy(
            lat=float(row["lat"]),
            lng=float(row["lng"]),
            lat0=lat0,
            lng0=lng0,
        ),
        axis=1,
    )

    gps_local["gx"] = [point[0] for point in xy]
    gps_local["gy"] = [point[1] for point in xy]
    gps_local["gz"] = gps_local["alt"].astype(float) - alt0

    return gps_local


# =====================================================================
# [4] GPS/BARO trajectory modes
# =====================================================================

def build_gps_referenced_trajectory(
    gps_local_df: pd.DataFrame,
    z_values: np.ndarray,
    source_mode: str,
    alt_source: str,
) -> pd.DataFrame:
    """
    Build a trajectory DataFrame from GPS-referenced local coordinates.
    """
    time_s = gps_local_df["time_s"].to_numpy(dtype=float)
    x = gps_local_df["gx"].to_numpy(dtype=float)
    y = gps_local_df["gy"].to_numpy(dtype=float)
    z = np.asarray(z_values, dtype=float)

    if len(z) != len(time_s):
        raise ValueError(
            f"z length mismatch for {source_mode}: z={len(z)}, time_s={len(time_s)}"
        )

    vx, vy, vz = build_gradient_velocity(time_s=time_s, x=x, y=y, z=z)

    trajectory_df = pd.DataFrame(
        {
            "time_s": time_s,
            "x": x,
            "y": y,
            "z": z,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "lat": gps_local_df["lat"].astype(float).to_numpy(),
            "lng": gps_local_df["lng"].astype(float).to_numpy(),
        }
    )

    return apply_trajectory_metadata(
        trajectory_df=trajectory_df,
        source_mode=source_mode,
        alt_source=alt_source,
    )


def reconstruct_trajectory_gps_only(
    gps_df: pd.DataFrame,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Reconstruct the trajectory directly from GPS local coordinates.
    """
    gps_local_df = add_gps_local_coordinates(gps_df)

    trajectory_df = build_gps_referenced_trajectory(
        gps_local_df=gps_local_df,
        z_values=gps_local_df["gz"].to_numpy(dtype=float),
        source_mode="gps-only",
        alt_source="gps_relative_alt",
    )

    trajectory_df.attrs["mode"] = "gps-only"
    trajectory_df.attrs["altitude_source_description"] = "GPS relative altitude"
    trajectory_df.attrs["gps_correction_count"] = "not used"
    trajectory_df.attrs["baro_interp_range"] = "not used"

    return trajectory_df, gps_local_df


def reconstruct_trajectory_gps_baro(
    gps_df: pd.DataFrame,
    baro_df: pd.DataFrame,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Reconstruct the trajectory from GPS horizontal position and BARO altitude.
    """
    gps_local_df = add_gps_local_coordinates(gps_df)

    baro_time_s = baro_df["time_s"].to_numpy(dtype=float)
    validate_time_axis(baro_time_s, name="BARO interpolation")

    baro_rel_alt = (
        baro_df["baro_alt"].astype(float).to_numpy()
        - float(baro_df["baro_alt"].iloc[0])
    )
    gps_time_s = gps_local_df["time_s"].to_numpy(dtype=float)

    z_interp = np.interp(gps_time_s, baro_time_s, baro_rel_alt)

    trajectory_df = build_gps_referenced_trajectory(
        gps_local_df=gps_local_df,
        z_values=z_interp,
        source_mode="gps-baro",
        alt_source="baro_interp",
    )

    trajectory_df.attrs["mode"] = "gps-baro"
    trajectory_df.attrs["altitude_source_description"] = (
        "BARO relative altitude interpolated to GPS timestamps"
    )
    trajectory_df.attrs["gps_correction_count"] = "not used"
    trajectory_df.attrs["baro_interp_range"] = (
        f"{baro_time_s[0]:.6f} ~ {baro_time_s[-1]:.6f} s"
    )
    trajectory_df.attrs["baro_edge_hold_used"] = bool(
        gps_time_s[0] < baro_time_s[0] or gps_time_s[-1] > baro_time_s[-1]
    )

    return trajectory_df, gps_local_df


# =====================================================================
# [5] IMU interpolation / RK4 integration
# =====================================================================

def get_accel_at(t: float, imu_df: pd.DataFrame) -> np.ndarray:
    """
    Linearly interpolate IMU acceleration at time t.
    """
    ax = np.interp(t, imu_df["time_s"], imu_df["ax"])
    ay = np.interp(t, imu_df["time_s"], imu_df["ay"])
    az = np.interp(t, imu_df["time_s"], imu_df["az"])

    return np.array([ax, ay, az], dtype=float)


def rk4_step(
    state: np.ndarray,
    t: float,
    dt: float,
    imu_df: pd.DataFrame,
    gravity: np.ndarray,
) -> np.ndarray:
    """
    Advance the 6D state [x, y, z, vx, vy, vz] using RK4.
    """

    def derivative(s: np.ndarray, time: float) -> np.ndarray:
        velocity = s[3:6]
        acc_measured = get_accel_at(time, imu_df)
        acc_true = acc_measured - gravity

        return np.concatenate([velocity, acc_true])

    k1 = derivative(state, t)
    k2 = derivative(state + 0.5 * dt * k1, t + 0.5 * dt)
    k3 = derivative(state + 0.5 * dt * k2, t + 0.5 * dt)
    k4 = derivative(state + dt * k3, t + dt)

    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def correct_position_with_gps(
    state: np.ndarray,
    gps_xyz: np.ndarray,
    blend: float,
) -> np.ndarray:
    """
    Correct only the position terms of the state vector with GPS.
    """
    blend = clamp(blend, 0.0, 1.0)

    corrected = state.copy()
    corrected[0:3] = blend * gps_xyz + (1.0 - blend) * state[0:3]

    return corrected


def reconstruct_trajectory_imu_gps(
    imu_df: pd.DataFrame,
    gps_df: pd.DataFrame,
    dt: float = 0.02,
    gravity_z: float = 9.81,
    gps_blend: float = 1.0,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Keep the existing RK4 IMU integration with GPS position correction.
    """
    if dt <= 0:
        raise ValueError("dt must be greater than 0.")

    gps_blend = clamp(gps_blend, 0.0, 1.0)
    gps_local_df = add_gps_local_coordinates(gps_df)

    lat0 = float(gps_local_df["lat"].iloc[0])
    lng0 = float(gps_local_df["lng"].iloc[0])

    t_start = float(gps_local_df["time_s"].iloc[0])
    t_end = float(imu_df["time_s"].iloc[-1])

    if t_end <= t_start:
        raise ValueError(
            f"Invalid time range for imu-gps reconstruction: "
            f"GPS start={t_start:.6f}, IMU end={t_end:.6f}"
        )

    state = np.array(
        [
            float(gps_local_df["gx"].iloc[0]),
            float(gps_local_df["gy"].iloc[0]),
            float(gps_local_df["gz"].iloc[0]),
            0.0,
            0.0,
            0.0,
        ],
        dtype=float,
    )

    gravity = np.array([0.0, 0.0, gravity_z], dtype=float)

    times: list[float] = [t_start]
    states: list[np.ndarray] = [state.copy()]
    correction_count = 0

    t = t_start
    gps_idx = 1

    while t < t_end:
        step = min(dt, t_end - t)

        state = rk4_step(
            state=state,
            t=t,
            dt=step,
            imu_df=imu_df,
            gravity=gravity,
        )

        t += step

        while (
            gps_idx < len(gps_local_df)
            and float(gps_local_df["time_s"].iloc[gps_idx]) <= t
        ):
            gps_row = gps_local_df.iloc[gps_idx]
            gps_xyz = np.array(
                [
                    float(gps_row["gx"]),
                    float(gps_row["gy"]),
                    float(gps_row["gz"]),
                ],
                dtype=float,
            )

            state = correct_position_with_gps(
                state=state,
                gps_xyz=gps_xyz,
                blend=gps_blend,
            )

            correction_count += 1
            gps_idx += 1

        times.append(t)
        states.append(state.copy())

    states_arr = np.vstack(states)

    trajectory_df = pd.DataFrame(
        {
            "time_s": times,
            "x": states_arr[:, 0],
            "y": states_arr[:, 1],
            "z": states_arr[:, 2],
            "vx": states_arr[:, 3],
            "vy": states_arr[:, 4],
            "vz": states_arr[:, 5],
        }
    )

    latlng = trajectory_df.apply(
        lambda row: local_xy_to_latlng(
            x=float(row["x"]),
            y=float(row["y"]),
            lat0=lat0,
            lng0=lng0,
        ),
        axis=1,
    )

    trajectory_df["lat"] = [point[0] for point in latlng]
    trajectory_df["lng"] = [point[1] for point in latlng]

    trajectory_df = apply_trajectory_metadata(
        trajectory_df=trajectory_df,
        source_mode="imu-gps",
        alt_source="imu_rk4_gps_correction_experimental",
    )

    trajectory_df.attrs["mode"] = "imu-gps"
    trajectory_df.attrs["altitude_source_description"] = (
        "IMU RK4 integration with GPS correction, experimental"
    )
    trajectory_df.attrs["gps_correction_count"] = correction_count
    trajectory_df.attrs["baro_interp_range"] = "not used"
    trajectory_df.attrs["warning"] = EXPERIMENTAL_WARNING

    return trajectory_df, gps_local_df


# =====================================================================
# [6] Summary writer
# =====================================================================

def format_used_flag(mode: str, value: object, unit: str = "") -> str:
    """
    Format a parameter line with whether it was used by the selected mode.
    """
    suffix = f" {unit}" if unit else ""

    if mode == "imu-gps":
        return f"`{value}`{suffix} (used)"

    return f"`{value}`{suffix} (not used)"


def format_range(series: pd.Series) -> str:
    """
    Format min/max range text.
    """
    return f"{float(series.min()):.3f} ~ {float(series.max()):.3f}"


def write_reconstruct_summary(
    summary_path: Path,
    mode: str,
    imu_path: Path | None,
    gps_path: Path,
    baro_path: Path | None,
    out_path: Path,
    gps_local_path: Path,
    imu_df: pd.DataFrame | None,
    gps_df: pd.DataFrame,
    gps_local_df: pd.DataFrame,
    baro_df: pd.DataFrame | None,
    trajectory_df: pd.DataFrame,
    dt: float,
    gravity_z: float,
    gps_blend: float,
) -> None:
    """
    Write a Markdown summary of the reconstruction result.
    """
    altitude_source = trajectory_df.attrs.get("altitude_source_description", "N/A")
    correction_count = trajectory_df.attrs.get("gps_correction_count", "N/A")
    baro_interp_range = trajectory_df.attrs.get("baro_interp_range", "N/A")
    warnings: list[str] = []

    if mode == "imu-gps":
        warnings.append(str(trajectory_df.attrs.get("warning", EXPERIMENTAL_WARNING)))
        warnings.append("future work: attitude rotation required")
    else:
        warnings.append(
            "This result is intended as a conservative GPS/BARO-based sanity-check "
            "trajectory rather than a precision inertial solution."
        )

    if mode == "gps-baro" and trajectory_df.attrs.get("baro_edge_hold_used", False):
        warnings.append(
            "GPS timestamps extend outside the BARO time range. np.interp edge "
            "values were held constant at the ends."
        )

    lines: list[str] = [
        "# Trajectory Reconstruction Summary",
        "",
        "## Mode",
        "",
        f"- mode: `{mode}`",
        "",
        "## Input files",
        "",
        f"- IMU CSV: `{describe_path(imu_path)}`",
        f"- GPS CSV: `{gps_path}`",
        f"- BARO CSV: `{describe_path(baro_path)}`",
        "",
        "## Output files",
        "",
        f"- Trajectory CSV: `{out_path}`",
        f"- GPS local CSV: `{gps_local_path}`",
        f"- Summary Markdown: `{summary_path}`",
        "",
        "## Parameters",
        "",
        f"- dt: {format_used_flag(mode, dt, 's')}",
        f"- gravity_z: {format_used_flag(mode, gravity_z, 'm/s^2')}",
        f"- gps_blend: {format_used_flag(mode, gps_blend)}",
        "",
        "## Input ranges",
        "",
    ]

    if imu_df is not None:
        lines.extend(
            [
                f"- IMU rows: `{len(imu_df)}`",
                f"- IMU time: `{imu_df['time_s'].iloc[0]:.6f}` ~ "
                f"`{imu_df['time_s'].iloc[-1]:.6f}` s",
            ]
        )
    else:
        lines.extend(
            [
                "- IMU rows: `N/A`",
                "- IMU time: `N/A`",
            ]
        )

    lines.extend(
        [
            f"- GPS rows: `{len(gps_df)}`",
            f"- GPS time: `{gps_df['time_s'].iloc[0]:.6f}` ~ "
            f"`{gps_df['time_s'].iloc[-1]:.6f}` s",
        ]
    )

    if baro_df is not None:
        lines.extend(
            [
                f"- BARO rows: `{len(baro_df)}`",
                f"- BARO time: `{baro_df['time_s'].iloc[0]:.6f}` ~ "
                f"`{baro_df['time_s'].iloc[-1]:.6f}` s",
            ]
        )
    else:
        lines.extend(
            [
                "- BARO rows: `N/A`",
                "- BARO time: `N/A`",
            ]
        )

    lines.extend(
        [
            f"- BARO interpolation range: `{baro_interp_range}`",
            "",
            "## Reconstruction result",
            "",
            f"- Trajectory rows: `{len(trajectory_df)}`",
            f"- Trajectory time: `{trajectory_df['time_s'].iloc[0]:.6f}` ~ "
            f"`{trajectory_df['time_s'].iloc[-1]:.6f}` s",
            f"- Final local position x/y/z: "
            f"`{trajectory_df['x'].iloc[-1]:.3f}`, "
            f"`{trajectory_df['y'].iloc[-1]:.3f}`, "
            f"`{trajectory_df['z'].iloc[-1]:.3f}` m",
            f"- GPS local x range: `{format_range(gps_local_df['gx'])}` m",
            f"- GPS local y range: `{format_range(gps_local_df['gy'])}` m",
            f"- GPS local z range: `{format_range(gps_local_df['gz'])}` m",
            f"- Altitude source: `{altitude_source}`",
            f"- GPS correction count: `{correction_count}`",
            "",
            "## Columns",
            "",
            "### trajectory",
            "",
            "```text",
            ", ".join(trajectory_df.columns),
            "```",
            "",
            "### gps_local",
            "",
            "```text",
            ", ".join(gps_local_df.columns),
            "```",
            "",
            "## Limitation / Warning",
            "",
        ]
    )

    for warning in warnings:
        lines.append(f"- {warning}")

    lines.append("")

    summary_path.write_text("\n".join(lines), encoding="utf-8")


# =====================================================================
# [7] Main pipeline
# =====================================================================

def reconstruct_from_csv(
    mode: str,
    imu_path: Path | None,
    gps_path: Path,
    baro_path: Path | None,
    out_path: Path,
    dt: float,
    gravity_z: float,
    gps_blend: float,
    gps_local_out: Path | None,
    summary_out: Path | None,
) -> int:
    """
    Execute the full reconstruction pipeline.
    """
    if mode not in MODE_CHOICES:
        raise ValueError(f"Unsupported mode: {mode}")

    if mode == "imu-gps" and imu_path is None:
        raise ValueError("imu-gps mode requires --imu.")

    if mode == "gps-baro" and baro_path is None:
        raise ValueError("gps-baro mode requires --baro.")

    print(f"[MODE] Reconstruction mode: {mode}")

    if mode != "imu-gps":
        print(
            "[INFO] --dt, --gravity, and --gps-blend are accepted for CLI "
            f"compatibility but not used in mode={mode}."
        )

    print(f"[LOAD] GPS: {gps_path}")
    gps_df = load_prepared_gps_csv(gps_path)
    print(
        f"      -> {len(gps_df)} rows, "
        f"{gps_df['time_s'].iloc[0]:.3f}s ~ {gps_df['time_s'].iloc[-1]:.3f}s"
    )

    imu_df = None

    if imu_path is not None:
        print(f"[LOAD] IMU: {imu_path}")
        imu_df = load_prepared_imu_csv(imu_path)
        assert imu_df is not None
        print(
            f"      -> {len(imu_df)} rows, "
            f"{imu_df['time_s'].iloc[0]:.3f}s ~ {imu_df['time_s'].iloc[-1]:.3f}s"
        )
    else:
        print("[LOAD] IMU: skipped")

    baro_df = None

    if baro_path is not None:
        print(f"[LOAD] BARO: {baro_path}")
        baro_df = load_prepared_baro_csv(baro_path)
        assert baro_df is not None
        print(
            f"      -> {len(baro_df)} rows, "
            f"{baro_df['time_s'].iloc[0]:.3f}s ~ {baro_df['time_s'].iloc[-1]:.3f}s"
        )
    else:
        print("[LOAD] BARO: skipped")

    print("[RUN] Reconstructing trajectory...")

    if mode == "gps-only":
        trajectory_df, gps_local_df = reconstruct_trajectory_gps_only(gps_df=gps_df)
    elif mode == "gps-baro":
        if baro_df is None:
            raise ValueError("gps-baro mode requires baro.csv.")

        trajectory_df, gps_local_df = reconstruct_trajectory_gps_baro(
            gps_df=gps_df,
            baro_df=baro_df,
        )
    else:
        if imu_df is None:
            raise ValueError("imu-gps mode requires imu.csv.")

        print(f"[WARNING] {EXPERIMENTAL_WARNING}")

        trajectory_df, gps_local_df = reconstruct_trajectory_imu_gps(
            imu_df=imu_df,
            gps_df=gps_df,
            dt=dt,
            gravity_z=gravity_z,
            gps_blend=gps_blend,
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    trajectory_df.to_csv(out_path, index=False)

    if gps_local_out is None:
        gps_local_out = out_path.with_name(f"{out_path.stem}_gps_local.csv")

    gps_local_out.parent.mkdir(parents=True, exist_ok=True)
    gps_local_df.to_csv(gps_local_out, index=False)

    if summary_out is None:
        summary_out = out_path.with_name(f"{out_path.stem}_summary.md")

    write_reconstruct_summary(
        summary_path=summary_out,
        mode=mode,
        imu_path=imu_path,
        gps_path=gps_path,
        baro_path=baro_path,
        out_path=out_path,
        gps_local_path=gps_local_out,
        imu_df=imu_df,
        gps_df=gps_df,
        gps_local_df=gps_local_df,
        baro_df=baro_df,
        trajectory_df=trajectory_df,
        dt=dt,
        gravity_z=gravity_z,
        gps_blend=gps_blend,
    )

    print("[SAVE] Outputs written")
    print(f"      -> trajectory: {out_path}")
    print(f"      -> gps_local : {gps_local_out}")
    print(f"      -> summary   : {summary_out}")
    print("[DONE] Trajectory reconstruction completed.")

    return 0


# =====================================================================
# [8] CLI
# =====================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reconstruct trajectory from prepared IMU/GPS/BARO CSV files."
    )

    parser.add_argument(
        "--mode",
        choices=MODE_CHOICES,
        default="gps-baro",
        help=(
            "Reconstruction mode. "
            "gps-only = GPS local x/y/z, "
            "gps-baro = GPS local x/y + BARO relative z, "
            "imu-gps = RK4 IMU integration + GPS correction. "
            "Default: gps-baro."
        ),
    )

    parser.add_argument(
        "--imu",
        type=Path,
        default=None,
        help=(
            "Path to prepared imu.csv from 02_prepare_trajectory_csv.py. "
            "Required for mode=imu-gps. Accepted but not used by gps-only/gps-baro."
        ),
    )

    parser.add_argument(
        "--gps",
        type=Path,
        required=True,
        help="Path to prepared gps.csv from 02_prepare_trajectory_csv.py.",
    )

    parser.add_argument(
        "--baro",
        type=Path,
        default=None,
        help=(
            "Path to prepared baro.csv from 02_prepare_trajectory_csv.py. "
            "Required for mode=gps-baro."
        ),
    )

    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output trajectory CSV path.",
    )

    parser.add_argument(
        "--gps-local-out",
        type=Path,
        default=None,
        help=(
            "Optional output path for GPS local coordinate CSV. "
            "Default: <out_stem>_gps_local.csv"
        ),
    )

    parser.add_argument(
        "--summary-out",
        type=Path,
        default=None,
        help=(
            "Optional output path for reconstruction summary Markdown. "
            "Default: <out_stem>_summary.md"
        ),
    )

    parser.add_argument(
        "--dt",
        type=float,
        default=0.02,
        help="RK4 integration step in seconds. Used only by mode=imu-gps.",
    )

    parser.add_argument(
        "--gravity",
        type=float,
        default=9.81,
        help="Gravity acceleration magnitude in m/s^2. Used only by mode=imu-gps.",
    )

    parser.add_argument(
        "--gps-blend",
        type=float,
        default=1.0,
        help=(
            "GPS position correction blend. "
            "1.0 = full snap to GPS. Used only by mode=imu-gps."
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        return reconstruct_from_csv(
            mode=args.mode,
            imu_path=args.imu,
            gps_path=args.gps,
            baro_path=args.baro,
            out_path=args.out,
            dt=args.dt,
            gravity_z=args.gravity,
            gps_blend=args.gps_blend,
            gps_local_out=args.gps_local_out,
            summary_out=args.summary_out,
        )
    except Exception as exc:
        print(f"[ERROR] Failed to reconstruct trajectory: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
