"""State construction and coordinate/orientation transforms for EKF replay."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


NED_TO_ENU = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)

# Change this fixed rotation if a displayed model's forward/up axes differ from
# ArduPilot body X-forward, Y-right, Z-down. Axis-only rendering uses identity.
MODEL_AXIS_OFFSET = np.eye(3, dtype=float)

FINAL_COLUMNS = [
    "time_s",
    "pn_m",
    "pe_m",
    "pd_m",
    "vn_mps",
    "ve_mps",
    "vd_mps",
    "x_east_m",
    "y_north_m",
    "z_up_m",
    "vx_east_mps",
    "vy_north_mps",
    "vz_up_mps",
    "qw",
    "qx",
    "qy",
    "qz",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "vel_test_ratio",
    "pos_test_ratio",
    "height_test_ratio",
    "mag_test_ratio",
    "fault_status",
    "timeout_status",
    "solution_status",
]


@dataclass
class StateBuildResult:
    states: pd.DataFrame
    warnings: List[str]
    quaternion_source: str
    quaternion_valid_ratio: float
    duplicate_xkf1_timestamps: int
    nonmonotonic_xkf1_timestamps: int


def ned_to_enu_position_velocity(
    pn: np.ndarray,
    pe: np.ndarray,
    pd_down: np.ndarray,
    vn: np.ndarray,
    ve: np.ndarray,
    vd_down: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert NED position/velocity to x-East, y-North, z-Up arrays."""
    position_enu = np.column_stack((pe, pn, -pd_down))
    velocity_enu = np.column_stack((ve, vn, -vd_down))
    return position_enu, velocity_enu


def quaternion_body_axes_enu(
    quaternion_wxyz: Sequence[float],
    use_conjugate: bool = False,
    model_axis_offset: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Return a 3x3 matrix whose columns are body axes in display ENU.

    XKQ is stored as scalar-first w,x,y,z. SciPy accepts x,y,z,w. The normal
    path interprets the quaternion as body-to-NED. The conjugate switch permits
    visual verification on firmware/log variants with the opposite convention.
    """
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        return NED_TO_ENU @ (MODEL_AXIS_OFFSET if model_axis_offset is None else model_axis_offset)
    q /= np.linalg.norm(q)
    rotation = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    if use_conjugate:
        rotation = rotation.inv()
    offset = MODEL_AXIS_OFFSET if model_axis_offset is None else np.asarray(model_axis_offset)
    return NED_TO_ENU @ rotation.as_matrix() @ offset


def _field_lookup(columns: Iterable[str], candidates: Sequence[str]) -> Optional[str]:
    by_lower = {str(column).lower(): str(column) for column in columns}
    for candidate in candidates:
        if candidate.lower() in by_lower:
            return by_lower[candidate.lower()]
    return None


def _numeric_column(
    frame: pd.DataFrame,
    candidates: Sequence[str],
    default: float = np.nan,
) -> pd.Series:
    name = _field_lookup(frame.columns, candidates)
    if name is None:
        return pd.Series(default, index=frame.index, dtype=float)
    return pd.to_numeric(frame[name], errors="coerce")


def _prepare_timed_frame(frame: pd.DataFrame, label: str, warnings: List[str]) -> pd.DataFrame:
    if frame.empty:
        return frame.copy()
    time_name = _field_lookup(frame.columns, ["TimeUS", "time_us"])
    if time_name is None:
        warnings.append(f"{label}: TimeUS 필드가 없어 시간 매칭에서 제외했습니다. 실제 필드: {list(frame.columns)}")
        return pd.DataFrame()
    prepared = frame.copy()
    prepared["_time_us"] = pd.to_numeric(prepared[time_name], errors="coerce")
    prepared = prepared.dropna(subset=["_time_us"])
    prepared = prepared.sort_values("_time_us", kind="stable")
    duplicate_count = int(prepared.duplicated("_time_us", keep="last").sum())
    if duplicate_count:
        warnings.append(f"{label}: 중복 timestamp {duplicate_count}개는 마지막 샘플을 사용했습니다.")
        prepared = prepared.drop_duplicates("_time_us", keep="last")
    return prepared.reset_index(drop=True)


def _euler_quaternions(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    angles = np.column_stack((roll, pitch, yaw))
    result = np.full((len(angles), 4), np.nan, dtype=float)
    valid = np.all(np.isfinite(angles), axis=1)
    if np.any(valid):
        xyzw = Rotation.from_euler("xyz", angles[valid], degrees=True).as_quat()
        result[valid] = xyzw[:, [3, 0, 1, 2]]
    return result


def _normalize_and_hold_quaternions(
    raw_wxyz: np.ndarray,
    fallback_wxyz: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    output = np.full_like(raw_wxyz, np.nan, dtype=float)
    valid_raw = np.zeros(len(raw_wxyz), dtype=bool)
    previous: Optional[np.ndarray] = None
    for index, raw in enumerate(raw_wxyz):
        norm = np.linalg.norm(raw) if np.all(np.isfinite(raw)) else np.nan
        if np.isfinite(norm) and norm >= 1e-8:
            current = raw / norm
            valid_raw[index] = True
            previous = current
        elif previous is not None:
            current = previous
        else:
            fallback = fallback_wxyz[index]
            fallback_norm = np.linalg.norm(fallback) if np.all(np.isfinite(fallback)) else np.nan
            current = fallback / fallback_norm if np.isfinite(fallback_norm) and fallback_norm >= 1e-8 else np.array([1.0, 0.0, 0.0, 0.0])
            previous = current
        output[index] = current
    return output, valid_raw


def _rename_for_merge(frame: pd.DataFrame, mapping: Dict[str, Sequence[str]], prefix: str) -> pd.DataFrame:
    selected = pd.DataFrame({"_time_us": frame["_time_us"]})
    for output_name, candidates in mapping.items():
        selected[prefix + output_name] = _numeric_column(frame, candidates)
    return selected


def build_states(
    xkf1: pd.DataFrame,
    xkq: pd.DataFrame,
    xkf4: pd.DataFrame,
    match_tolerance_s: float = 0.05,
) -> StateBuildResult:
    """Build the XKF1-based, nearest-time-aligned replay state table."""
    warnings: List[str] = []
    if xkf1.empty:
        raise ValueError("XKF1 메시지가 없습니다.")
    time_name = _field_lookup(xkf1.columns, ["TimeUS", "time_us"])
    if time_name is None:
        raise ValueError(f"XKF1에 TimeUS가 없습니다. 실제 필드: {list(xkf1.columns)}")

    original_time = pd.to_numeric(xkf1[time_name], errors="coerce")
    valid_original_time = original_time.dropna()
    if valid_original_time.empty:
        raise ValueError("XKF1.TimeUS에 유효한 숫자 timestamp가 없습니다.")
    first_valid_time_us = float(valid_original_time.iloc[0])
    time_values = valid_original_time.to_numpy()
    nonmonotonic = int(np.sum(np.diff(time_values) < 0))

    base = xkf1.copy()
    base["_time_us"] = original_time
    base = base.dropna(subset=["_time_us"]).sort_values("_time_us", kind="stable")
    duplicate_count = int(base.duplicated("_time_us", keep="last").sum())
    if nonmonotonic:
        warnings.append(f"XKF1: 비단조 timestamp 구간 {nonmonotonic}개를 시간순으로 정렬했습니다.")
    if duplicate_count:
        warnings.append(f"XKF1: 중복 timestamp {duplicate_count}개는 마지막 샘플을 사용했습니다.")
        base = base.drop_duplicates("_time_us", keep="last")
    base = base.reset_index(drop=True)

    required = {
        "Roll": ["Roll"], "Pitch": ["Pitch"], "Yaw": ["Yaw"],
        "VN": ["VN"], "VE": ["VE"], "VD": ["VD"],
        "PN": ["PN"], "PE": ["PE"], "PD": ["PD"],
    }
    missing_required = [name for name, aliases in required.items() if _field_lookup(base.columns, aliases) is None]
    if missing_required:
        warnings.append(f"XKF1 예상 필드 누락 {missing_required}. 실제 필드: {list(xkf1.columns)}")

    states = pd.DataFrame({"_time_us": base["_time_us"]})
    states["time_s"] = (states["_time_us"] - first_valid_time_us) / 1_000_000.0
    states["roll_deg"] = _numeric_column(base, ["Roll"])
    states["pitch_deg"] = _numeric_column(base, ["Pitch"])
    states["yaw_deg"] = _numeric_column(base, ["Yaw"])
    states["vn_mps"] = _numeric_column(base, ["VN"])
    states["ve_mps"] = _numeric_column(base, ["VE"])
    states["vd_mps"] = _numeric_column(base, ["VD"])
    states["pn_m"] = _numeric_column(base, ["PN"])
    states["pe_m"] = _numeric_column(base, ["PE"])
    states["pd_m"] = _numeric_column(base, ["PD"])
    for optional in ("GX", "GY", "GZ", "OH", "dPD"):
        states[optional.lower()] = _numeric_column(base, [optional])

    # DataFlash TimeUS and merge keys are int64; pandas requires an integer
    # tolerance for an integer merge_asof key.
    tolerance_us = int(round(max(0.0, float(match_tolerance_s)) * 1_000_000.0))
    prepared_q = _prepare_timed_frame(xkq, "XKQ", warnings)
    q_mapping = {"qw_raw": ["Q1"], "qx_raw": ["Q2"], "qy_raw": ["Q3"], "qz_raw": ["Q4"]}
    if not prepared_q.empty:
        missing_q = [key for key, aliases in q_mapping.items() if _field_lookup(prepared_q.columns, aliases) is None]
        if missing_q:
            warnings.append(f"XKQ 예상 quaternion 필드 누락 {missing_q}. 실제 필드: {list(xkq.columns)}")
        q_selected = _rename_for_merge(prepared_q, q_mapping, "")
        states = pd.merge_asof(
            states.sort_values("_time_us"), q_selected.sort_values("_time_us"),
            on="_time_us", direction="nearest", tolerance=tolerance_us,
        )
        quaternion_source = "XKQ nearest; invalid/unmatched samples hold previous (initial samples use Euler fallback)"
    else:
        for name in q_mapping:
            states[name] = np.nan
        quaternion_source = "XKF1 Euler fallback (XKQ unavailable)"

    health_mapping = {
        "vel_test_ratio": ["SV"],
        "pos_test_ratio": ["SP"],
        "height_test_ratio": ["SH"],
        "mag_test_ratio": ["SM"],
        "fault_status": ["FS"],
        "timeout_status": ["TS"],
        "solution_status": ["SS"],
        # Preserve additional fields observed in a concrete FMT definition
        # without assigning firmware-dependent semantics to them here.
        "xkf4_gps": ["GPS"],
        "xkf4_primary_index": ["PI"],
        "xkf4_of_north": ["OFN"],
        "xkf4_of_east": ["OFE"],
        "xkf4_svt": ["SVT"],
        "xkf4_err_rp": ["errRP"],
    }
    prepared_h = _prepare_timed_frame(xkf4, "XKF4", warnings)
    if not prepared_h.empty:
        missing_health = [key for key, aliases in health_mapping.items() if _field_lookup(prepared_h.columns, aliases) is None]
        if missing_health:
            warnings.append(f"XKF4 예상 필드 누락 {missing_health}. 실제 필드: {list(xkf4.columns)}")
        health_selected = _rename_for_merge(prepared_h, health_mapping, "")
        states = pd.merge_asof(
            states.sort_values("_time_us"), health_selected.sort_values("_time_us"),
            on="_time_us", direction="nearest", tolerance=tolerance_us,
        )
    else:
        for name in health_mapping:
            states[name] = np.nan

    fallback = _euler_quaternions(
        states["roll_deg"].to_numpy(float),
        states["pitch_deg"].to_numpy(float),
        states["yaw_deg"].to_numpy(float),
    )
    raw_q = states[["qw_raw", "qx_raw", "qy_raw", "qz_raw"]].to_numpy(float)
    normalized_q, valid_q = _normalize_and_hold_quaternions(raw_q, fallback)
    states[["qw", "qx", "qy", "qz"]] = normalized_q
    states["quaternion_matched_valid"] = valid_q

    position_enu, velocity_enu = ned_to_enu_position_velocity(
        states["pn_m"].to_numpy(float), states["pe_m"].to_numpy(float), states["pd_m"].to_numpy(float),
        states["vn_mps"].to_numpy(float), states["ve_mps"].to_numpy(float), states["vd_mps"].to_numpy(float),
    )
    states[["x_east_m", "y_north_m", "z_up_m"]] = position_enu
    states[["vx_east_mps", "vy_north_mps", "vz_up_mps"]] = velocity_enu

    for column in FINAL_COLUMNS:
        if column not in states:
            states[column] = np.nan
    extras = [column for column in states.columns if column not in FINAL_COLUMNS and not column.endswith("_raw") and column != "_time_us"]
    states = states[FINAL_COLUMNS + extras].reset_index(drop=True)
    ratio = float(valid_q.mean()) if len(valid_q) else 0.0
    return StateBuildResult(states, warnings, quaternion_source, ratio, duplicate_count, nonmonotonic)
