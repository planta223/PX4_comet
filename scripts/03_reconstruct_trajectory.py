#!/usr/bin/env python3
"""
03_reconstruct_trajectory.py

ROK2 궤적 분석 파이프라인 3단계:
02_prepare_trajectory_csv.py에서 생성한 표준 CSV를 입력받아
IMU + GPS 기반 궤적을 재구성하는 스크립트입니다.

입력:
    output/trajectory_inputs_<log_id>/imu.csv
    output/trajectory_inputs_<log_id>/gps.csv
    output/trajectory_inputs_<log_id>/baro.csv   # 선택. 현재 계산에는 직접 사용하지 않음

출력:
    output/trajectory_<log_id>.csv
    output/trajectory_<log_id>_gps_local.csv
    output/trajectory_<log_id>_summary.md

역할:
    - GPS 위도/경도를 로컬 ENU 좌표로 변환
    - IMU 가속도를 시간 보간
    - RK4 적분으로 위치/속도 예측
    - GPS 관측 시점마다 위치 보정
    - 재구성된 궤적 CSV 저장

주의:
    - 이 스크립트는 플롯을 생성하지 않습니다.
    - 플롯은 04_plot_trajectory.py에서 수행합니다.
    - 현재 IMU 가속도 ax/ay/az는 이미 earth-frame 계열이라고 가정합니다.
    - ArduPilot IMU AccX/AccY/AccZ가 body-frame이면, ATT/XKQ 기반 회전 변환을
      먼저 적용해야 정밀 궤적이 됩니다.
    - 따라서 현재 결과는 정밀 항법 결과가 아니라 파이프라인 검증 및
      sanity check용 궤적입니다.
"""

# =====================================================================
# 실행 방법
# ---------------------------------------------------------------------
# 기본 실행:
#
#   python scripts/03_reconstruct_trajectory.py \
#     --imu output/trajectory_inputs_00000197/imu.csv \
#     --gps output/trajectory_inputs_00000197/gps.csv \
#     --baro output/trajectory_inputs_00000197/baro.csv \
#     --out output/trajectory_00000197.csv
#
# BARO 없이 실행:
#
#   python scripts/03_reconstruct_trajectory.py \
#     --imu output/trajectory_inputs_00000197/imu.csv \
#     --gps output/trajectory_inputs_00000197/gps.csv \
#     --out output/trajectory_00000197.csv
#
# GPS 보정 강도 변경:
#
#   python scripts/03_reconstruct_trajectory.py \
#     --imu output/trajectory_inputs_00000197/imu.csv \
#     --gps output/trajectory_inputs_00000197/gps.csv \
#     --baro output/trajectory_inputs_00000197/baro.csv \
#     --gps-blend 0.8 \
#     --out output/trajectory_00000197.csv
#
# 출력:
#
#   output/trajectory_00000197.csv
#   output/trajectory_00000197_gps_local.csv
#   output/trajectory_00000197_summary.md
#
# 후속 실행:
#
#   python scripts/04_plot_trajectory.py \
#     --trajectory output/trajectory_00000197.csv \
#     --gps-local output/trajectory_00000197_gps_local.csv \
#     --baro output/trajectory_inputs_00000197/baro.csv
# =====================================================================

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd


EARTH_RADIUS_M = 6378137.0


# =====================================================================
# [1] 공통 유틸
# =====================================================================

def require_columns(df: pd.DataFrame, required: set[str], name: str) -> None:
    """
    DataFrame에 필요한 컬럼이 있는지 검사한다.
    """
    missing = required - set(df.columns)

    if missing:
        raise ValueError(
            f"{name} CSV에 필요한 컬럼이 없습니다: {sorted(missing)}\n"
            f"실제 컬럼: {list(df.columns)}"
        )


def validate_file(path: Path, name: str) -> None:
    """
    입력 파일 존재 여부를 검사한다.
    """
    if not path.exists():
        raise FileNotFoundError(f"{name} 파일을 찾을 수 없습니다: {path}")

    if not path.is_file():
        raise ValueError(f"{name} 경로가 파일이 아닙니다: {path}")


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    값을 지정 범위 안으로 제한한다.
    """
    return max(min_value, min(max_value, value))


# =====================================================================
# [2] CSV 로딩
# =====================================================================

def load_prepared_imu_csv(path: Path) -> pd.DataFrame:
    """
    02_prepare_trajectory_csv.py가 생성한 imu.csv를 읽는다.

    필요 컬럼:
        time_s, ax, ay, az

    선택 컬럼:
        gx, gy, gz
    """
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
        raise ValueError("IMU 데이터가 너무 적습니다. 최소 2개 이상 필요합니다.")

    return df


def load_prepared_gps_csv(path: Path) -> pd.DataFrame:
    """
    02_prepare_trajectory_csv.py가 생성한 gps.csv를 읽는다.

    필요 컬럼:
        time_s, lat, lng, alt

    선택 컬럼:
        spd, vz, nsats, hdop, status
    """
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

    if len(df) < 1:
        raise ValueError("GPS 데이터가 비어 있습니다.")

    return df


def load_prepared_baro_csv(path: Path | None) -> pd.DataFrame | None:
    """
    02_prepare_trajectory_csv.py가 생성한 baro.csv를 읽는다.

    현재 03번 계산에는 직접 사용하지 않고, summary에 시간 범위만 기록한다.
    실제 고도 비교 플롯은 04_plot_trajectory.py에서 수행한다.
    """
    if path is None:
        return None

    df = pd.read_csv(path)

    require_columns(df, {"time_s", "baro_alt"}, "BARO")

    df = df.sort_values("time_s").dropna(subset=["time_s", "baro_alt"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 1:
        raise ValueError("BARO 데이터가 비어 있습니다.")

    return df


# =====================================================================
# [3] 좌표 변환
# =====================================================================

def latlng_to_local_xy(
    lat: float,
    lng: float,
    lat0: float,
    lng0: float,
) -> tuple[float, float]:
    """
    위도/경도를 기준점 기준 로컬 ENU 평면좌표로 변환한다.

    반환:
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
    로컬 ENU 평면좌표를 다시 위도/경도로 변환한다.
    """
    lat0_rad = math.radians(lat0)

    dlat = y / EARTH_RADIUS_M
    dlng = x / (EARTH_RADIUS_M * math.cos(lat0_rad))

    lat = lat0 + math.degrees(dlat)
    lng = lng0 + math.degrees(dlng)

    return lat, lng


def add_gps_local_coordinates(gps_df: pd.DataFrame) -> pd.DataFrame:
    """
    GPS 위경도/고도를 첫 GPS 샘플 기준 로컬 ENU 좌표로 변환한다.

    추가 컬럼:
        gx = East [m]
        gy = North [m]
        gz = Up [m], 첫 GPS 고도 기준 상대고도
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
# [4] IMU 보간 / RK4 적분
# =====================================================================

def get_accel_at(t: float, imu_df: pd.DataFrame) -> np.ndarray:
    """
    임의 시각 t에서 IMU 가속도를 선형보간으로 얻는다.

    입력:
        imu_df columns = time_s, ax, ay, az

    반환:
        np.array([ax, ay, az])
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
    RK4로 상태를 한 스텝 적분한다.

    state:
        [x, y, z, vx, vy, vz]

    모델:
        d(position) / dt = velocity
        d(velocity) / dt = measured_acceleration - gravity

    주의:
        measured_acceleration이 earth-frame 기준이라는 단순 가정이다.
        body-frame raw IMU를 그대로 넣으면 결과가 물리적으로 틀어질 수 있다.
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
    GPS 관측 위치로 상태벡터의 위치 성분만 보정한다.

    blend:
        1.0 = GPS 위치로 완전 스냅
        0.0 = GPS 보정 사용 안 함
        0.0~1.0 = 기존 추정치와 GPS 관측치 가중평균
    """
    blend = clamp(blend, 0.0, 1.0)

    corrected = state.copy()
    corrected[0:3] = blend * gps_xyz + (1.0 - blend) * state[0:3]

    return corrected


# =====================================================================
# [5] 궤적 재구성
# =====================================================================

def reconstruct_trajectory(
    imu_df: pd.DataFrame,
    gps_df: pd.DataFrame,
    dt: float = 0.02,
    gravity_z: float = 9.81,
    gps_blend: float = 1.0,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    IMU 예측 + GPS 위치 보정 방식으로 궤적을 재구성한다.

    반환:
        trajectory_df:
            time_s, x, y, z, vx, vy, vz, lat, lng

        gps_local_df:
            gps.csv에 gx, gy, gz를 추가한 DataFrame
    """
    if dt <= 0:
        raise ValueError("dt는 0보다 커야 합니다.")

    gps_blend = clamp(gps_blend, 0.0, 1.0)

    gps_local = add_gps_local_coordinates(gps_df)

    lat0 = float(gps_local["lat"].iloc[0])
    lng0 = float(gps_local["lng"].iloc[0])

    t_start = float(gps_local["time_s"].iloc[0])
    t_end = float(imu_df["time_s"].iloc[-1])

    if t_end <= t_start:
        raise ValueError(
            f"시간 범위가 이상합니다. "
            f"GPS start={t_start:.6f}, IMU end={t_end:.6f}"
        )

    state = np.array(
        [
            float(gps_local["gx"].iloc[0]),
            float(gps_local["gy"].iloc[0]),
            float(gps_local["gz"].iloc[0]),
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

        while gps_idx < len(gps_local) and float(gps_local["time_s"].iloc[gps_idx]) <= t:
            gps_row = gps_local.iloc[gps_idx]
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

    trajectory_df.attrs["gps_correction_count"] = correction_count

    return trajectory_df, gps_local


# =====================================================================
# [6] 요약 파일 작성
# =====================================================================

def write_reconstruct_summary(
    summary_path: Path,
    imu_path: Path,
    gps_path: Path,
    baro_path: Path | None,
    out_path: Path,
    gps_local_path: Path,
    imu_df: pd.DataFrame,
    gps_df: pd.DataFrame,
    gps_local_df: pd.DataFrame,
    baro_df: pd.DataFrame | None,
    trajectory_df: pd.DataFrame,
    dt: float,
    gravity_z: float,
    gps_blend: float,
) -> None:
    """
    궤적 재구성 결과 요약 Markdown 파일을 작성한다.
    """
    correction_count = trajectory_df.attrs.get("gps_correction_count", "N/A")

    lines: list[str] = [
        "# Trajectory Reconstruction Summary",
        "",
        "## Inputs",
        "",
        f"- IMU CSV: `{imu_path}`",
        f"- GPS CSV: `{gps_path}`",
        f"- BARO CSV: `{baro_path if baro_path is not None else 'N/A'}`",
        "",
        "## Outputs",
        "",
        f"- Trajectory CSV: `{out_path}`",
        f"- GPS local CSV: `{gps_local_path}`",
        "",
        "## Parameters",
        "",
        f"- dt: `{dt}` s",
        f"- gravity_z: `{gravity_z}` m/s^2",
        f"- gps_blend: `{gps_blend}`",
        "",
        "## Input ranges",
        "",
        f"- IMU rows: `{len(imu_df)}`",
        f"- IMU time: `{imu_df['time_s'].iloc[0]:.6f}` ~ "
        f"`{imu_df['time_s'].iloc[-1]:.6f}` s",
        f"- GPS rows: `{len(gps_df)}`",
        f"- GPS time: `{gps_df['time_s'].iloc[0]:.6f}` ~ "
        f"`{gps_df['time_s'].iloc[-1]:.6f}` s",
    ]

    if baro_df is not None:
        lines.extend(
            [
                f"- BARO rows: `{len(baro_df)}`",
                f"- BARO time: `{baro_df['time_s'].iloc[0]:.6f}` ~ "
                f"`{baro_df['time_s'].iloc[-1]:.6f}` s",
            ]
        )
    else:
        lines.append("- BARO rows: `N/A`")

    lines.extend(
        [
            "",
            "## Reconstruction result",
            "",
            f"- Trajectory rows: `{len(trajectory_df)}`",
            f"- Trajectory time: `{trajectory_df['time_s'].iloc[0]:.6f}` ~ "
            f"`{trajectory_df['time_s'].iloc[-1]:.6f}` s",
            f"- GPS correction count: `{correction_count}`",
            f"- Start lat/lng: "
            f"`{trajectory_df['lat'].iloc[0]:.8f}`, "
            f"`{trajectory_df['lng'].iloc[0]:.8f}`",
            f"- End lat/lng: "
            f"`{trajectory_df['lat'].iloc[-1]:.8f}`, "
            f"`{trajectory_df['lng'].iloc[-1]:.8f}`",
            f"- Final local position x/y/z: "
            f"`{trajectory_df['x'].iloc[-1]:.3f}`, "
            f"`{trajectory_df['y'].iloc[-1]:.3f}`, "
            f"`{trajectory_df['z'].iloc[-1]:.3f}` m",
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
            "## Important limitation",
            "",
            "- This reconstruction assumes IMU acceleration is already expressed in an "
            "earth-fixed frame. If AccX/AccY/AccZ are body-frame raw IMU values, "
            "attitude-based rotation using ATT/XKQ is required before precision use.",
            "",
        ]
    )

    summary_path.write_text("\n".join(lines), encoding="utf-8")


# =====================================================================
# [7] 메인 처리
# =====================================================================

def reconstruct_from_csv(
    imu_path: Path,
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
    전체 궤적 재구성 파이프라인 실행.
    """
    validate_file(imu_path, "IMU")
    validate_file(gps_path, "GPS")

    if baro_path is not None:
        validate_file(baro_path, "BARO")

    print(f"[1/5] Loading IMU: {imu_path}")
    imu_df = load_prepared_imu_csv(imu_path)
    print(
        f"      -> {len(imu_df)} rows, "
        f"{imu_df['time_s'].iloc[0]:.3f}s ~ {imu_df['time_s'].iloc[-1]:.3f}s"
    )

    print(f"[2/5] Loading GPS: {gps_path}")
    gps_df = load_prepared_gps_csv(gps_path)
    print(
        f"      -> {len(gps_df)} rows, "
        f"{gps_df['time_s'].iloc[0]:.3f}s ~ {gps_df['time_s'].iloc[-1]:.3f}s"
    )

    baro_df = None

    if baro_path is not None:
        print(f"[3/5] Loading BARO: {baro_path}")
        baro_df = load_prepared_baro_csv(baro_path)
        assert baro_df is not None
        print(
            f"      -> {len(baro_df)} rows, "
            f"{baro_df['time_s'].iloc[0]:.3f}s ~ {baro_df['time_s'].iloc[-1]:.3f}s"
        )
    else:
        print("[3/5] Loading BARO: skipped")

    print(f"[4/5] Reconstructing trajectory: dt={dt}, gps_blend={gps_blend}")
    trajectory_df, gps_local_df = reconstruct_trajectory(
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

    print("[5/5] Saving outputs")
    print(f"      -> trajectory: {out_path}")
    print(f"      -> gps_local : {gps_local_out}")
    print(f"      -> summary   : {summary_out}")

    print("[DONE] Trajectory reconstruction completed.")

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reconstruct ROK2 trajectory from prepared IMU/GPS/BARO CSV files."
    )

    parser.add_argument(
        "--imu",
        type=Path,
        required=True,
        help="Path to prepared imu.csv from 02_prepare_trajectory_csv.py.",
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
        help="Optional path to prepared baro.csv from 02_prepare_trajectory_csv.py.",
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
        help="RK4 integration step in seconds. Default: 0.02.",
    )

    parser.add_argument(
        "--gravity",
        type=float,
        default=9.81,
        help="Gravity acceleration magnitude in m/s^2. Default: 9.81.",
    )

    parser.add_argument(
        "--gps-blend",
        type=float,
        default=1.0,
        help="GPS position correction blend. 1.0 = full snap to GPS. Default: 1.0.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        return reconstruct_from_csv(
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