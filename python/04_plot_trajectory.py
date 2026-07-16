#!/usr/bin/env python3
"""
04_plot_trajectory.py

ROK2 궤적 분석 파이프라인 4단계:
03_reconstruct_trajectory.py에서 생성한 trajectory.csv와 gps_local.csv를 읽어
궤적 및 센서 비교 플롯을 생성하는 스크립트입니다.

입력:
    output/03_trajectory_<log_id>/trajectory.csv
    output/03_trajectory_<log_id>/gps_local.csv
    output/02_trajectory_inputs_<log_id>/baro.csv   # 선택
    output/02_trajectory_inputs_<log_id>/imu.csv    # 선택

출력:
    화면 표시 또는 PNG 파일 저장

역할:
    - 3D 궤적 플롯
    - XY 평면 궤적 플롯
    - 고도 profile 플롯
    - GPS altitude drift 플롯
    - 속도 profile 플롯
    - GPS 품질 플롯. nsats/hdop/status 컬럼이 있을 때만
    - IMU 가속도 norm 플롯. --imu 입력이 있을 때만

중요:
    - 이 스크립트는 궤적을 다시 계산하지 않습니다.
    - 03_reconstruct_trajectory.py가 만든 trajectory.csv를 그대로 시각화합니다.
    - gps-baro 모드에서는 trajectory z가 BARO 기반이고, gps_local.gz는 GPS altitude drift입니다.
      따라서 3D plot에서 GPS observation의 z는 trajectory z에 투영해서 표시합니다.
    - GPS altitude drift는 별도 plot으로 분리합니다.
"""

# =====================================================================
# 실행 방법
# ---------------------------------------------------------------------
# 기본 실행. 화면에 플롯 표시:
#
#   python scripts/04_plot_trajectory.py \
#     --trajectory output/03_trajectory_00000197/trajectory.csv \
#     --gps-local output/03_trajectory_00000197/gps_local.csv \
#     --baro output/02_trajectory_inputs_00000197/baro.csv
#
# IMU acceleration norm까지 표시:
#
#   python scripts/04_plot_trajectory.py \
#     --trajectory output/03_trajectory_00000197/trajectory.csv \
#     --gps-local output/03_trajectory_00000197/gps_local.csv \
#     --baro output/02_trajectory_inputs_00000197/baro.csv \
#     --imu output/02_trajectory_inputs_00000197/imu.csv
#
# PNG 저장 + 화면 표시:
#
#   python scripts/04_plot_trajectory.py \
#     --trajectory output/03_trajectory_00000197/trajectory.csv \
#     --gps-local output/03_trajectory_00000197/gps_local.csv \
#     --baro output/02_trajectory_inputs_00000197/baro.csv \
#     --imu output/02_trajectory_inputs_00000197/imu.csv \
#     --save-dir output/04_plots_00000197
#
# PNG 저장만 하고 화면 표시 생략:
#
#   python scripts/04_plot_trajectory.py \
#     --trajectory output/03_trajectory_00000197/trajectory.csv \
#     --gps-local output/03_trajectory_00000197/gps_local.csv \
#     --baro output/02_trajectory_inputs_00000197/baro.csv \
#     --imu output/02_trajectory_inputs_00000197/imu.csv \
#     --save-dir output/04_plots_00000197 \
#     --no-show
#
# trajectory만 확인:
#
#   python scripts/04_plot_trajectory.py \
#     --trajectory output/03_trajectory_00000197/trajectory.csv
#
# 출력 예:
#
#   output/04_plots_00000197/
#   ├── 01_trajectory_3d.png
#   ├── 02_trajectory_xy.png
#   ├── 03_altitude_profile.png
#   ├── 04_gps_altitude_drift.png
#   ├── 05_velocity_profile.png
#   ├── 06_gps_quality.png
#   └── 07_imu_acc_norm.png
# =====================================================================

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


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


def save_figure(fig: plt.Figure, save_dir: Path | None, filename: str) -> None:
    """
    save_dir이 지정된 경우 figure를 PNG로 저장한다.
    """
    if save_dir is None:
        return

    save_dir.mkdir(parents=True, exist_ok=True)

    out_path = save_dir / filename
    fig.savefig(out_path, dpi=200, bbox_inches="tight")

    print(f"[SAVE] {out_path}")


def format_range(series: pd.Series) -> str:
    """
    min/max 범위를 문자열로 표시한다.
    """
    return f"{float(series.min()):.3f} ~ {float(series.max()):.3f}"


# =====================================================================
# [2] CSV 로딩
# =====================================================================

def load_trajectory_csv(path: Path) -> pd.DataFrame:
    """
    03_reconstruct_trajectory.py가 생성한 trajectory.csv를 읽는다.

    필요 컬럼:
        time_s, x, y, z, vx, vy, vz, lat, lng

    선택 컬럼:
        source_mode, alt_source
    """
    validate_file(path, "trajectory")

    df = pd.read_csv(path)

    require_columns(
        df,
        {"time_s", "x", "y", "z", "vx", "vy", "vz", "lat", "lng"},
        "trajectory",
    )

    df = df.sort_values("time_s")
    df = df.dropna(subset=["time_s", "x", "y", "z"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 2:
        raise ValueError("trajectory 데이터가 너무 적습니다. 최소 2개 이상 필요합니다.")

    return df


def load_gps_local_csv(path: Path | None) -> pd.DataFrame | None:
    """
    03_reconstruct_trajectory.py가 생성한 gps_local.csv를 읽는다.

    필요 컬럼:
        time_s, gx, gy, gz

    선택 컬럼:
        lat, lng, alt, nsats, hdop, status
    """
    if path is None:
        return None

    validate_file(path, "gps_local")

    df = pd.read_csv(path)

    require_columns(df, {"time_s", "gx", "gy", "gz"}, "gps_local")

    df = df.sort_values("time_s")
    df = df.dropna(subset=["time_s", "gx", "gy", "gz"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 1:
        raise ValueError("gps_local 데이터가 비어 있습니다.")

    return df


def load_baro_csv(path: Path | None) -> pd.DataFrame | None:
    """
    02_prepare_trajectory_csv.py가 생성한 baro.csv를 읽는다.

    필요 컬럼:
        time_s, baro_alt
    """
    if path is None:
        return None

    validate_file(path, "baro")

    df = pd.read_csv(path)

    require_columns(df, {"time_s", "baro_alt"}, "baro")

    df = df.sort_values("time_s")
    df = df.dropna(subset=["time_s", "baro_alt"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 1:
        raise ValueError("baro 데이터가 비어 있습니다.")

    return df


def load_imu_csv(path: Path | None) -> pd.DataFrame | None:
    """
    02_prepare_trajectory_csv.py가 생성한 imu.csv를 읽는다.

    필요 컬럼:
        time_s, ax, ay, az
    """
    if path is None:
        return None

    validate_file(path, "imu")

    df = pd.read_csv(path)

    require_columns(df, {"time_s", "ax", "ay", "az"}, "imu")

    df = df.sort_values("time_s")
    df = df.dropna(subset=["time_s", "ax", "ay", "az"])
    df = df.drop_duplicates(subset="time_s", keep="last")
    df = df.reset_index(drop=True)

    if len(df) < 2:
        raise ValueError("imu 데이터가 너무 적습니다. 최소 2개 이상 필요합니다.")

    return df


# =====================================================================
# [3] trajectory metadata
# =====================================================================

def get_trajectory_mode(trajectory_df: pd.DataFrame) -> str | None:
    """
    trajectory.csv의 source_mode 컬럼을 읽는다.
    """
    if "source_mode" not in trajectory_df.columns:
        return None

    modes = [
        str(value).strip()
        for value in trajectory_df["source_mode"].dropna().unique().tolist()
        if str(value).strip()
    ]

    if not modes:
        return None

    return modes[0]


def get_trajectory_alt_source(trajectory_df: pd.DataFrame) -> str | None:
    """
    trajectory.csv의 alt_source 컬럼을 읽는다.
    """
    if "alt_source" not in trajectory_df.columns:
        return None

    sources = [
        str(value).strip()
        for value in trajectory_df["alt_source"].dropna().unique().tolist()
        if str(value).strip()
    ]

    if not sources:
        return None

    return sources[0]


def make_title_prefix(title_prefix: str, trajectory_df: pd.DataFrame) -> str:
    """
    plot 제목에 reconstruction mode와 altitude source를 표시한다.
    """
    mode = get_trajectory_mode(trajectory_df)
    alt_source = get_trajectory_alt_source(trajectory_df)

    if mode is None:
        return title_prefix

    if alt_source is None:
        return f"{title_prefix} [{mode}]"

    return f"{title_prefix} [{mode} / {alt_source}]"


def make_trajectory_label(trajectory_df: pd.DataFrame) -> str:
    """
    trajectory line legend를 만든다.
    """
    mode = get_trajectory_mode(trajectory_df)

    if mode is None:
        return "Reconstructed trajectory"

    return f"Trajectory ({mode})"


# =====================================================================
# [4] plot용 데이터 가공
# =====================================================================

def make_projected_gps_for_plot(
    trajectory_df: pd.DataFrame,
    gps_local_df: pd.DataFrame,
) -> pd.DataFrame:
    """
    GPS x/y 관측점을 trajectory z 기준으로 투영한다.

    gps-baro 모드에서는 trajectory z가 BARO 보간 고도이고,
    gps_local.gz는 GPS altitude drift이다.

    따라서 3D plot에서 GPS observation을 그대로 gz로 찍으면
    실제 gps-baro 궤적과 다른 고도 기준이 섞여 보인다.

    이 함수는 GPS time_s에 해당하는 trajectory z를 보간해서 plot_z로 추가한다.
    """
    projected = gps_local_df.copy()

    projected["plot_z"] = np.interp(
        projected["time_s"].to_numpy(dtype=float),
        trajectory_df["time_s"].to_numpy(dtype=float),
        trajectory_df["z"].to_numpy(dtype=float),
    )

    return projected


def compute_baro_relative_altitude(baro_df: pd.DataFrame) -> pd.Series:
    """
    BARO 상대고도 계산.
    """
    return baro_df["baro_alt"].astype(float) - float(baro_df["baro_alt"].iloc[0])


# =====================================================================
# [5] 플롯 함수
# =====================================================================

def plot_trajectory_3d(
    trajectory_df: pd.DataFrame,
    gps_local_df: pd.DataFrame | None = None,
    trajectory_label: str = "Reconstructed trajectory",
    title: str = "ROK2 Reconstructed Trajectory - 3D",
) -> plt.Figure:
    """
    3D 궤적 플롯.

    축:
        x = East [m]
        y = North [m]
        z = Up [m]

    gps-baro 모드:
        trajectory z는 BARO 기반이다.
        GPS observation은 x/y 관측점만 의미가 크므로 z는 trajectory z로 투영한다.

    gps-only / imu-gps 모드:
        GPS observation은 gps_local.gz 그대로 표시한다.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        trajectory_df["x"],
        trajectory_df["y"],
        trajectory_df["z"],
        linewidth=1.5,
        label=trajectory_label,
    )

    mode = get_trajectory_mode(trajectory_df)

    if gps_local_df is not None:
        if mode == "gps-baro":
            gps_plot = make_projected_gps_for_plot(
                trajectory_df=trajectory_df,
                gps_local_df=gps_local_df,
            )

            ax.scatter(
                gps_plot["gx"],
                gps_plot["gy"],
                gps_plot["plot_z"],
                s=18,
                label="GPS horizontal obs. projected to BARO z",
            )
        else:
            ax.scatter(
                gps_local_df["gx"],
                gps_local_df["gy"],
                gps_local_df["gz"],
                s=18,
                label="GPS observations",
            )

    ax.set_xlabel("East x (m)")
    ax.set_ylabel("North y (m)")
    ax.set_zlabel("Up z (m)")
    ax.set_title(title)
    ax.legend()

    plt.tight_layout()

    return fig


def plot_trajectory_xy(
    trajectory_df: pd.DataFrame,
    gps_local_df: pd.DataFrame | None = None,
    trajectory_label: str = "Reconstructed trajectory",
    title: str = "ROK2 Reconstructed Trajectory - XY",
) -> plt.Figure:
    """
    XY 평면 궤적 플롯.

    축:
        x = East [m]
        y = North [m]
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.plot(
        trajectory_df["x"],
        trajectory_df["y"],
        linewidth=1.5,
        label=trajectory_label,
    )

    if gps_local_df is not None:
        ax.scatter(
            gps_local_df["gx"],
            gps_local_df["gy"],
            s=18,
            label="GPS observations",
        )

    ax.scatter(
        trajectory_df["x"].iloc[0],
        trajectory_df["y"].iloc[0],
        s=60,
        marker="o",
        label="Start",
    )

    ax.scatter(
        trajectory_df["x"].iloc[-1],
        trajectory_df["y"].iloc[-1],
        s=60,
        marker="x",
        label="End",
    )

    ax.set_xlabel("East x (m)")
    ax.set_ylabel("North y (m)")
    ax.set_title(title)
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend()

    plt.tight_layout()

    return fig


def plot_altitude_profile(
    trajectory_df: pd.DataFrame,
    baro_df: pd.DataFrame | None = None,
    trajectory_label: str = "Reconstructed altitude",
    title: str = "Altitude Profile",
) -> plt.Figure:
    """
    시간에 따른 고도 플롯.

    포함:
        - trajectory z
        - BARO relative altitude

    제외:
        - GPS altitude drift

    GPS altitude drift는 plot_gps_altitude_drift()에서 별도 표시한다.
    """
    fig, ax = plt.subplots(figsize=(11, 5))

    ax.plot(
        trajectory_df["time_s"],
        trajectory_df["z"],
        linewidth=1.5,
        label=trajectory_label,
    )

    if baro_df is not None:
        baro_rel_alt = compute_baro_relative_altitude(baro_df)

        ax.plot(
            baro_df["time_s"],
            baro_rel_alt,
            linewidth=1.0,
            label="BARO relative altitude",
        )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m, relative)")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend()

    plt.tight_layout()

    return fig


def plot_gps_altitude_drift(
    gps_local_df: pd.DataFrame,
    title: str = "GPS Altitude Drift",
) -> plt.Figure:
    """
    GPS 상대고도 drift 확인용 플롯.

    gps-baro mode에서 gps_local.gz는 trajectory z로 쓰지 않는다.
    이 plot은 GPS altitude가 얼마나 drift하는지 별도로 확인하기 위한 것이다.
    """
    fig, ax = plt.subplots(figsize=(11, 5))

    ax.plot(
        gps_local_df["time_s"],
        gps_local_df["gz"],
        linewidth=1.0,
        marker=".",
        label="GPS relative altitude",
    )

    ax.axhline(
        0.0,
        linestyle="--",
        linewidth=1.0,
        label="Initial GPS altitude",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("GPS altitude drift (m, relative)")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend()

    plt.tight_layout()

    return fig


def plot_velocity_profile(
    trajectory_df: pd.DataFrame,
    title: str = "Velocity Profile",
) -> plt.Figure:
    """
    시간에 따른 속도 성분 및 속도 norm 플롯.
    """
    fig, ax = plt.subplots(figsize=(11, 5))

    speed = np.sqrt(
        trajectory_df["vx"] ** 2
        + trajectory_df["vy"] ** 2
        + trajectory_df["vz"] ** 2
    )

    ax.plot(
        trajectory_df["time_s"],
        trajectory_df["vx"],
        linewidth=1.0,
        label="vx",
    )

    ax.plot(
        trajectory_df["time_s"],
        trajectory_df["vy"],
        linewidth=1.0,
        label="vy",
    )

    ax.plot(
        trajectory_df["time_s"],
        trajectory_df["vz"],
        linewidth=1.0,
        label="vz",
    )

    ax.plot(
        trajectory_df["time_s"],
        speed,
        linewidth=1.4,
        label="speed norm",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend()

    plt.tight_layout()

    return fig


def plot_gps_quality(
    gps_local_df: pd.DataFrame,
    title: str = "GPS Quality",
) -> plt.Figure | None:
    """
    GPS 품질 플롯.

    nsats, hdop, status 컬럼 중 하나라도 있으면 그린다.
    """
    has_nsats = "nsats" in gps_local_df.columns and not gps_local_df["nsats"].isna().all()
    has_hdop = "hdop" in gps_local_df.columns and not gps_local_df["hdop"].isna().all()
    has_status = "status" in gps_local_df.columns and not gps_local_df["status"].isna().all()

    if not (has_nsats or has_hdop or has_status):
        return None

    fig, ax = plt.subplots(figsize=(11, 5))

    if has_nsats:
        ax.plot(
            gps_local_df["time_s"],
            gps_local_df["nsats"],
            marker=".",
            linewidth=1.0,
            label="NSats",
        )

    if has_status:
        ax.plot(
            gps_local_df["time_s"],
            gps_local_df["status"],
            marker=".",
            linewidth=1.0,
            label="Status",
        )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("NSats / Status")
    ax.set_title(title)
    ax.grid(alpha=0.3)

    if has_hdop:
        ax2 = ax.twinx()

        ax2.plot(
            gps_local_df["time_s"],
            gps_local_df["hdop"],
            linewidth=1.0,
            label="HDop",
        )

        ax2.set_ylabel("HDop")

        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2)
    else:
        ax.legend()

    plt.tight_layout()

    return fig


def plot_imu_acc_norm(
    imu_df: pd.DataFrame,
    title: str = "IMU Acceleration Norm",
) -> plt.Figure:
    """
    IMU 가속도 norm 플롯.

    정지 구간에서는 보통 9.81 m/s² 근처가 나와야 한다.
    이동/진동/충격 구간에서는 피크가 나타날 수 있다.
    """
    fig, ax = plt.subplots(figsize=(11, 5))

    acc_norm = np.sqrt(
        imu_df["ax"] ** 2
        + imu_df["ay"] ** 2
        + imu_df["az"] ** 2
    )

    ax.plot(
        imu_df["time_s"],
        acc_norm,
        linewidth=1.0,
        label="|acc|",
    )

    ax.axhline(
        9.81,
        linestyle="--",
        linewidth=1.0,
        label="1 g reference",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration norm (m/s²)")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend()

    plt.tight_layout()

    return fig


# =====================================================================
# [6] 플롯 파이프라인
# =====================================================================

def print_basic_ranges(
    trajectory_df: pd.DataFrame,
    gps_local_df: pd.DataFrame | None,
    baro_df: pd.DataFrame | None,
) -> None:
    """
    플롯 전 주요 범위를 콘솔에 출력한다.
    """
    print("[RANGE] trajectory")
    print(f"        x: {format_range(trajectory_df['x'])} m")
    print(f"        y: {format_range(trajectory_df['y'])} m")
    print(f"        z: {format_range(trajectory_df['z'])} m")

    speed = np.sqrt(
        trajectory_df["vx"] ** 2
        + trajectory_df["vy"] ** 2
        + trajectory_df["vz"] ** 2
    )
    print(f"        speed max: {float(speed.max()):.3f} m/s")

    if gps_local_df is not None:
        print("[RANGE] gps_local")
        print(f"        gx: {format_range(gps_local_df['gx'])} m")
        print(f"        gy: {format_range(gps_local_df['gy'])} m")
        print(f"        gz: {format_range(gps_local_df['gz'])} m")

    if baro_df is not None:
        baro_rel_alt = compute_baro_relative_altitude(baro_df)
        print("[RANGE] baro")
        print(f"        rel_alt: {format_range(baro_rel_alt)} m")


def plot_from_csv(
    trajectory_path: Path,
    gps_local_path: Path | None,
    baro_path: Path | None,
    imu_path: Path | None,
    save_dir: Path | None,
    no_show: bool,
    title_prefix: str,
) -> int:
    """
    CSV 로딩 후 전체 플롯 생성.
    """
    print(f"[1/4] Loading trajectory: {trajectory_path}")
    trajectory_df = load_trajectory_csv(trajectory_path)
    print(
        f"      -> {len(trajectory_df)} rows, "
        f"{trajectory_df['time_s'].iloc[0]:.3f}s ~ "
        f"{trajectory_df['time_s'].iloc[-1]:.3f}s"
    )

    gps_local_df = None

    if gps_local_path is not None:
        print(f"[2/4] Loading GPS local: {gps_local_path}")
        gps_local_df = load_gps_local_csv(gps_local_path)
        assert gps_local_df is not None
        print(
            f"      -> {len(gps_local_df)} rows, "
            f"{gps_local_df['time_s'].iloc[0]:.3f}s ~ "
            f"{gps_local_df['time_s'].iloc[-1]:.3f}s"
        )
    else:
        print("[2/4] Loading GPS local: skipped")

    baro_df = None

    if baro_path is not None:
        print(f"[3/4] Loading BARO: {baro_path}")
        baro_df = load_baro_csv(baro_path)
        assert baro_df is not None
        print(
            f"      -> {len(baro_df)} rows, "
            f"{baro_df['time_s'].iloc[0]:.3f}s ~ "
            f"{baro_df['time_s'].iloc[-1]:.3f}s"
        )
    else:
        print("[3/4] Loading BARO: skipped")

    imu_df = None

    if imu_path is not None:
        print(f"[4/4] Loading IMU: {imu_path}")
        imu_df = load_imu_csv(imu_path)
        assert imu_df is not None
        print(
            f"      -> {len(imu_df)} rows, "
            f"{imu_df['time_s'].iloc[0]:.3f}s ~ "
            f"{imu_df['time_s'].iloc[-1]:.3f}s"
        )
    else:
        print("[4/4] Loading IMU: skipped")

    print_basic_ranges(
        trajectory_df=trajectory_df,
        gps_local_df=gps_local_df,
        baro_df=baro_df,
    )

    print("[INFO] Generating plots...")

    title_prefix = make_title_prefix(title_prefix, trajectory_df)
    trajectory_label = make_trajectory_label(trajectory_df)

    fig_3d = plot_trajectory_3d(
        trajectory_df=trajectory_df,
        gps_local_df=gps_local_df,
        trajectory_label=trajectory_label,
        title=f"{title_prefix} - 3D Trajectory",
    )
    save_figure(fig_3d, save_dir, "01_trajectory_3d.png")

    fig_xy = plot_trajectory_xy(
        trajectory_df=trajectory_df,
        gps_local_df=gps_local_df,
        trajectory_label=trajectory_label,
        title=f"{title_prefix} - XY Trajectory",
    )
    save_figure(fig_xy, save_dir, "02_trajectory_xy.png")

    fig_alt = plot_altitude_profile(
        trajectory_df=trajectory_df,
        baro_df=baro_df,
        trajectory_label=f"{trajectory_label} altitude",
        title=f"{title_prefix} - Altitude Profile",
    )
    save_figure(fig_alt, save_dir, "03_altitude_profile.png")

    if gps_local_df is not None:
        fig_gps_alt = plot_gps_altitude_drift(
            gps_local_df=gps_local_df,
            title=f"{title_prefix} - GPS Altitude Drift",
        )
        save_figure(fig_gps_alt, save_dir, "04_gps_altitude_drift.png")
    else:
        print("[INFO] GPS altitude drift plot skipped: gps_local.csv not provided.")

    fig_vel = plot_velocity_profile(
        trajectory_df=trajectory_df,
        title=f"{title_prefix} - Velocity Profile",
    )
    save_figure(fig_vel, save_dir, "05_velocity_profile.png")

    if gps_local_df is not None:
        fig_gps = plot_gps_quality(
            gps_local_df=gps_local_df,
            title=f"{title_prefix} - GPS Quality",
        )

        if fig_gps is not None:
            save_figure(fig_gps, save_dir, "06_gps_quality.png")
        else:
            print("[INFO] GPS quality plot skipped: nsats/hdop/status columns not available.")
    else:
        print("[INFO] GPS quality plot skipped: gps_local.csv not provided.")

    if imu_df is not None:
        fig_imu = plot_imu_acc_norm(
            imu_df=imu_df,
            title=f"{title_prefix} - IMU Acceleration Norm",
        )
        save_figure(fig_imu, save_dir, "07_imu_acc_norm.png")
    else:
        print("[INFO] IMU acceleration norm plot skipped: imu.csv not provided.")

    if not no_show:
        plt.show()
    else:
        plt.close("all")

    print("[DONE] Plotting completed.")

    return 0


# =====================================================================
# [7] CLI
# =====================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot ROK2 reconstructed trajectory and related sensor data."
    )

    parser.add_argument(
        "--trajectory",
        type=Path,
        required=True,
        help="Path to trajectory CSV generated by 03_reconstruct_trajectory.py.",
    )

    parser.add_argument(
        "--gps-local",
        type=Path,
        default=None,
        help="Optional path to GPS local CSV generated by 03_reconstruct_trajectory.py.",
    )

    parser.add_argument(
        "--baro",
        type=Path,
        default=None,
        help="Optional path to prepared baro.csv generated by 02_prepare_trajectory_csv.py.",
    )

    parser.add_argument(
        "--imu",
        type=Path,
        default=None,
        help="Optional path to prepared imu.csv generated by 02_prepare_trajectory_csv.py.",
    )

    parser.add_argument(
        "--save-dir",
        type=Path,
        default=None,
        help="Optional directory to save plot PNG files.",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not show matplotlib windows. Useful when only saving PNG files.",
    )

    parser.add_argument(
        "--title-prefix",
        default="ROK2",
        help="Title prefix for generated plots. Default: ROK2.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        return plot_from_csv(
            trajectory_path=args.trajectory,
            gps_local_path=args.gps_local,
            baro_path=args.baro,
            imu_path=args.imu,
            save_dir=args.save_dir,
            no_show=args.no_show,
            title_prefix=args.title_prefix,
        )
    except Exception as exc:
        print(f"[ERROR] Failed to plot trajectory: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())