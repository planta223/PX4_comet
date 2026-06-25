#!/usr/bin/env python3
"""
02_prepare_trajectory_csv.py

ROK2 궤적 분석 파이프라인 2단계:
01_export_bin_to_csv.py에서 추출한 raw IMU/GPS/BARO CSV를
궤적 재구성용 표준 CSV로 정리하는 스크립트입니다.

입력:
    output/csv_<log_id>/IMU.csv
    output/csv_<log_id>/GPS.csv
    output/csv_<log_id>/BARO.csv   # 선택

출력:
    output/trajectory_inputs_<log_id>/imu.csv
    output/trajectory_inputs_<log_id>/gps.csv
    output/trajectory_inputs_<log_id>/baro.csv
    output/trajectory_inputs_<log_id>/prepare_summary.md

역할:
    - TimeUS를 공통 t0_us 기준 time_s로 변환
    - IMU 인스턴스 선택. 기본 I == 0
    - GPS fix 상태 필터링. 기본 Status >= 3
    - GPS lat/lng == 0 제거
    - BARO 인스턴스 선택. 기본 I == 0
    - 03_reconstruct_trajectory.py에서 사용할 표준 컬럼으로 저장

주의:
    - 이 스크립트는 궤적을 계산하지 않습니다.
    - RK4 적분, GPS 보정, 플롯은 03_reconstruct_trajectory.py에서 수행합니다.
    - ATT/XKQ 기반 body-frame -> earth-frame 회전 변환도 여기서는 수행하지 않습니다.
"""

# =====================================================================
# 실행 방법
# ---------------------------------------------------------------------
# 기본 실행:
#
#   python scripts/02_prepare_trajectory_csv.py \
#     --imu output/csv_00000197/IMU.csv \
#     --gps output/csv_00000197/GPS.csv \
#     --baro output/csv_00000197/BARO.csv \
#     --out-dir output/trajectory_inputs_00000197
#
# BARO 없이 실행:
#
#   python scripts/02_prepare_trajectory_csv.py \
#     --imu output/csv_00000197/IMU.csv \
#     --gps output/csv_00000197/GPS.csv \
#     --out-dir output/trajectory_inputs_00000197
#
# IMU 인스턴스 변경:
#
#   python scripts/02_prepare_trajectory_csv.py \
#     --imu output/csv_00000197/IMU.csv \
#     --gps output/csv_00000197/GPS.csv \
#     --baro output/csv_00000197/BARO.csv \
#     --imu-instance 1 \
#     --out-dir output/trajectory_inputs_00000197
#
# GPS Status 필터 변경:
#
#   python scripts/02_prepare_trajectory_csv.py \
#     --imu output/csv_00000197/IMU.csv \
#     --gps output/csv_00000197/GPS.csv \
#     --baro output/csv_00000197/BARO.csv \
#     --min-gps-status 2 \
#     --out-dir output/trajectory_inputs_00000197
#
# 출력:
#
#   output/trajectory_inputs_00000197/imu.csv
#   output/trajectory_inputs_00000197/gps.csv
#   output/trajectory_inputs_00000197/baro.csv
#   output/trajectory_inputs_00000197/prepare_summary.md
#
# 후속 실행:
#
#   python scripts/03_reconstruct_trajectory.py \
#     --imu output/trajectory_inputs_00000197/imu.csv \
#     --gps output/trajectory_inputs_00000197/gps.csv \
#     --baro output/trajectory_inputs_00000197/baro.csv \
#     --out output/trajectory_00000197.csv
# =====================================================================

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


# =====================================================================
# [1] 공통 유틸
# =====================================================================

def require_columns(df: pd.DataFrame, required: set[str], name: str) -> None:
    """
    CSV에 필요한 컬럼이 있는지 검사한다.
    """
    missing = required - set(df.columns)

    if missing:
        raise ValueError(
            f"{name} CSV에 필요한 컬럼이 없습니다: {sorted(missing)}\n"
            f"실제 컬럼: {list(df.columns)}"
        )


def read_min_time_us(csv_path: Path) -> float:
    """
    CSV에서 TimeUS의 최솟값을 읽는다.

    목적:
        IMU/GPS/BARO는 센서별 첫 기록 시각이 다를 수 있다.
        따라서 전체 센서 중 가장 이른 TimeUS를 공통 t0_us로 사용한다.

    주의:
        첫 행 값이 아니라 TimeUS.min()을 사용한다.
        CSV 정렬이 깨져 있어도 공통 시간 기준이 안정적으로 잡힌다.
    """
    df = pd.read_csv(csv_path, usecols=["TimeUS"])

    if df.empty:
        raise ValueError(f"TimeUS를 읽을 수 없습니다: {csv_path}")

    return float(df["TimeUS"].min())


def make_common_t0_us(paths: list[Path]) -> float:
    """
    여러 센서 CSV의 TimeUS 최솟값 중 가장 이른 값을 공통 시작 시각으로 사용한다.
    """
    if not paths:
        raise ValueError("t0_us 계산에 사용할 CSV 경로가 없습니다.")

    return min(read_min_time_us(path) for path in paths)


def normalize_time_us(
    df: pd.DataFrame,
    t0_us: float,
    time_col: str = "TimeUS",
) -> pd.Series:
    """
    TimeUS를 공통 t0_us 기준 초 단위 time_s로 변환한다.
    """
    if time_col not in df.columns:
        raise ValueError(f"필요한 시간 컬럼이 없습니다: {time_col}")

    return (df[time_col].astype(float) - t0_us) / 1_000_000.0


def safe_float_series(df: pd.DataFrame, column: str) -> pd.Series:
    """
    컬럼이 있으면 float Series로 반환하고, 없으면 NaN Series를 반환한다.
    """
    if column in df.columns:
        return df[column].astype(float)

    return pd.Series(np.nan, index=df.index, dtype=float)


# =====================================================================
# [2] IMU 준비
# =====================================================================

def prepare_imu_csv(
    imu_path: Path,
    t0_us: float,
    imu_instance: int = 0,
) -> tuple[pd.DataFrame, dict[str, object]]:
    """
    raw IMU.csv를 궤적 계산용 imu.csv로 정리한다.

    입력 예상 컬럼:
        Type, TimeUS, I, GyrX, GyrY, GyrZ, AccX, AccY, AccZ, ...

    출력 컬럼:
        time_s, ax, ay, az, gx, gy, gz
    """
    df = pd.read_csv(imu_path)

    require_columns(
        df,
        {"TimeUS", "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ"},
        "IMU",
    )

    raw_rows = len(df)
    has_instance = "I" in df.columns
    available_instances: list[object] = []

    if has_instance:
        available_instances = sorted(df["I"].dropna().unique().tolist())
        df = df[df["I"] == imu_instance].copy()

        if df.empty:
            raise ValueError(
                f"IMU 인스턴스 I={imu_instance} 데이터가 없습니다. "
                f"사용 가능한 I 값: {available_instances}"
            )

    df = df.sort_values("TimeUS").reset_index(drop=True)

    out = pd.DataFrame(
        {
            "time_s": normalize_time_us(df, t0_us=t0_us),
            "ax": df["AccX"].astype(float),
            "ay": df["AccY"].astype(float),
            "az": df["AccZ"].astype(float),
            "gx": df["GyrX"].astype(float),
            "gy": df["GyrY"].astype(float),
            "gz": df["GyrZ"].astype(float),
        }
    )

    out = out.dropna(subset=["time_s", "ax", "ay", "az"]).reset_index(drop=True)

    if len(out) < 2:
        raise ValueError("정리된 IMU 데이터가 너무 적습니다. 최소 2개 이상 필요합니다.")

    summary = {
        "input": imu_path,
        "raw_rows": raw_rows,
        "output_rows": len(out),
        "imu_instance": imu_instance if has_instance else "N/A",
        "available_instances": available_instances,
        "time_start_s": float(out["time_s"].iloc[0]),
        "time_end_s": float(out["time_s"].iloc[-1]),
        "columns": list(out.columns),
    }

    return out, summary


# =====================================================================
# [3] GPS 준비
# =====================================================================

def prepare_gps_csv(
    gps_path: Path,
    t0_us: float,
    min_status: int = 3,
) -> tuple[pd.DataFrame, dict[str, object]]:
    """
    raw GPS.csv를 궤적 계산용 gps.csv로 정리한다.

    입력 예상 컬럼:
        Type, TimeUS, I, Status, NSats, HDop, Lat, Lng, Alt, Spd, VZ, ...

    출력 컬럼:
        time_s, lat, lng, alt, spd, vz, nsats, hdop, status
    """
    df = pd.read_csv(gps_path)

    require_columns(df, {"TimeUS", "Lat", "Lng", "Alt"}, "GPS")

    raw_rows = len(df)
    has_status = "Status" in df.columns

    if has_status:
        df = df[df["Status"].astype(float) >= min_status].copy()

    after_status_rows = len(df)

    df = df[(df["Lat"] != 0) & (df["Lng"] != 0)].copy()
    after_coord_rows = len(df)

    if df.empty:
        raise ValueError(
            "유효한 GPS 데이터가 없습니다. "
            "Status 필터 또는 lat/lng == 0 제거 조건을 확인하십시오."
        )

    df = df.sort_values("TimeUS").reset_index(drop=True)

    out = pd.DataFrame(
        {
            "time_s": normalize_time_us(df, t0_us=t0_us),
            "lat": df["Lat"].astype(float),
            "lng": df["Lng"].astype(float),
            "alt": df["Alt"].astype(float),
            "spd": safe_float_series(df, "Spd"),
            "vz": safe_float_series(df, "VZ"),
            "nsats": safe_float_series(df, "NSats"),
            "hdop": safe_float_series(df, "HDop"),
            "status": safe_float_series(df, "Status"),
        }
    )

    out = out.dropna(subset=["time_s", "lat", "lng", "alt"]).reset_index(drop=True)

    if len(out) < 1:
        raise ValueError("정리된 GPS 데이터가 비어 있습니다.")

    summary = {
        "input": gps_path,
        "raw_rows": raw_rows,
        "after_status_rows": after_status_rows,
        "after_coord_rows": after_coord_rows,
        "output_rows": len(out),
        "min_status": min_status if has_status else "N/A",
        "time_start_s": float(out["time_s"].iloc[0]),
        "time_end_s": float(out["time_s"].iloc[-1]),
        "columns": list(out.columns),
    }

    return out, summary


# =====================================================================
# [4] BARO 준비
# =====================================================================

def prepare_baro_csv(
    baro_path: Path,
    t0_us: float,
    baro_instance: int = 0,
) -> tuple[pd.DataFrame, dict[str, object]]:
    """
    raw BARO.csv를 궤적 검증용 baro.csv로 정리한다.

    입력 예상 컬럼:
        Type, TimeUS, I, Alt, AltAMSL, Press, Temp, CRt, Health, ...

    출력 컬럼:
        time_s, baro_alt, baro_alt_amsl, press, temp, climb_rate, health
    """
    df = pd.read_csv(baro_path)

    require_columns(df, {"TimeUS", "Alt"}, "BARO")

    raw_rows = len(df)
    has_instance = "I" in df.columns
    available_instances: list[object] = []

    if has_instance:
        available_instances = sorted(df["I"].dropna().unique().tolist())
        df = df[df["I"] == baro_instance].copy()

        if df.empty:
            raise ValueError(
                f"BARO 인스턴스 I={baro_instance} 데이터가 없습니다. "
                f"사용 가능한 I 값: {available_instances}"
            )

    df = df.sort_values("TimeUS").reset_index(drop=True)

    out = pd.DataFrame(
        {
            "time_s": normalize_time_us(df, t0_us=t0_us),
            "baro_alt": df["Alt"].astype(float),
            "baro_alt_amsl": safe_float_series(df, "AltAMSL"),
            "press": safe_float_series(df, "Press"),
            "temp": safe_float_series(df, "Temp"),
            "climb_rate": safe_float_series(df, "CRt"),
        }
    )

    if "Health" in df.columns:
        out["health"] = df["Health"]
    else:
        out["health"] = np.nan

    out = out.dropna(subset=["time_s", "baro_alt"]).reset_index(drop=True)

    if len(out) < 1:
        raise ValueError("정리된 BARO 데이터가 비어 있습니다.")

    summary = {
        "input": baro_path,
        "raw_rows": raw_rows,
        "output_rows": len(out),
        "baro_instance": baro_instance if has_instance else "N/A",
        "available_instances": available_instances,
        "time_start_s": float(out["time_s"].iloc[0]),
        "time_end_s": float(out["time_s"].iloc[-1]),
        "columns": list(out.columns),
    }

    return out, summary


# =====================================================================
# [5] 요약 파일 작성
# =====================================================================

def format_summary_section(name: str, summary: dict[str, object]) -> list[str]:
    """
    센서별 summary dict를 Markdown 섹션으로 변환한다.
    """
    lines = [
        f"## {name}",
        "",
    ]

    for key, value in summary.items():
        if key == "columns":
            continue
        lines.append(f"- {key}: `{value}`")

    lines.extend(
        [
            "",
            "### Columns",
            "",
            "```text",
            ", ".join(str(col) for col in summary.get("columns", [])),
            "```",
            "",
        ]
    )

    return lines


def write_prepare_summary(
    summary_path: Path,
    t0_us: float,
    output_paths: dict[str, Path],
    summaries: dict[str, dict[str, object]],
) -> None:
    """
    전처리 결과 요약 Markdown 파일을 작성한다.
    """
    lines = [
        "# Trajectory CSV Prepare Summary",
        "",
        f"- Common t0_us: `{t0_us:.0f}`",
        "",
        "## Output files",
        "",
    ]

    for name, path in output_paths.items():
        lines.append(f"- {name}: `{path}`")

    lines.append("")

    for name, summary in summaries.items():
        lines.extend(format_summary_section(name, summary))

    summary_path.write_text("\n".join(lines), encoding="utf-8")


# =====================================================================
# [6] 메인 처리
# =====================================================================

def prepare_trajectory_csv(
    imu_path: Path,
    gps_path: Path,
    baro_path: Path | None,
    out_dir: Path,
    imu_instance: int,
    baro_instance: int,
    min_gps_status: int,
) -> int:
    """
    전체 준비 파이프라인 실행.
    """
    input_paths = [imu_path, gps_path]

    if baro_path is not None:
        input_paths.append(baro_path)

    for path in input_paths:
        if not path.exists():
            print(f"[ERROR] Input CSV not found: {path}", file=sys.stderr)
            return 1

        if not path.is_file():
            print(f"[ERROR] Input path is not a file: {path}", file=sys.stderr)
            return 1

    out_dir.mkdir(parents=True, exist_ok=True)

    print("[INFO] Calculating common t0_us...")
    t0_us = make_common_t0_us(input_paths)
    print(f"[INFO] Common t0_us: {t0_us:.0f}")

    summaries: dict[str, dict[str, object]] = {}
    output_paths: dict[str, Path] = {}

    print(f"[1/3] Preparing IMU: {imu_path}")
    imu_df, imu_summary = prepare_imu_csv(
        imu_path=imu_path,
        t0_us=t0_us,
        imu_instance=imu_instance,
    )

    imu_out = out_dir / "imu.csv"
    imu_df.to_csv(imu_out, index=False)

    summaries["IMU"] = imu_summary
    output_paths["imu"] = imu_out

    print(
        f"      -> {len(imu_df)} rows, "
        f"{imu_df['time_s'].iloc[0]:.3f}s ~ {imu_df['time_s'].iloc[-1]:.3f}s"
    )

    print(f"[2/3] Preparing GPS: {gps_path}")
    gps_df, gps_summary = prepare_gps_csv(
        gps_path=gps_path,
        t0_us=t0_us,
        min_status=min_gps_status,
    )

    gps_out = out_dir / "gps.csv"
    gps_df.to_csv(gps_out, index=False)

    summaries["GPS"] = gps_summary
    output_paths["gps"] = gps_out

    print(
        f"      -> {len(gps_df)} rows, "
        f"{gps_df['time_s'].iloc[0]:.3f}s ~ {gps_df['time_s'].iloc[-1]:.3f}s"
    )

    if baro_path is not None:
        print(f"[3/3] Preparing BARO: {baro_path}")
        baro_df, baro_summary = prepare_baro_csv(
            baro_path=baro_path,
            t0_us=t0_us,
            baro_instance=baro_instance,
        )

        baro_out = out_dir / "baro.csv"
        baro_df.to_csv(baro_out, index=False)

        summaries["BARO"] = baro_summary
        output_paths["baro"] = baro_out

        print(
            f"      -> {len(baro_df)} rows, "
            f"{baro_df['time_s'].iloc[0]:.3f}s ~ {baro_df['time_s'].iloc[-1]:.3f}s"
        )
    else:
        print("[3/3] Preparing BARO: skipped")

    summary_path = out_dir / "prepare_summary.md"

    write_prepare_summary(
        summary_path=summary_path,
        t0_us=t0_us,
        output_paths=output_paths,
        summaries=summaries,
    )

    print(f"[DONE] Prepared CSV files saved to: {out_dir}")
    print(f"[DONE] Summary written: {summary_path}")

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare raw Pixhawk/ArduPilot CSV logs for trajectory reconstruction."
    )

    parser.add_argument(
        "--imu",
        type=Path,
        required=True,
        help="Path to raw IMU.csv exported by 01_export_bin_to_csv.py.",
    )

    parser.add_argument(
        "--gps",
        type=Path,
        required=True,
        help="Path to raw GPS.csv exported by 01_export_bin_to_csv.py.",
    )

    parser.add_argument(
        "--baro",
        type=Path,
        default=None,
        help="Optional path to raw BARO.csv exported by 01_export_bin_to_csv.py.",
    )

    parser.add_argument(
        "--out-dir",
        type=Path,
        required=True,
        help="Output directory for prepared CSV files.",
    )

    parser.add_argument(
        "--imu-instance",
        type=int,
        default=0,
        help="IMU instance index to use when column I exists. Default: 0.",
    )

    parser.add_argument(
        "--baro-instance",
        type=int,
        default=0,
        help="BARO instance index to use when column I exists. Default: 0.",
    )

    parser.add_argument(
        "--min-gps-status",
        type=int,
        default=3,
        help="Minimum GPS Status value to keep when Status column exists. Default: 3.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        return prepare_trajectory_csv(
            imu_path=args.imu,
            gps_path=args.gps,
            baro_path=args.baro,
            out_dir=args.out_dir,
            imu_instance=args.imu_instance,
            baro_instance=args.baro_instance,
            min_gps_status=args.min_gps_status,
        )
    except Exception as exc:
        print(f"[ERROR] Failed to prepare trajectory CSV: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())