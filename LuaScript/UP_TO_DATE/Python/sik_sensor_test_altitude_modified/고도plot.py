#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


ALT_VALID_BIT = 0x10

# 현재 로그에서 확인되는 구간 경계입니다.
# 시간은 CSV의 elapsed_s 기준입니다.
PHASES = [
    (78.0, 90.0, "-4F descent"),
    (90.0, 102.0, "-4F hold"),
    (102.0, 115.0, "+5F ascent"),
    (115.0, 128.0, "+5F hold"),
    (128.0, 136.0, "-1F descent"),
    (136.0, None, "-1F hold"),
]

# 움직임이 시작되기 전 긴 대기 구간을 제외하고 싶으면 78.0을 사용합니다.
# 전체 altitude 유효구간을 보고 싶으면 None으로 바꾸십시오.
PLOT_START_S = 78.0
PLOT_END_S = None

# 그래프 표시용 중앙값 필터 샘플 수
SMOOTHING_WINDOW = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot only valid COMET barometric altitude samples "
            "from a SiK dashboard CSV log."
        )
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        type=Path,
        default=Path("comet_sik_debug_20260803_225150.csv"),
        help="Input CSV path",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("comet_valid_altitude_phases.png"),
        help="Output PNG path",
    )
    parser.add_argument(
        "--show-all-valid",
        action="store_true",
        help="Plot the full altitude-valid interval instead of cropping at 78 s",
    )
    return parser.parse_args()


def load_valid_altitude(csv_path: Path) -> pd.DataFrame:
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    df = pd.read_csv(csv_path)

    required_columns = {
        "elapsed_s",
        "message_name",
        "altitude_m",
        "valid_flags",
    }
    missing = required_columns - set(df.columns)
    if missing:
        raise ValueError(
            "CSV is missing required columns: "
            + ", ".join(sorted(missing))
        )

    # C_ALT 메시지가 들어온 행만 사용합니다.
    altitude = df.loc[
        df["message_name"].eq("C_ALT"),
        ["elapsed_s", "altitude_m", "valid_flags"],
    ].copy()

    altitude["elapsed_s"] = pd.to_numeric(
        altitude["elapsed_s"],
        errors="coerce",
    )
    altitude["altitude_m"] = pd.to_numeric(
        altitude["altitude_m"],
        errors="coerce",
    )
    altitude["valid_flags"] = pd.to_numeric(
        altitude["valid_flags"],
        errors="coerce",
    ).fillna(0).astype(int)

    # altitude_valid bit(bit 4, 0x10)이 켜진 데이터만 남깁니다.
    altitude = altitude.loc[
        (altitude["valid_flags"] & ALT_VALID_BIT) != 0
    ]

    altitude = altitude.dropna(
        subset=["elapsed_s", "altitude_m"]
    ).sort_values("elapsed_s")

    if altitude.empty:
        raise ValueError(
            "No valid altitude samples were found. "
            "Check whether C_VALID bit 0x10 is being transmitted."
        )

    # 표시용 중앙값 필터입니다. 원본값은 altitude_m에 그대로 유지합니다.
    altitude["altitude_filtered_m"] = (
        altitude["altitude_m"]
        .rolling(
            window=SMOOTHING_WINDOW,
            center=True,
            min_periods=1,
        )
        .median()
    )

    return altitude


def add_phase_annotations(
    axis,
    phases: list[tuple[float, float | None, str]],
    plot_end_s: float,
) -> None:
    y_min, y_max = axis.get_ylim()
    y_text = y_max - 0.06 * (y_max - y_min)

    for start_s, end_s, label in phases:
        actual_end_s = plot_end_s if end_s is None else end_s

        if actual_end_s < axis.get_xlim()[0]:
            continue
        if start_s > axis.get_xlim()[1]:
            continue

        axis.axvline(start_s, linestyle="--", linewidth=0.9)

        center_s = (start_s + actual_end_s) / 2.0
        axis.text(
            center_s,
            y_text,
            label,
            ha="center",
            va="top",
            fontsize=9,
            rotation=0,
        )


def main() -> int:
    args = parse_args()
    altitude = load_valid_altitude(args.csv_path)

    plot_start_s = (
        float(altitude["elapsed_s"].min())
        if args.show_all_valid
        else PLOT_START_S
    )
    plot_end_s = (
        float(altitude["elapsed_s"].max())
        if PLOT_END_S is None
        else PLOT_END_S
    )

    visible = altitude.loc[
        altitude["elapsed_s"].between(
            plot_start_s,
            plot_end_s,
            inclusive="both",
        )
    ].copy()

    if visible.empty:
        raise ValueError(
            f"No valid altitude samples in "
            f"{plot_start_s:.1f}–{plot_end_s:.1f} s."
        )

    # 선택 구간의 시작시간을 0초로 맞춥니다.
    visible["plot_time_s"] = (
        visible["elapsed_s"] - plot_start_s
    )

    adjusted_phases = []
    for start_s, end_s, label in PHASES:
        adjusted_start = start_s - plot_start_s
        adjusted_end = (
            None if end_s is None
            else end_s - plot_start_s
        )
        adjusted_phases.append(
            (adjusted_start, adjusted_end, label)
        )

    figure, axis = plt.subplots(figsize=(13, 6))

    # 얇은 선: 원본값
    axis.plot(
        visible["plot_time_s"],
        visible["altitude_m"],
        linewidth=0.8,
        alpha=0.45,
        label="Raw valid altitude",
    )

    # 굵은 선: 5-sample rolling median
    axis.plot(
        visible["plot_time_s"],
        visible["altitude_filtered_m"],
        linewidth=1.8,
        label="Median-filtered altitude",
    )

    axis.axhline(
        0.0,
        linestyle=":",
        linewidth=1.0,
        label="Startup reference",
    )

    axis.set_title(
        "COMET Barometric Relative Altitude — Valid Samples Only"
    )
    axis.set_xlabel("Time from selected valid interval [s]")
    axis.set_ylabel("Relative altitude [m]")
    axis.grid(True)
    axis.legend(loc="best")

    axis.margins(x=0.01, y=0.15)

    add_phase_annotations(
        axis,
        adjusted_phases,
        plot_end_s=float(visible["plot_time_s"].max()),
    )

    # 각 구간의 중앙값을 터미널에 출력합니다.
    print("[VALID ALTITUDE SUMMARY]")
    for start_s, end_s, label in PHASES:
        actual_end_s = (
            float(altitude["elapsed_s"].max())
            if end_s is None
            else end_s
        )

        segment = altitude.loc[
            altitude["elapsed_s"].between(
                start_s,
                actual_end_s,
                inclusive="left",
            )
        ]

        if segment.empty:
            continue

        print(
            f"{label:12s} "
            f"{start_s:6.1f}–{actual_end_s:6.1f} s | "
            f"median={segment['altitude_m'].median():7.2f} m | "
            f"min={segment['altitude_m'].min():7.2f} m | "
            f"max={segment['altitude_m'].max():7.2f} m"
        )

    args.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )
    figure.tight_layout()
    figure.savefig(args.output, dpi=180)

    print(f"[SAVED] {args.output.resolve()}")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
