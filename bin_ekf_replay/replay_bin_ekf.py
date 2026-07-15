#!/usr/bin/env python3
"""CLI entry point for direct ArduPilot EKF3 DataFlash BIN replay."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd

from bin_parser import PymavlinkUnavailable, read_dataflash_bin
from state_model import StateBuildResult, build_states


def _positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("0보다 커야 합니다.")
    return number


def _nonnegative_float(value: str) -> float:
    number = float(value)
    if number < 0:
        raise argparse.ArgumentTypeError("0 이상이어야 합니다.")
    return number


def make_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ArduPilot DataFlash BIN의 EKF3 상태를 시간 정렬해 3D로 재생합니다."
    )
    parser.add_argument("bin_file", type=Path, help="입력 ArduPilot .BIN 파일")
    parser.add_argument("--core", type=int, default=0, help="EKF core 번호 (기본: 0)")
    parser.add_argument("--fps", type=_positive_float, default=30.0, help="화면 갱신 FPS (기본: 30)")
    parser.add_argument("--speed", type=_positive_float, default=1.0, help="초기 재생 배속 (기본: 1.0)")
    parser.add_argument("--trail-seconds", type=_nonnegative_float, default=30.0, help="현재 궤적 길이, 초 (기본: 30)")
    parser.add_argument("--match-tolerance", type=_nonnegative_float, default=0.05, help="XKQ/XKF4 nearest 매칭 허용 오차, 초 (기본: 0.05)")
    parser.add_argument("--save-csv", type=Path, help="시간 정렬된 상태 CSV 출력 경로")
    parser.add_argument("--no-health", action="store_true", help="XKF4 health panel 숨기기")
    parser.add_argument("--use-quaternion-conjugate", action="store_true", help="자세 표시에 XKQ quaternion conjugate 사용")
    parser.add_argument("--summary-only", action="store_true", help="애니메이션 없이 요약만 출력")
    return parser


def _finite_range(frame: pd.DataFrame, columns: Iterable[str]) -> str:
    parts = []
    for column in columns:
        values = pd.to_numeric(frame[column], errors="coerce").to_numpy(float)
        finite = values[np.isfinite(values)]
        parts.append(f"{column}=[{finite.min():.3f}, {finite.max():.3f}]" if len(finite) else f"{column}=n/a")
    return ", ".join(parts)


def _status_statistics(states: pd.DataFrame) -> str:
    lines = []
    for column in ("fault_status", "timeout_status", "solution_status"):
        values = pd.to_numeric(states[column], errors="coerce").dropna()
        if values.empty:
            lines.append(f"  {column}: n/a")
            continue
        integer_values = values.astype(np.int64)
        counts = integer_values.value_counts().head(8)
        formatted = ", ".join(f"0x{value:X}:{count}" for value, count in counts.items())
        nonzero = int((integer_values != 0).sum())
        lines.append(f"  {column}: nonzero={nonzero}/{len(integer_values)}, top {{{formatted}}}")
    for column in ("vel_test_ratio", "pos_test_ratio", "height_test_ratio", "mag_test_ratio"):
        values = pd.to_numeric(states[column], errors="coerce").dropna()
        if values.empty:
            lines.append(f"  {column}: n/a")
        else:
            lines.append(f"  {column}: mean={values.mean():.3f}, max={values.max():.3f}, >1={int((values > 1).sum())}/{len(values)}")
    return "\n".join(lines)


def print_summary(parse_result, build_result: StateBuildResult, bin_path: Path, core: int) -> None:
    states = build_result.states
    counts = ", ".join(f"{name}={parse_result.message_counts.get(name, 0)}" for name in ("XKF1", "XKQ", "XKF4", "ORGN", "MSG"))
    print(f"BIN: {bin_path}")
    print(f"EKF core: {core}")
    print(f"메시지 개수(전체 core): {counts}")
    print(f"선택 core 보존 샘플: XKF1={len(parse_result.xkf1)}, XKQ={len(parse_result.xkq)}, XKF4={len(parse_result.xkf4)}")
    print(f"정렬 상태 샘플: {len(states)}")
    print(f"시간 범위: {states['time_s'].min():.3f} .. {states['time_s'].max():.3f} s (duration {states['time_s'].max() - states['time_s'].min():.3f} s)")
    print("위치 NED: " + _finite_range(states, ("pn_m", "pe_m", "pd_m")))
    print("속도 NED: " + _finite_range(states, ("vn_mps", "ve_mps", "vd_mps")))
    print(f"quaternion 원본 유효 매칭 비율: {build_result.quaternion_valid_ratio:.2%}")
    print(f"quaternion 자세 소스: {build_result.quaternion_source}")
    print("EKF status 통계:")
    print(_status_statistics(states))
    print("실제 대상 메시지 필드:")
    for name in ("XKF1", "XKQ", "XKF4", "ORGN", "MSG"):
        print(f"  {name}: {parse_result.fields_by_type.get(name, [])}")
    if parse_result.bad_data_count:
        print(f"BAD_DATA 건수: {parse_result.bad_data_count}")
    if parse_result.parse_errors:
        print("파싱 경고:")
        for warning in parse_result.parse_errors:
            print(f"  - {warning}")
    if build_result.warnings:
        print("정렬/필드 경고:")
        for warning in build_result.warnings:
            print(f"  - {warning}")


def save_csv(states: pd.DataFrame, path: Path) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    states.to_csv(output, index=False)
    print(f"CSV 저장: {output} ({len(states)} rows, {len(states.columns)} columns)")


def main(argv=None) -> int:
    args = make_argument_parser().parse_args(argv)
    try:
        parsed = read_dataflash_bin(args.bin_file, core=args.core)
    except (FileNotFoundError, PymavlinkUnavailable) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"오류: BIN을 열거나 읽지 못했습니다: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    if parsed.xkf1.empty:
        print(f"오류: 선택한 core {args.core}의 XKF1 메시지가 없습니다.", file=sys.stderr)
        print("사용 가능한 메시지 종류: " + (", ".join(parsed.available_types) or "없음"), file=sys.stderr)
        print("XKF1 실제 필드: " + str(parsed.fields_by_type.get("XKF1", [])), file=sys.stderr)
        return 3

    try:
        built = build_states(parsed.xkf1, parsed.xkq, parsed.xkf4, args.match_tolerance)
    except ValueError as exc:
        print(f"오류: {exc}", file=sys.stderr)
        print("XKF1 실제 필드: " + str(parsed.fields_by_type.get("XKF1", [])), file=sys.stderr)
        return 3

    if parsed.xkq.empty:
        print("경고: XKQ가 없어 XKF1 Euler angle 기반 quaternion으로 자세를 표시합니다.", file=sys.stderr)
    if parsed.xkf4.empty and not args.no_health:
        print("경고: XKF4가 없어 health panel을 비활성화합니다.", file=sys.stderr)

    print_summary(parsed, built, args.bin_file, args.core)
    if args.save_csv:
        save_csv(built.states, args.save_csv)
    if args.summary_only:
        return 0

    try:
        from visualizer import replay_states
        replay_states(
            built.states,
            fps=args.fps,
            speed=args.speed,
            trail_seconds=args.trail_seconds,
            show_health=not args.no_health and not parsed.xkf4.empty,
            use_quaternion_conjugate=args.use_quaternion_conjugate,
        )
    except ImportError as exc:
        print(f"오류: 시각화 의존성을 불러오지 못했습니다: {exc}. `python -m pip install -r requirements.txt`를 실행하세요.", file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
