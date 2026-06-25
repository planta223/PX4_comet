#!/usr/bin/env python3
"""
export_bin_to_csv.py

Pixhawk 4 / ArduPilot DataFlash .BIN 로그에서 지정한 메시지 타입을
CSV 파일로 추출하는 스크립트입니다.

기본 추출 대상:
    GPS, IMU, BARO

특징:
    - pymavlink.tools.mavlogdump를 사용하지 않음
    - pymavlink.mavutil API로 .BIN 파일을 직접 읽음
    - 원본 .BIN 파일은 읽기만 하며 수정하지 않음

사용 예:
    python scripts/export_bin_to_csv.py LOGS/00000197.BIN

    python scripts/export_bin_to_csv.py LOGS/00000197.BIN --types GPS IMU BARO

    python scripts/export_bin_to_csv.py LOGS/00000197.BIN ^
        --types ATT AHR2 POS XKF1 XKF2 XKQ ^
        --out-dir output/csv_00000197_estimator
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any

from pymavlink import mavutil


# Raw / primary sensor logs.
# ATT, AHR2, POS, XKF1, XKF2, XKQ 등은 estimator output이므로
# 필요할 때 별도 옵션으로 추출한다.
DEFAULT_TYPES = ["GPS", "IMU", "BARO"]


def msg_to_dict(msg: Any) -> dict[str, Any]:
    """
    pymavlink 메시지 객체를 CSV 저장 가능한 dict로 변환한다.
    """
    if hasattr(msg, "to_dict"):
        data = msg.to_dict()
    else:
        data = {
            key: value
            for key, value in vars(msg).items()
            if not key.startswith("_")
        }

    # MAVLink message dict에는 'mavpackettype'이 들어갈 수 있다.
    # CSV에는 Type 컬럼으로 통일한다.
    msg_type = msg.get_type() if hasattr(msg, "get_type") else data.get("mavpackettype", "")

    cleaned: dict[str, Any] = {"Type": msg_type}

    for key, value in data.items():
        if key == "mavpackettype":
            continue
        cleaned[key] = value

    return cleaned


def open_writers(
    out_dir: Path,
    msg_types: list[str],
) -> tuple[dict[str, Any], dict[str, csv.DictWriter]]:
    """
    메시지 타입별 CSV 파일과 DictWriter를 생성한다.
    """
    files: dict[str, Any] = {}
    writers: dict[str, csv.DictWriter] = {}

    for msg_type in msg_types:
        csv_path = out_dir / f"{msg_type}.csv"
        f = csv_path.open("w", encoding="utf-8", newline="")
        files[msg_type] = f

        # header는 첫 메시지를 읽은 뒤 결정하므로 writer는 아직 만들지 않는다.
        writers[msg_type] = None  # type: ignore[assignment]

    return files, writers


def close_files(files: dict[str, Any]) -> None:
    for f in files.values():
        try:
            f.close()
        except Exception:
            pass


def export_bin_to_csv(
    bin_path: Path,
    out_dir: Path,
    msg_types: list[str],
) -> list[dict[str, Any]]:
    """
    .BIN 파일을 한 번 순회하면서 지정한 메시지 타입만 CSV로 저장한다.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    msg_type_set = set(msg_types)

    files, writers = open_writers(out_dir, msg_types)

    counts = {msg_type: 0 for msg_type in msg_types}
    sizes = {msg_type: 0 for msg_type in msg_types}
    headers = {msg_type: [] for msg_type in msg_types}

    try:
        print(f"[INFO] Opening BIN: {bin_path}")
        log = mavutil.mavlink_connection(str(bin_path))

        while True:
            msg = log.recv_match(blocking=False)

            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type not in msg_type_set:
                continue

            row = msg_to_dict(msg)

            if writers[msg_type] is None:
                fieldnames = list(row.keys())
                headers[msg_type] = fieldnames

                writer = csv.DictWriter(
                    files[msg_type],
                    fieldnames=fieldnames,
                    extrasaction="ignore",
                )
                writer.writeheader()
                writers[msg_type] = writer

            writers[msg_type].writerow(row)
            counts[msg_type] += 1

    finally:
        close_files(files)

    results: list[dict[str, Any]] = []

    for msg_type in msg_types:
        csv_path = out_dir / f"{msg_type}.csv"
        size = csv_path.stat().st_size if csv_path.exists() else 0
        sizes[msg_type] = size

        if counts[msg_type] == 0:
            status = "EMPTY"
        elif size == 0:
            status = "FAILED"
        else:
            status = "OK"

        results.append(
            {
                "type": msg_type,
                "csv_path": csv_path,
                "count": counts[msg_type],
                "size": size,
                "status": status,
                "headers": headers[msg_type],
            }
        )

    return results


def write_summary(
    summary_path: Path,
    bin_path: Path,
    out_dir: Path,
    results: list[dict[str, Any]],
) -> None:
    lines: list[str] = [
        "# BIN to CSV Export Summary",
        "",
        f"- Input BIN: `{bin_path}`",
        f"- Output directory: `{out_dir}`",
        "",
        "## Exported message types",
        "",
        "| Type | Output CSV | Rows | Size(bytes) | Status |",
        "| --- | --- | ---: | ---: | --- |",
    ]

    for result in results:
        lines.append(
            f"| {result['type']} | `{result['csv_path']}` | "
            f"{result['count']} | {result['size']} | {result['status']} |"
        )

    lines.extend(
        [
            "",
            "## CSV headers",
            "",
        ]
    )

    for result in results:
        lines.append(f"### {result['type']}")
        lines.append("")
        if result["headers"]:
            lines.append("```text")
            lines.append(", ".join(result["headers"]))
            lines.append("```")
        else:
            lines.append("No messages found.")
        lines.append("")

    summary_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export selected ArduPilot DataFlash BIN message types to CSV."
    )

    parser.add_argument(
        "bin_path",
        type=Path,
        help="Path to ArduPilot .BIN log file. Example: LOGS/00000197.BIN",
    )

    parser.add_argument(
        "--types",
        nargs="*",
        default=DEFAULT_TYPES,
        help=(
            "Message types to export. "
            "Default: GPS IMU BARO. "
            "Example: --types GPS IMU BARO ATT POS"
        ),
    )

    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help=(
            "Output directory. "
            "Default: output/csv_<BIN filename stem>. "
            "Example: output/csv_00000197"
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    bin_path: Path = args.bin_path

    if not bin_path.exists():
        print(f"[ERROR] BIN file not found: {bin_path}", file=sys.stderr)
        return 1

    if not bin_path.is_file():
        print(f"[ERROR] BIN path is not a file: {bin_path}", file=sys.stderr)
        return 1

    if bin_path.suffix.lower() != ".bin":
        print(f"[WARN] Input file extension is not .BIN: {bin_path}")

    out_dir: Path = args.out_dir or Path("output") / f"csv_{bin_path.stem}"
    msg_types: list[str] = args.types

    print(f"[INFO] Input BIN : {bin_path}")
    print(f"[INFO] Output dir: {out_dir}")
    print(f"[INFO] Types     : {', '.join(msg_types)}")

    try:
        results = export_bin_to_csv(
            bin_path=bin_path,
            out_dir=out_dir,
            msg_types=msg_types,
        )
    except ImportError as exc:
        print("[ERROR] pymavlink is not installed correctly.", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        print("Install with: pip install pymavlink", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"[ERROR] Failed to export BIN: {exc}", file=sys.stderr)
        return 1

    for result in results:
        print(
            f"[{result['status']}] {result['type']}: "
            f"{result['count']} rows, {result['size']} bytes"
        )

    summary_path = out_dir / "export_summary.md"
    write_summary(
        summary_path=summary_path,
        bin_path=bin_path,
        out_dir=out_dir,
        results=results,
    )

    print(f"[DONE] Summary written: {summary_path}")

    failed_count = sum(1 for result in results if result["status"] != "OK")

    if failed_count > 0:
        print(f"[WARN] {failed_count} message type(s) were not exported normally.")
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())