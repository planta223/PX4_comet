"""Direct ArduPilot DataFlash BIN parsing through pymavlink."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, DefaultDict, Dict, List, Set

import pandas as pd


TARGET_TYPES = {"XKF1", "XKQ", "XKF4", "ORGN", "MSG"}
CORE_FIELD_CANDIDATES = ("C", "Core", "core", "Instance", "I")


class PymavlinkUnavailable(RuntimeError):
    pass


@dataclass
class BinParseResult:
    xkf1: pd.DataFrame
    xkq: pd.DataFrame
    xkf4: pd.DataFrame
    orgn: pd.DataFrame
    messages: pd.DataFrame
    message_counts: Dict[str, int]
    available_types: List[str]
    fields_by_type: Dict[str, List[str]]
    parse_errors: List[str] = field(default_factory=list)
    bad_data_count: int = 0


def _message_dict(message: Any) -> Dict[str, Any]:
    try:
        values = dict(message.to_dict())
    except Exception:
        values = {name: getattr(message, name) for name in message.get_fieldnames()}
    values.pop("mavpackettype", None)
    return values


def _belongs_to_core(values: Dict[str, Any], core: int) -> bool:
    for field_name in CORE_FIELD_CANDIDATES:
        if field_name in values:
            try:
                return int(values[field_name]) == core
            except (TypeError, ValueError):
                return False
    # Some firmware message definitions have no core discriminator.
    return True


def read_dataflash_bin(path: Path, core: int = 0) -> BinParseResult:
    """Read a DataFlash BIN, retaining target messages for one EKF core.

    Parsing continues past BAD_DATA and individual decoding exceptions whenever
    pymavlink can advance to the next packet.
    """
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise PymavlinkUnavailable(
            "pymavlink가 설치되지 않았습니다. `python -m pip install -r requirements.txt`를 실행하세요."
        ) from exc

    bin_path = Path(path)
    if not bin_path.is_file():
        raise FileNotFoundError(f"BIN 파일을 찾을 수 없습니다: {bin_path}")

    rows: DefaultDict[str, List[Dict[str, Any]]] = defaultdict(list)
    counts: Counter[str] = Counter()
    available: Set[str] = set()
    fields: DefaultDict[str, Set[str]] = defaultdict(set)
    errors: List[str] = []
    bad_data_count = 0

    connection = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    consecutive_errors = 0
    while True:
        try:
            message = connection.recv_match(blocking=False)
        except Exception as exc:  # damaged records should not discard prior valid data
            consecutive_errors += 1
            if len(errors) < 20:
                errors.append(f"decode error: {type(exc).__name__}: {exc}")
            if consecutive_errors >= 100:
                errors.append("연속 decode error 100회로 읽기를 중단했습니다.")
                break
            continue
        if message is None:
            break
        consecutive_errors = 0
        message_type = str(message.get_type())
        if message_type == "BAD_DATA":
            bad_data_count += 1
            continue
        counts[message_type] += 1
        available.add(message_type)
        if message_type not in TARGET_TYPES:
            continue
        try:
            values = _message_dict(message)
        except Exception as exc:
            if len(errors) < 20:
                errors.append(f"{message_type} 변환 오류: {type(exc).__name__}: {exc}")
            continue
        fields[message_type].update(values.keys())
        if message_type in {"XKF1", "XKQ", "XKF4"} and not _belongs_to_core(values, core):
            continue
        rows[message_type].append(values)

    return BinParseResult(
        xkf1=pd.DataFrame(rows["XKF1"]),
        xkq=pd.DataFrame(rows["XKQ"]),
        xkf4=pd.DataFrame(rows["XKF4"]),
        orgn=pd.DataFrame(rows["ORGN"]),
        messages=pd.DataFrame(rows["MSG"]),
        message_counts=dict(counts),
        available_types=sorted(available),
        fields_by_type={name: sorted(names) for name, names in fields.items()},
        parse_errors=errors,
        bad_data_count=bad_data_count,
    )
