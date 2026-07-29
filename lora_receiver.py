#!/usr/bin/env python3
"""Receive and log 34-byte COMET telemetry packets from the LoRa UART."""

from __future__ import annotations

import argparse
import csv
import json
import statistics as statistics_math
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import IO, Callable, Iterable, Optional


TELEMETRY_HEADER = 0xA5
TELEMETRY_LENGTH = 34
STATE_NAMES = {
    0: "READY",
    1: "ASCENT",
    2: "DEPLOY",
    3: "DESCENT",
    4: "LANDED",
    5: "ERROR",
}
STATUS_BITS = {
    0x01: "POSITION",
    0x02: "VELOCITY",
    0x04: "ATTITUDE",
    0x08: "ACCELERATION",
}
FAULT_BITS = {
    0x01: "ACCEL_SENSOR_INVALID",
    0x02: "POSITION_INVALID",
    0x04: "VELOCITY_INVALID",
    0x08: "ASCENT_TIMEOUT",
    0x10: "DEPLOY_COMMAND_FAILED",
    0x20: "LORA_UNAVAILABLE",
}
CSV_FIELDS = [
    "timestamp_iso",
    "packet_type",
    "sequence",
    "state_id",
    "state_name",
    "status_flags",
    "status_names",
    "fault_flags",
    "fault_names",
    "time_ms",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "velocity_n_mps",
    "velocity_e_mps",
    "velocity_d_mps",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "raw_hex",
    "pc_receive_time",
    "pc_elapsed_s",
    "sequence_delta",
    "missed_since_previous",
    "duplicate",
    "out_of_order",
    "interval_ms",
    "expected_period_ms",
    "jitter_ms",
    "received_count",
    "missed_count",
    "duplicate_count",
    "out_of_order_count",
    "loss_rate_percent",
]


@dataclass(slots=True)
class PacketObservation:
    """Timing and sequence result for one CRC-valid telemetry packet."""

    pc_receive_time: datetime
    pc_receive_monotonic: float
    pc_elapsed_s: float
    previous_sequence: Optional[int]
    sequence_delta: Optional[int]
    missed_since_previous: int
    duplicate: bool
    out_of_order: bool
    interval_ms: Optional[float]
    expected_period_ms: Optional[float]
    jitter_ms: Optional[float]
    received_count: int
    missed_count: int
    duplicate_count: int
    out_of_order_count: int
    loss_rate_percent: float


@dataclass(slots=True)
class Statistics:
    """Framing, sequence, timing, loss, and throughput statistics."""

    expected_period_ms: Optional[float] = None
    packet_length_bytes: int = TELEMETRY_LENGTH

    total_serial_bytes: int = 0
    crc_errors: int = 0
    discarded_resync_bytes: int = 0
    header_candidates: int = 0

    received_count: int = 0
    missed_count: int = 0
    duplicate_count: int = 0
    out_of_order_count: int = 0

    first_receive_time: Optional[datetime] = None
    last_receive_time: Optional[datetime] = None
    intervals_ms: list[float] = field(default_factory=list)

    _first_receive_monotonic: Optional[float] = None
    _last_receive_monotonic: Optional[float] = None
    _last_sequence: Optional[int] = None

    @property
    def total_valid_packets(self) -> int:
        """Backward-compatible name for the CRC-valid packet count."""
        return self.received_count

    @property
    def estimated_lost_packets(self) -> int:
        """Backward-compatible name for the sequence-derived missed count."""
        return self.missed_count

    @property
    def duplicate_packets(self) -> int:
        """Backward-compatible name for duplicate packet count."""
        return self.duplicate_count

    @property
    def total_expected_count(self) -> int:
        """Expected unique forward packets, including sequence gaps."""
        forward_received = (
            self.received_count
            - self.duplicate_count
            - self.out_of_order_count
        )
        return max(0, forward_received + self.missed_count)

    @property
    def loss_rate_percent(self) -> float:
        total = self.total_expected_count
        if total == 0:
            return 0.0
        return self.missed_count * 100.0 / total

    @property
    def average_interval_ms(self) -> float:
        if not self.intervals_ms:
            return 0.0
        return statistics_math.fmean(self.intervals_ms)

    @property
    def minimum_interval_ms(self) -> float:
        return min(self.intervals_ms, default=0.0)

    @property
    def maximum_interval_ms(self) -> float:
        return max(self.intervals_ms, default=0.0)

    @property
    def interval_stddev_ms(self) -> float:
        if len(self.intervals_ms) < 2:
            return 0.0
        return statistics_math.pstdev(self.intervals_ms)

    def duration_s(self, end_monotonic: Optional[float] = None) -> float:
        """Return elapsed test time from the first valid packet."""
        if self._first_receive_monotonic is None:
            return 0.0
        if end_monotonic is None:
            end_monotonic = self._last_receive_monotonic
        if end_monotonic is None:
            return 0.0
        return max(0.0, end_monotonic - self._first_receive_monotonic)

    def record_packet(
        self,
        sequence: int,
        receive_time: datetime,
        receive_monotonic: float,
    ) -> PacketObservation:
        """Record one CRC-valid packet using wrap-aware sequence rules."""
        previous_sequence = self._last_sequence
        sequence_delta: Optional[int] = None
        missed_since_previous = 0
        duplicate = False
        out_of_order = False

        if previous_sequence is None:
            self._last_sequence = sequence
        else:
            sequence_delta = (sequence - previous_sequence) & 0xFF
            if sequence_delta == 0:
                duplicate = True
                self.duplicate_count += 1
            elif sequence_delta <= 127:
                missed_since_previous = sequence_delta - 1
                self.missed_count += missed_since_previous
                self._last_sequence = sequence
            else:
                out_of_order = True
                self.out_of_order_count += 1

        interval_ms: Optional[float] = None
        if self._last_receive_monotonic is not None:
            interval_ms = (
                receive_monotonic - self._last_receive_monotonic
            ) * 1000.0
            self.intervals_ms.append(interval_ms)

        if self._first_receive_monotonic is None:
            self._first_receive_monotonic = receive_monotonic
            self.first_receive_time = receive_time

        self._last_receive_monotonic = receive_monotonic
        self.last_receive_time = receive_time
        self.received_count += 1

        jitter_ms: Optional[float] = None
        if interval_ms is not None and self.expected_period_ms is not None:
            jitter_ms = abs(interval_ms - self.expected_period_ms)

        return PacketObservation(
            pc_receive_time=receive_time,
            pc_receive_monotonic=receive_monotonic,
            pc_elapsed_s=self.duration_s(receive_monotonic),
            previous_sequence=previous_sequence,
            sequence_delta=sequence_delta,
            missed_since_previous=missed_since_previous,
            duplicate=duplicate,
            out_of_order=out_of_order,
            interval_ms=interval_ms,
            expected_period_ms=self.expected_period_ms,
            jitter_ms=jitter_ms,
            received_count=self.received_count,
            missed_count=self.missed_count,
            duplicate_count=self.duplicate_count,
            out_of_order_count=self.out_of_order_count,
            loss_rate_percent=self.loss_rate_percent,
        )

    def summary(self, end_monotonic: Optional[float] = None) -> dict[str, object]:
        """Build the final JSON-compatible test summary."""
        duration_s = self.duration_s(end_monotonic)
        average_packet_rate_hz = (
            self.received_count / duration_s if duration_s > 0.0 else 0.0
        )
        average_payload_rate_bps = (
            self.received_count * self.packet_length_bytes / duration_s
            if duration_s > 0.0
            else 0.0
        )
        estimated_required_payload_rate_bps = (
            self.packet_length_bytes * 1000.0 / self.expected_period_ms
            if self.expected_period_ms is not None
            else 0.0
        )
        return {
            "expected_period_ms": self.expected_period_ms,
            "packet_length_bytes": self.packet_length_bytes,
            "received_count": self.received_count,
            "missed_count": self.missed_count,
            "duplicate_count": self.duplicate_count,
            "out_of_order_count": self.out_of_order_count,
            "total_expected_count": self.total_expected_count,
            "loss_rate_percent": self.loss_rate_percent,
            "first_receive_time": (
                self.first_receive_time.isoformat()
                if self.first_receive_time is not None
                else None
            ),
            "last_receive_time": (
                self.last_receive_time.isoformat()
                if self.last_receive_time is not None
                else None
            ),
            "duration_s": duration_s,
            "average_interval_ms": self.average_interval_ms,
            "minimum_interval_ms": self.minimum_interval_ms,
            "maximum_interval_ms": self.maximum_interval_ms,
            "interval_stddev_ms": self.interval_stddev_ms,
            "average_packet_rate_hz": average_packet_rate_hz,
            "average_payload_rate_Bps": average_payload_rate_bps,
            "estimated_required_payload_rate_Bps": (
                estimated_required_payload_rate_bps
            ),
            "crc_errors": self.crc_errors,
            "total_serial_bytes": self.total_serial_bytes,
            "discarded_resync_bytes": self.discarded_resync_bytes,
        }


@dataclass(slots=True)
class TelemetryPacket:
    """Decoded COMET telemetry packet."""

    timestamp: datetime
    sequence: int
    state_id: int
    status_flags: int
    fault_flags: int
    time_ms: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    velocity_n_mps: float
    velocity_e_mps: float
    velocity_d_mps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    raw_hex: str
    observation: Optional[PacketObservation] = None

    @property
    def state_name(self) -> str:
        """Return a readable flight-state name."""
        return STATE_NAMES.get(self.state_id, f"UNKNOWN_{self.state_id}")

    @property
    def status_names(self) -> str:
        """Return a pipe-separated list of valid sensor groups."""
        names = [name for bit, name in STATUS_BITS.items() if self.status_flags & bit]
        return "|".join(names) if names else "NONE"

    @property
    def fault_names(self) -> str:
        """Return a pipe-separated list of active flight faults."""
        names = [name for bit, name in FAULT_BITS.items() if self.fault_flags & bit]
        return "|".join(names) if names else "NONE"

    def csv_row(self) -> dict[str, object]:
        """Return one row matching the fixed CSV schema."""
        row: dict[str, object] = {
            "timestamp_iso": self.timestamp.isoformat(timespec="milliseconds"),
            "packet_type": "TELEMETRY",
            "sequence": self.sequence,
            "state_id": self.state_id,
            "state_name": self.state_name,
            "status_flags": f"0x{self.status_flags:02X}",
            "status_names": self.status_names,
            "fault_flags": f"0x{self.fault_flags:02X}",
            "fault_names": self.fault_names,
            "time_ms": self.time_ms,
            "latitude_deg": self.latitude_deg,
            "longitude_deg": self.longitude_deg,
            "altitude_m": self.altitude_m,
            "velocity_n_mps": self.velocity_n_mps,
            "velocity_e_mps": self.velocity_e_mps,
            "velocity_d_mps": self.velocity_d_mps,
            "roll_deg": self.roll_deg,
            "pitch_deg": self.pitch_deg,
            "yaw_deg": self.yaw_deg,
            "raw_hex": self.raw_hex,
        }
        observation = self.observation
        row.update(
            {
                "pc_receive_time": (
                    observation.pc_receive_time.isoformat(timespec="milliseconds")
                    if observation is not None
                    else ""
                ),
                "pc_elapsed_s": (
                    observation.pc_elapsed_s if observation is not None else ""
                ),
                "sequence_delta": (
                    observation.sequence_delta
                    if observation is not None
                    and observation.sequence_delta is not None
                    else ""
                ),
                "missed_since_previous": (
                    observation.missed_since_previous
                    if observation is not None
                    else ""
                ),
                "duplicate": (
                    int(observation.duplicate) if observation is not None else ""
                ),
                "out_of_order": (
                    int(observation.out_of_order)
                    if observation is not None
                    else ""
                ),
                "interval_ms": (
                    observation.interval_ms
                    if observation is not None
                    and observation.interval_ms is not None
                    else ""
                ),
                "expected_period_ms": (
                    observation.expected_period_ms
                    if observation is not None
                    and observation.expected_period_ms is not None
                    else ""
                ),
                "jitter_ms": (
                    observation.jitter_ms
                    if observation is not None
                    and observation.jitter_ms is not None
                    else ""
                ),
                "received_count": (
                    observation.received_count if observation is not None else ""
                ),
                "missed_count": (
                    observation.missed_count if observation is not None else ""
                ),
                "duplicate_count": (
                    observation.duplicate_count if observation is not None else ""
                ),
                "out_of_order_count": (
                    observation.out_of_order_count
                    if observation is not None
                    else ""
                ),
                "loss_rate_percent": (
                    observation.loss_rate_percent
                    if observation is not None
                    else ""
                ),
            }
        )
        return row


class StreamParser:
    """Incrementally recover fixed-length COMET packets from a UART stream."""

    def __init__(
        self,
        statistics: Optional[Statistics] = None,
        packet_length: int = TELEMETRY_LENGTH,
        monotonic_clock: Callable[[], float] = time.monotonic,
        wall_clock: Optional[Callable[[], datetime]] = None,
    ) -> None:
        self.buffer = bytearray()
        self.statistics = statistics if statistics is not None else Statistics()
        self.packet_length = packet_length
        self._monotonic_clock = monotonic_clock
        self._wall_clock = wall_clock or (lambda: datetime.now().astimezone())

    def feed(self, data: bytes) -> list[TelemetryPacket]:
        """Append a UART chunk and return every complete, CRC-valid packet."""
        if data:
            self.statistics.total_serial_bytes += len(data)
            self.buffer.extend(data)

        packets: list[TelemetryPacket] = []
        while True:
            header_index = self.buffer.find(TELEMETRY_HEADER)
            if header_index < 0:
                self._discard(len(self.buffer))
                break

            if header_index > 0:
                self._discard(header_index)

            if len(self.buffer) < self.packet_length:
                break

            self.statistics.header_candidates += 1
            candidate = bytes(self.buffer[: self.packet_length])
            expected_crc = crc8_atm(candidate[:-1])
            if candidate[-1] != expected_crc:
                self.statistics.crc_errors += 1
                # Keep searching inside the failed candidate for the next header.
                self._discard(1)
                continue

            del self.buffer[: self.packet_length]
            receive_monotonic = self._monotonic_clock()
            receive_time = self._wall_clock()
            packet = decode_packet(candidate, receive_time)
            packet.observation = self.statistics.record_packet(
                packet.sequence,
                receive_time,
                receive_monotonic,
            )
            packets.append(packet)

        return packets

    def _discard(self, count: int) -> None:
        if count <= 0:
            return
        del self.buffer[:count]
        self.statistics.discarded_resync_bytes += count

class CsvLogger:
    """Append telemetry to CSV and flush after every packet."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._file: Optional[IO[str]] = None
        self._writer: Optional[csv.DictWriter] = None

    def open(self) -> None:
        csv_file = self.path.open("a+", newline="", encoding="utf-8")
        try:
            csv_file.seek(0)
            existing_header = next(csv.reader(csv_file), None)
            if existing_header is not None and existing_header != CSV_FIELDS:
                raise ValueError(
                    "existing CSV header does not match the current schema"
                )
            csv_file.seek(0, 2)
            is_empty = existing_header is None
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            if is_empty:
                writer.writeheader()
                csv_file.flush()
        except Exception:
            csv_file.close()
            raise

        self._file = csv_file
        self._writer = writer

    def write(self, packet: TelemetryPacket) -> None:
        if self._file is None or self._writer is None:
            raise RuntimeError("CSV logger is not open")
        self._writer.writerow(packet.csv_row())
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None


def crc8_atm(values: Iterable[int]) -> int:
    """Calculate CRC-8/ATM with polynomial 0x07 and initial value 0x00."""
    crc = 0
    for value in values:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def decode_packet(
    raw: bytes,
    receive_time: Optional[datetime] = None,
) -> TelemetryPacket:
    """Decode the exact layout emitted by comet_lora.lua."""
    (
        header,
        sequence,
        state_id,
        status_flags,
        fault_flags,
        time_ms,
        latitude_raw,
        longitude_raw,
        altitude_raw,
        velocity_n_raw,
        velocity_e_raw,
        velocity_d_raw,
        roll_raw,
        pitch_raw,
        yaw_raw,
    ) = struct.unpack_from("<BBBBBIiiihhhhhh", raw)

    if header != TELEMETRY_HEADER:
        raise ValueError(f"unexpected telemetry header: 0x{header:02X}")

    if receive_time is None:
        receive_time = datetime.now().astimezone()

    return TelemetryPacket(
        timestamp=receive_time,
        sequence=sequence,
        state_id=state_id,
        status_flags=status_flags,
        fault_flags=fault_flags,
        time_ms=time_ms,
        latitude_deg=latitude_raw / 10_000_000.0,
        longitude_deg=longitude_raw / 10_000_000.0,
        altitude_m=altitude_raw / 100.0,
        velocity_n_mps=velocity_n_raw / 10.0,
        velocity_e_mps=velocity_e_raw / 10.0,
        velocity_d_mps=velocity_d_raw / 10.0,
        roll_deg=roll_raw / 100.0,
        pitch_deg=pitch_raw / 100.0,
        yaw_deg=yaw_raw / 100.0,
        raw_hex=raw.hex(" ").upper(),
    )


def format_packet(packet: TelemetryPacket, show_raw_hex: bool) -> str:
    """Format one decoded telemetry packet for the console."""
    timestamp = packet.timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    line = (
        f"[{timestamp}] TELEMETRY seq={packet.sequence} state={packet.state_name} "
        f"status=0x{packet.status_flags:02X}({packet.status_names}) "
        f"faults=0x{packet.fault_flags:02X}({packet.fault_names}) "
        f"lat={packet.latitude_deg:.7f} lon={packet.longitude_deg:.7f} "
        f"alt={packet.altitude_m:.2f}m "
        f"velNED=({packet.velocity_n_mps:.1f},{packet.velocity_e_mps:.1f},"
        f"{packet.velocity_d_mps:.1f})m/s "
        f"rpy=({packet.roll_deg:.2f},{packet.pitch_deg:.2f},"
        f"{packet.yaw_deg:.2f})deg uptime={packet.time_ms}ms"
    )
    observation = packet.observation
    if observation is not None and observation.interval_ms is not None:
        line += f" interval={observation.interval_ms:.1f}ms"
        if observation.jitter_ms is not None:
            line += f" jitter={observation.jitter_ms:.1f}ms"
    if show_raw_hex:
        line += f" raw={packet.raw_hex}"
    return line


def format_stream_chunk(chunk: bytes) -> str:
    """Format unfiltered UART data for electrical/baud diagnostics."""
    timestamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    return f"[{timestamp}] RX bytes={len(chunk)} hex={chunk.hex(' ').upper()}"


def print_sequence_warning(observation: PacketObservation, sequence: int) -> None:
    """Print an immediate warning for a sequence anomaly."""
    previous = observation.previous_sequence
    if observation.missed_since_previous > 0:
        print(
            f"[LOSS] last={previous} current={sequence} "
            f"missed={observation.missed_since_previous}",
            flush=True,
        )
    elif observation.duplicate:
        print(f"[DUP] sequence={sequence}", flush=True)
    elif observation.out_of_order:
        print(
            f"[ORDER] last={previous} current={sequence}",
            flush=True,
        )


def format_periodic_statistics(
    statistics: Statistics,
    end_monotonic: Optional[float] = None,
) -> str:
    """Format one compact cumulative statistics line."""
    summary = statistics.summary(end_monotonic)
    return (
        f"[STATS] rx={summary['received_count']} "
        f"missed={summary['missed_count']} "
        f"duplicate={summary['duplicate_count']} "
        f"out_of_order={summary['out_of_order_count']} "
        f"loss={summary['loss_rate_percent']:.2f}% "
        f"avg_interval={summary['average_interval_ms']:.1f}ms"
    )


def print_test_summary(summary: dict[str, object]) -> None:
    """Print the final LoRa test summary."""
    expected_period = summary["expected_period_ms"]
    expected_text = (
        f"{expected_period:g} ms"
        if isinstance(expected_period, (int, float))
        else "not specified"
    )
    print("\n=== LoRa Test Summary ===")
    print(f"Expected period: {expected_text}")
    print(f"Packet length: {summary['packet_length_bytes']} bytes")
    print(f"Received: {summary['received_count']}")
    print(f"Missed: {summary['missed_count']}")
    print(f"Total expected: {summary['total_expected_count']}")
    print(f"Duplicates: {summary['duplicate_count']}")
    print(f"Out of order: {summary['out_of_order_count']}")
    print(f"Loss rate: {summary['loss_rate_percent']:.2f} %")
    print(f"Duration: {summary['duration_s']:.3f} s")
    print(f"Average interval: {summary['average_interval_ms']:.1f} ms")
    print(f"Min interval: {summary['minimum_interval_ms']:.1f} ms")
    print(f"Max interval: {summary['maximum_interval_ms']:.1f} ms")
    print(f"Stddev: {summary['interval_stddev_ms']:.1f} ms")
    print(f"Average packet rate: {summary['average_packet_rate_hz']:.3f} packet/s")
    print(f"Average payload rate: {summary['average_payload_rate_Bps']:.1f} B/s")
    print(
        "Estimated required payload rate: "
        f"{summary['estimated_required_payload_rate_Bps']:.1f} B/s"
    )
    print(f"CRC errors: {summary['crc_errors']}")
    print(f"Total serial bytes: {summary['total_serial_bytes']}")


def write_summary_json(path: Path, summary: dict[str, object]) -> None:
    """Write one UTF-8 JSON summary."""
    with path.open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, ensure_ascii=False, indent=2)
        summary_file.write("\n")


def receive_packets(args: argparse.Namespace) -> int:
    """Open the serial port and receive until Ctrl+C or disconnection."""
    try:
        import serial
    except ImportError:
        print("Error: install pyserial with: pip install pyserial", file=sys.stderr)
        return 2

    statistics = Statistics(
        expected_period_ms=args.expected_period_ms,
        packet_length_bytes=args.packet_length,
    )
    parser = StreamParser(statistics, packet_length=args.packet_length)
    csv_logger: Optional[CsvLogger] = None
    summary_path = (
        Path(args.summary_json)
        if args.summary_json is not None
        else Path(
            "lora_test_summary_"
            + datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
            + ".json"
        )
    )

    if not args.no_csv:
        csv_logger = CsvLogger(Path(args.csv))
        try:
            csv_logger.open()
        except (OSError, ValueError, csv.Error) as exc:
            print(f"CSV open error for {args.csv!r}: {exc}", file=sys.stderr)
            return 2

    exit_code = 0
    last_stats_report_monotonic = time.monotonic()
    try:
        try:
            with serial.Serial(
                port=args.port,
                baudrate=args.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.25,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            ) as ser:
                print(
                    f"Listening on {args.port} at {args.baud} baud "
                    "(8N1, no flow control). Press Ctrl+C to stop."
                )
                if args.expected_period_ms is not None:
                    required_rate = (
                        args.packet_length
                        * 1000.0
                        / args.expected_period_ms
                    )
                    print(
                        f"Test target: {args.packet_length} bytes / "
                        f"{args.expected_period_ms:g} ms = "
                        f"{required_rate:.1f} B/s"
                    )
                warning_printed = False
                while True:
                    chunk = ser.read(256)
                    if chunk and args.debug_stream:
                        print(format_stream_chunk(chunk), flush=True)

                    packets = parser.feed(chunk)
                    for packet in packets:
                        print(format_packet(packet, args.raw_hex), flush=True)
                        if packet.observation is not None:
                            print_sequence_warning(
                                packet.observation,
                                packet.sequence,
                            )
                        if csv_logger is not None:
                            try:
                                csv_logger.write(packet)
                            except (OSError, csv.Error) as exc:
                                print(
                                    f"CSV write failed; logging disabled: {exc}",
                                    file=sys.stderr,
                                    flush=True,
                                )
                                try:
                                    csv_logger.close()
                                except OSError:
                                    pass
                                csv_logger = None

                    now_monotonic = time.monotonic()
                    if (
                        now_monotonic - last_stats_report_monotonic
                        >= args.stats_interval_s
                    ):
                        print(
                            format_periodic_statistics(
                                statistics,
                                now_monotonic,
                            ),
                            flush=True,
                        )
                        last_stats_report_monotonic = now_monotonic

                    if (
                        not warning_printed
                        and not args.debug_stream
                        and statistics.total_valid_packets == 0
                        and statistics.discarded_resync_bytes >= 256
                    ):
                        print(
                            "Warning: UART bytes are arriving but no valid COMET "
                            "packet was found. Add --debug-stream to inspect them.",
                            file=sys.stderr,
                            flush=True,
                        )
                        warning_printed = True
        except KeyboardInterrupt:
            print("\nCtrl+C received; shutting down.")
        except (serial.SerialException, OSError) as exc:
            print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
            exit_code = 1
    finally:
        end_monotonic = time.monotonic()
        if csv_logger is not None:
            try:
                csv_logger.close()
            except OSError as exc:
                print(f"CSV close error: {exc}", file=sys.stderr)
                exit_code = 1

        summary = statistics.summary(end_monotonic)
        print_test_summary(summary)
        try:
            write_summary_json(summary_path, summary)
            print(f"Summary JSON: {summary_path}")
        except (OSError, TypeError) as exc:
            print(
                f"Summary JSON write error for {str(summary_path)!r}: {exc}",
                file=sys.stderr,
            )
            exit_code = 1

    return exit_code


def build_test_packet(
    sequence: int,
    state_id: int,
    status_flags: int,
    fault_flags: int,
    time_ms: int,
    latitude_raw: int,
    longitude_raw: int,
    altitude_raw: int,
    velocity_n_raw: int,
    velocity_e_raw: int,
    velocity_d_raw: int,
    roll_raw: int,
    pitch_raw: int,
    yaw_raw: int,
) -> bytes:
    """Build one Lua-compatible packet for the built-in self-test."""
    payload = struct.pack(
        "<BBBBBIiiihhhhhh",
        TELEMETRY_HEADER,
        sequence,
        state_id,
        status_flags,
        fault_flags,
        time_ms,
        latitude_raw,
        longitude_raw,
        altitude_raw,
        velocity_n_raw,
        velocity_e_raw,
        velocity_d_raw,
        roll_raw,
        pitch_raw,
        yaw_raw,
    )
    return payload + bytes([crc8_atm(payload)])


def run_self_test() -> int:
    """Test decoding, CRC recovery, sequence rules, timing, CSV, and summary."""
    first = build_test_packet(
        sequence=254,
        state_id=1,
        status_flags=0x0F,
        fault_flags=0x20,
        time_ms=123_456,
        latitude_raw=-371_234_567,
        longitude_raw=1_271_234_567,
        altitude_raw=-1234,
        velocity_n_raw=-12,
        velocity_e_raw=34,
        velocity_d_raw=-56,
        roll_raw=-789,
        pitch_raw=1234,
        yaw_raw=-17999,
    )
    bad = bytearray(first)
    bad[-1] ^= 0x80
    second = build_test_packet(
        sequence=0,
        state_id=3,
        status_flags=0x07,
        fault_flags=0x00,
        time_ms=124_000,
        latitude_raw=371_234_567,
        longitude_raw=-1_271_234_567,
        altitude_raw=9876,
        velocity_n_raw=10,
        velocity_e_raw=-20,
        velocity_d_raw=30,
        roll_raw=100,
        pitch_raw=-200,
        yaw_raw=300,
    )

    stats = Statistics()
    parser = StreamParser(stats)
    packets: list[TelemetryPacket] = []
    stream = b"\x00\xE0\x7E" + first
    for start, end in ((0, 1), (1, 5), (5, 14), (14, 29), (29, len(stream))):
        packets.extend(parser.feed(stream[start:end]))
    packets.extend(parser.feed(bytes(bad) + second))

    assert len(first) == TELEMETRY_LENGTH
    assert len(packets) == 2
    assert packets[0].sequence == 254
    assert packets[0].state_name == "ASCENT"
    assert packets[0].fault_flags == 0x20
    assert packets[0].fault_names == "LORA_UNAVAILABLE"
    assert packets[0].latitude_deg == -37.1234567
    assert packets[0].longitude_deg == 127.1234567
    assert packets[0].altitude_m == -12.34
    assert packets[0].velocity_n_mps == -1.2
    assert packets[0].roll_deg == -7.89
    assert packets[0].yaw_deg == -179.99
    assert packets[1].longitude_deg == -127.1234567
    assert packets[1].pitch_deg == -2.0
    assert stats.total_valid_packets == 2
    assert stats.crc_errors == 1
    assert stats.estimated_lost_packets == 1
    assert stats.discarded_resync_bytes >= 4

    parser.feed(second)
    assert stats.duplicate_packets == 1

    test_time = datetime.now().astimezone()

    loss_stats = Statistics(expected_period_ms=500.0)
    first_observation = loss_stats.record_packet(10, test_time, 10.0)
    loss_observation = loss_stats.record_packet(13, test_time, 10.5)
    assert first_observation.sequence_delta is None
    assert loss_observation.sequence_delta == 3
    assert loss_observation.missed_since_previous == 2
    assert loss_observation.interval_ms == 500.0
    assert loss_observation.jitter_ms == 0.0
    assert loss_stats.missed_count == 2

    duplicate_observation = loss_stats.record_packet(13, test_time, 10.75)
    assert duplicate_observation.duplicate
    assert loss_stats.duplicate_count == 1

    wrap_stats = Statistics()
    wrap_stats.record_packet(255, test_time, 20.0)
    wrap_observation = wrap_stats.record_packet(0, test_time, 20.5)
    assert wrap_observation.sequence_delta == 1
    assert wrap_stats.missed_count == 0

    duplicate_stats = Statistics()
    duplicate_stats.record_packet(20, test_time, 30.0)
    duplicate_observation = duplicate_stats.record_packet(
        20,
        test_time,
        30.5,
    )
    assert duplicate_observation.duplicate
    assert duplicate_stats.duplicate_count == 1

    order_stats = Statistics()
    order_stats.record_packet(100, test_time, 40.0)
    order_observation = order_stats.record_packet(98, test_time, 40.5)
    assert order_observation.out_of_order
    assert order_stats.out_of_order_count == 1
    assert order_stats.missed_count == 0
    next_observation = order_stats.record_packet(101, test_time, 41.0)
    assert next_observation.sequence_delta == 1

    summary = loss_stats.summary(11.0)
    assert summary["received_count"] == 3
    assert summary["missed_count"] == 2
    assert summary["total_expected_count"] == 4
    assert summary["loss_rate_percent"] == 50.0
    assert summary["average_payload_rate_Bps"] == 102.0
    assert summary["estimated_required_payload_rate_Bps"] == 68.0

    import tempfile

    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_path = Path(temporary_directory)
        csv_path = temporary_path / "telemetry.csv"
        csv_logger = CsvLogger(csv_path)
        csv_logger.open()
        csv_logger.write(packets[0])
        csv_logger.close()
        with csv_path.open(newline="", encoding="utf-8") as csv_file:
            csv_rows = list(csv.DictReader(csv_file))
        assert len(csv_rows) == 1
        assert "pc_receive_time" in csv_rows[0]
        assert "loss_rate_percent" in csv_rows[0]

        summary_path = temporary_path / "summary.json"
        write_summary_json(summary_path, summary)
        with summary_path.open(encoding="utf-8") as summary_file:
            saved_summary = json.load(summary_file)
        assert saved_summary["packet_length_bytes"] == TELEMETRY_LENGTH
        assert saved_summary["missed_count"] == 2

        import contextlib
        import io
        import types

        class FakeSerialPort:
            closed = False

            def __init__(self, **_: object) -> None:
                self._reads = iter((first,))

            def __enter__(self) -> "FakeSerialPort":
                return self

            def __exit__(self, *_: object) -> None:
                type(self).closed = True

            def read(self, _: int) -> bytes:
                try:
                    return next(self._reads)
                except StopIteration:
                    raise KeyboardInterrupt from None

        fake_serial_module = types.SimpleNamespace(
            Serial=FakeSerialPort,
            SerialException=OSError,
            EIGHTBITS=8,
            PARITY_NONE="N",
            STOPBITS_ONE=1,
        )
        previous_serial_module = sys.modules.get("serial")
        sys.modules["serial"] = fake_serial_module
        interrupt_summary_path = temporary_path / "interrupt_summary.json"
        interrupt_csv_path = temporary_path / "interrupt.csv"
        interrupt_args = argparse.Namespace(
            port="TEST",
            baud=57600,
            csv=str(interrupt_csv_path),
            summary_json=str(interrupt_summary_path),
            expected_period_ms=500.0,
            packet_length=TELEMETRY_LENGTH,
            stats_interval_s=5.0,
            raw_hex=False,
            debug_stream=False,
            no_csv=False,
        )
        try:
            captured_output = io.StringIO()
            with contextlib.redirect_stdout(captured_output):
                assert receive_packets(interrupt_args) == 0
        finally:
            if previous_serial_module is None:
                del sys.modules["serial"]
            else:
                sys.modules["serial"] = previous_serial_module

        assert FakeSerialPort.closed
        assert interrupt_summary_path.exists()
        assert interrupt_csv_path.exists()
        assert "=== LoRa Test Summary ===" in captured_output.getvalue()
        with interrupt_summary_path.open(encoding="utf-8") as summary_file:
            interrupt_summary = json.load(summary_file)
        assert interrupt_summary["received_count"] == 1

    print("SELF-TEST PASSED")
    return 0


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def telemetry_packet_length(value: str) -> int:
    number = positive_int(value)
    if number < TELEMETRY_LENGTH:
        raise argparse.ArgumentTypeError(
            f"must be at least {TELEMETRY_LENGTH} bytes"
        )
    return number


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    """Parse command-line options."""
    parser = argparse.ArgumentParser(
        description="Receive 34-byte COMET LoRa telemetry packets."
    )
    parser.add_argument("--port", default="COM5", help="serial port (default: COM5)")
    parser.add_argument(
        "--baud", type=positive_int, default=57600, help="baud rate (default: 57600)"
    )
    parser.add_argument(
        "--csv", default="lora_log.csv", help="CSV path (default: lora_log.csv)"
    )
    parser.add_argument(
        "--summary-json",
        help="summary JSON path (default: timestamped file in current directory)",
    )
    parser.add_argument(
        "--expected-period-ms",
        type=positive_float,
        help="nominal telemetry period used for jitter and required-rate calculations",
    )
    parser.add_argument(
        "--packet-length",
        type=telemetry_packet_length,
        default=TELEMETRY_LENGTH,
        help=f"wire packet length in bytes (default: {TELEMETRY_LENGTH})",
    )
    parser.add_argument(
        "--stats-interval-s",
        type=positive_float,
        default=5.0,
        help="cumulative console statistics interval (default: 5)",
    )
    parser.add_argument(
        "--raw-hex", action="store_true", help="append validated packet hex dumps"
    )
    parser.add_argument(
        "--debug-stream",
        action="store_true",
        help="print all UART bytes, including invalid data",
    )
    parser.add_argument("--no-csv", action="store_true", help="disable CSV logging")
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="test the parser without opening a COM port",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return run_self_test()
    return receive_packets(args)


if __name__ == "__main__":
    raise SystemExit(main())
