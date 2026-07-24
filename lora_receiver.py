#!/usr/bin/env python3
"""Receive and log 33-byte COMET telemetry packets from the LoRa UART."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import IO, Iterable, Optional


TELEMETRY_HEADER = 0xA5
TELEMETRY_LENGTH = 33
STATE_NAMES = {
    0: "READY",
    1: "ASCENT",
    2: "APOGEE",
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
CSV_FIELDS = [
    "timestamp_iso",
    "packet_type",
    "sequence",
    "state_id",
    "state_name",
    "status_flags",
    "status_names",
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
]


@dataclass(slots=True)
class Statistics:
    """Receiver and stream-parser counters."""

    total_serial_bytes: int = 0
    total_valid_packets: int = 0
    crc_errors: int = 0
    discarded_resync_bytes: int = 0
    header_candidates: int = 0
    estimated_lost_packets: int = 0
    duplicate_packets: int = 0


@dataclass(slots=True)
class TelemetryPacket:
    """Decoded COMET telemetry packet."""

    timestamp: datetime
    sequence: int
    state_id: int
    status_flags: int
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

    @property
    def state_name(self) -> str:
        """Return a readable flight-state name."""
        return STATE_NAMES.get(self.state_id, f"UNKNOWN_{self.state_id}")

    @property
    def status_names(self) -> str:
        """Return a pipe-separated list of valid sensor groups."""
        names = [name for bit, name in STATUS_BITS.items() if self.status_flags & bit]
        return "|".join(names) if names else "NONE"

    def csv_row(self) -> dict[str, object]:
        """Return one row matching the fixed CSV schema."""
        return {
            "timestamp_iso": self.timestamp.isoformat(timespec="milliseconds"),
            "packet_type": "TELEMETRY",
            "sequence": self.sequence,
            "state_id": self.state_id,
            "state_name": self.state_name,
            "status_flags": f"0x{self.status_flags:02X}",
            "status_names": self.status_names,
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


class StreamParser:
    """Incrementally recover fixed-length COMET packets from a UART stream."""

    def __init__(self, statistics: Optional[Statistics] = None) -> None:
        self.buffer = bytearray()
        self.statistics = statistics if statistics is not None else Statistics()
        self._last_sequence: Optional[int] = None

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

            if len(self.buffer) < TELEMETRY_LENGTH:
                break

            self.statistics.header_candidates += 1
            candidate = bytes(self.buffer[:TELEMETRY_LENGTH])
            expected_crc = crc8_atm(candidate[:-1])
            if candidate[-1] != expected_crc:
                self.statistics.crc_errors += 1
                # Keep searching inside the failed candidate for the next header.
                self._discard(1)
                continue

            del self.buffer[:TELEMETRY_LENGTH]
            packet = decode_packet(candidate)
            self._record_valid_packet(packet.sequence)
            packets.append(packet)

        return packets

    def _discard(self, count: int) -> None:
        if count <= 0:
            return
        del self.buffer[:count]
        self.statistics.discarded_resync_bytes += count

    def _record_valid_packet(self, sequence: int) -> None:
        stats = self.statistics
        stats.total_valid_packets += 1
        if self._last_sequence is None:
            self._last_sequence = sequence
            return

        forward_distance = (sequence - self._last_sequence) & 0xFF
        if forward_distance == 0:
            stats.duplicate_packets += 1
        else:
            stats.estimated_lost_packets += forward_distance - 1
            self._last_sequence = sequence


class CsvLogger:
    """Append telemetry to CSV and flush after every packet."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._file: Optional[IO[str]] = None
        self._writer: Optional[csv.DictWriter] = None

    def open(self) -> None:
        csv_file = self.path.open("a+", newline="", encoding="utf-8")
        try:
            csv_file.seek(0, 2)
            is_empty = csv_file.tell() == 0
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


def decode_packet(raw: bytes) -> TelemetryPacket:
    """Decode the exact layout emitted by comet_lora.lua."""
    (
        header,
        sequence,
        state_id,
        status_flags,
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
    ) = struct.unpack_from("<BBBBIiiihhhhhh", raw)

    if header != TELEMETRY_HEADER:
        raise ValueError(f"unexpected telemetry header: 0x{header:02X}")

    return TelemetryPacket(
        timestamp=datetime.now().astimezone(),
        sequence=sequence,
        state_id=state_id,
        status_flags=status_flags,
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
        f"flags=0x{packet.status_flags:02X}({packet.status_names}) "
        f"lat={packet.latitude_deg:.7f} lon={packet.longitude_deg:.7f} "
        f"alt={packet.altitude_m:.2f}m "
        f"velNED=({packet.velocity_n_mps:.1f},{packet.velocity_e_mps:.1f},"
        f"{packet.velocity_d_mps:.1f})m/s "
        f"rpy=({packet.roll_deg:.2f},{packet.pitch_deg:.2f},"
        f"{packet.yaw_deg:.2f})deg uptime={packet.time_ms}ms"
    )
    if show_raw_hex:
        line += f" raw={packet.raw_hex}"
    return line


def format_stream_chunk(chunk: bytes) -> str:
    """Format unfiltered UART data for electrical/baud diagnostics."""
    timestamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    return f"[{timestamp}] RX bytes={len(chunk)} hex={chunk.hex(' ').upper()}"


def print_statistics(statistics: Statistics) -> None:
    """Print receiver statistics at shutdown."""
    print("\nReceiver statistics:")
    print(f"  total serial bytes:     {statistics.total_serial_bytes}")
    print(f"  total valid packets:    {statistics.total_valid_packets}")
    print(f"  CRC-8 errors:           {statistics.crc_errors}")
    print(f"  0xA5 header candidates: {statistics.header_candidates}")
    print(f"  discarded/resync bytes: {statistics.discarded_resync_bytes}")
    print(f"  estimated lost packets: {statistics.estimated_lost_packets}")
    print(f"  duplicate packets:      {statistics.duplicate_packets}")


def receive_packets(args: argparse.Namespace) -> int:
    """Open the serial port and receive until Ctrl+C or disconnection."""
    try:
        import serial
    except ImportError:
        print("Error: install pyserial with: pip install pyserial", file=sys.stderr)
        return 2

    statistics = Statistics()
    parser = StreamParser(statistics)
    csv_logger: Optional[CsvLogger] = None

    if not args.no_csv:
        csv_logger = CsvLogger(Path(args.csv))
        try:
            csv_logger.open()
        except (OSError, csv.Error) as exc:
            print(f"CSV open error for {args.csv!r}: {exc}", file=sys.stderr)
            return 2

    exit_code = 0
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
                warning_printed = False
                while True:
                    chunk = ser.read(256)
                    if chunk and args.debug_stream:
                        print(format_stream_chunk(chunk), flush=True)

                    packets = parser.feed(chunk)
                    for packet in packets:
                        print(format_packet(packet, args.raw_hex), flush=True)
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
        if csv_logger is not None:
            try:
                csv_logger.close()
            except OSError as exc:
                print(f"CSV close error: {exc}", file=sys.stderr)
                exit_code = 1
        print_statistics(statistics)

    return exit_code


def build_test_packet(
    sequence: int,
    state_id: int,
    status_flags: int,
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
        "<BBBBIiiihhhhhh",
        TELEMETRY_HEADER,
        sequence,
        state_id,
        status_flags,
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
    """Test CRC, fragmentation, resync, signed fields, and sequence tracking."""
    first = build_test_packet(
        sequence=254,
        state_id=1,
        status_flags=0x0F,
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
    print("SELF-TEST PASSED")
    return 0


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    """Parse command-line options."""
    parser = argparse.ArgumentParser(
        description="Receive 33-byte COMET LoRa telemetry packets."
    )
    parser.add_argument("--port", default="COM5", help="serial port (default: COM5)")
    parser.add_argument(
        "--baud", type=positive_int, default=57600, help="baud rate (default: 57600)"
    )
    parser.add_argument(
        "--csv", default="lora_log.csv", help="CSV path (default: lora_log.csv)"
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
