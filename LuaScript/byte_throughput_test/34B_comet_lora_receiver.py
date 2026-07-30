#!/usr/bin/env python3
"""COMET LoRa fixed 34-byte, 10 Hz throughput receiver."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import serial
except ImportError:
    print(
        "pyserial is required: python -m pip install pyserial",
        file=sys.stderr,
    )
    raise

HEADER = 0xA5
MARKER = 0x5A
PACKET_LENGTH = 34
TEST_PAYLOAD_LENGTH = 24
MARKER_OFFSET = 32


def crc8_atm(data: bytes) -> int:
    """CRC-8/ATM: poly=0x07, init=0x00, xorout=0x00."""
    crc = 0

    for value in data:
        crc ^= value

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


@dataclass(frozen=True)
class Packet:
    sequence: int
    packet_counter: int
    loop_delta_ms: int


def decode_packet(frame: bytes) -> Packet | None:
    if len(frame) != PACKET_LENGTH:
        return None

    if frame[0] != HEADER or frame[MARKER_OFFSET] != MARKER:
        return None

    if crc8_atm(frame[:-1]) != frame[-1]:
        return None

    sequence = frame[1]

    expected_payload = bytes(
        (sequence + i) & 0xFF
        for i in range(TEST_PAYLOAD_LENGTH)
    )

    if frame[8:32] != expected_payload:
        return None

    packet_counter, loop_delta_ms = struct.unpack_from("<IH", frame, 2)

    return Packet(
        sequence=sequence,
        packet_counter=packet_counter,
        loop_delta_ms=loop_delta_ms,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Example: COM5")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Seconds; 0 means run until Ctrl+C",
    )
    parser.add_argument("--report-period", type=float, default=1.0)
    parser.add_argument("--csv", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    csv_file = None
    csv_writer = None

    if args.csv is not None:
        csv_file = args.csv.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                "host_time_s",
                "sequence",
                "packet_counter",
                "loop_delta_ms",
            ]
        )

    valid_total = 0
    invalid_total = 0
    lost_total = 0
    raw_bytes_total = 0

    period_valid = 0
    period_invalid = 0
    period_lost = 0
    period_raw_bytes = 0

    previous_counter: int | None = None
    rx_buffer = bytearray()

    start = time.monotonic()
    report_start = start

    print(
        f"Opening {args.port} at {args.baud} baud, "
        f"frame={PACKET_LENGTH} B, expected=10 Hz / 340 B/s"
    )

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        ) as ser:
            ser.reset_input_buffer()

            while True:
                now = time.monotonic()

                if args.duration > 0 and (now - start) >= args.duration:
                    break

                chunk = ser.read(ser.in_waiting or 1)

                if chunk:
                    rx_buffer.extend(chunk)
                    raw_bytes_total += len(chunk)
                    period_raw_bytes += len(chunk)

                while True:
                    try:
                        header_index = rx_buffer.index(HEADER)
                    except ValueError:
                        if rx_buffer:
                            period_invalid += len(rx_buffer)
                            invalid_total += len(rx_buffer)
                            rx_buffer.clear()
                        break

                    if header_index > 0:
                        del rx_buffer[:header_index]
                        period_invalid += header_index
                        invalid_total += header_index

                    if len(rx_buffer) < PACKET_LENGTH:
                        break

                    candidate = bytes(rx_buffer[:PACKET_LENGTH])
                    packet = decode_packet(candidate)

                    if packet is None:
                        del rx_buffer[0]
                        period_invalid += 1
                        invalid_total += 1
                        continue

                    del rx_buffer[:PACKET_LENGTH]
                    valid_total += 1
                    period_valid += 1

                    if previous_counter is not None:
                        lost = (
                            packet.packet_counter
                            - previous_counter
                            - 1
                        ) & 0xFFFFFFFF

                        lost_total += lost
                        period_lost += lost

                    previous_counter = packet.packet_counter

                    if csv_writer is not None:
                        csv_writer.writerow(
                            [
                                time.time(),
                                packet.sequence,
                                packet.packet_counter,
                                packet.loop_delta_ms,
                            ]
                        )

                if (now - report_start) >= args.report_period:
                    elapsed = now - report_start
                    packet_rate = period_valid / elapsed
                    valid_rate = (
                        period_valid * PACKET_LENGTH / elapsed
                    )
                    raw_rate = period_raw_bytes / elapsed
                    attempted = period_valid + period_lost
                    delivery = (
                        100.0 * period_valid / attempted
                        if attempted > 0
                        else 0.0
                    )

                    print(
                        f"RX {packet_rate:7.2f} pkt/s | "
                        f"valid {valid_rate:7.1f} B/s | "
                        f"raw {raw_rate:7.1f} B/s | "
                        f"lost {period_lost:5d} | "
                        f"invalid/resync {period_invalid:5d} | "
                        f"delivery {delivery:6.2f}%"
                    )

                    period_valid = 0
                    period_invalid = 0
                    period_lost = 0
                    period_raw_bytes = 0
                    report_start = now

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2
    finally:
        if csv_file is not None:
            csv_file.close()

    elapsed_total = max(time.monotonic() - start, 1e-9)
    attempted_total = valid_total + lost_total

    delivery_total = (
        100.0 * valid_total / attempted_total
        if attempted_total > 0
        else 0.0
    )

    print("\n=== Final result ===")
    print(f"Elapsed             : {elapsed_total:.3f} s")
    print(f"Valid packets       : {valid_total}")
    print(f"Estimated lost      : {lost_total}")
    print(f"Invalid/resync bytes: {invalid_total}")
    print(f"Raw received bytes  : {raw_bytes_total}")
    print(f"Valid packet rate   : {valid_total / elapsed_total:.2f} pkt/s")
    print(
        f"Valid payload rate  : "
        f"{valid_total * PACKET_LENGTH / elapsed_total:.1f} B/s"
    )
    print(f"Raw serial rate     : {raw_bytes_total / elapsed_total:.1f} B/s")
    print(f"Delivery ratio      : {delivery_total:.3f}%")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
