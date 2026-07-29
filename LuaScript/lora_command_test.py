#!/usr/bin/env python3
"""Comet LoRa command 송신/ACK 수신 테스트 도구.

필요 패키지:
    python -m pip install pyserial

예:
    python LuaScript/lora_command_test.py --list-commands
    python LuaScript/lora_command_test.py --command FORCE_ASCENT --dry-run
    python LuaScript/lora_command_test.py --command FORCE_ASCENT --yes
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterator


BAUD = 9600
DEFAULT_PORT = "COM18"

TELEMETRY_HEADER = 0xA5
COMMAND_HEADER = 0x5A
ACK_HEADER = 0xA6

TELEMETRY_LENGTH = 34
COMMAND_LENGTH = 4
ACK_LENGTH = 4

COMMANDS = {
    "SERVO_MAIN_LOCK": 1,
    "SERVO_MAIN_RELEASE": 2,
    "SERVO_CANDIDATE_LOCK": 3,
    "SERVO_CANDIDATE_RELEASE": 4,
    "LOCK_ALL": 5,
    "RELEASE_ALL": 6,
    "FORCE_ASCENT": 7,
    "FORCE_DEPLOY": 8,
    "FORCE_DESCENT": 9,
    "FORCE_LANDED": 10,
}

ACK_RESULTS = {
    0: "OK",
    1: "INVALID_COMMAND",
    2: "NOT_ALLOWED",
    3: "EXECUTION_FAILED",
}


def crc8_atm(data: bytes) -> int:
    """Lua 구현과 동일한 CRC-8/ATM(poly=0x07, init=0x00)을 계산한다."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def make_command_packet(sequence: int, command: int) -> bytes:
    body = bytes((COMMAND_HEADER, sequence, command))
    return body + bytes((crc8_atm(body),))


def parse_command(value: str) -> tuple[str, int]:
    normalized = value.strip().upper().replace("-", "_")
    if normalized in COMMANDS:
        return normalized, COMMANDS[normalized]

    try:
        number = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"알 수 없는 명령: {value!r}. --list-commands로 목록을 확인하세요."
        ) from exc

    if not 0 <= number <= 0xFF:
        raise argparse.ArgumentTypeError("명령 번호는 0~255 범위여야 합니다.")

    name = next(
        (known_name for known_name, known_value in COMMANDS.items()
         if known_value == number),
        f"UNKNOWN_{number}",
    )
    return name, number


def parse_u8(value: str) -> int:
    try:
        number = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"잘못된 8비트 값: {value!r}") from exc

    if not 0 <= number <= 0xFF:
        raise argparse.ArgumentTypeError("값은 0~255 범위여야 합니다.")
    return number


def prompt_for_command() -> tuple[str, int] | None:
    print("\nComet LoRa 명령 송신 테스트")
    print(" 0 (0x00)  ACK 통신 시험 (장치 동작 없음)")
    for name, value in COMMANDS.items():
        print(f"{value:2d} (0x{value:02X})  {name}")

    try:
        value = input("\n명령 번호 또는 이름을 입력하세요 [기본값: 0, 종료: Q]: ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\n취소했습니다.")
        return None

    if value.upper() in {"Q", "QUIT", "EXIT"}:
        print("취소했습니다.")
        return None
    if not value:
        value = "0"

    try:
        return parse_command(value)
    except argparse.ArgumentTypeError as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return None


@dataclass(frozen=True)
class Frame:
    kind: str
    raw: bytes


class FrameDecoder:
    """ACK와 주기적으로 섞여 들어오는 텔레메트리를 고정 길이로 분리한다."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Iterator[Frame]:
        self._buffer.extend(data)

        while self._buffer:
            header_index = self._find_next_header()
            if header_index is None:
                self._buffer.clear()
                return
            if header_index:
                del self._buffer[:header_index]

            frame_length = (
                ACK_LENGTH
                if self._buffer[0] == ACK_HEADER
                else TELEMETRY_LENGTH
            )
            if len(self._buffer) < frame_length:
                return

            candidate = bytes(self._buffer[:frame_length])
            if crc8_atm(candidate[:-1]) == candidate[-1]:
                del self._buffer[:frame_length]
                kind = "ack" if candidate[0] == ACK_HEADER else "telemetry"
                yield Frame(kind, candidate)
            else:
                # 잘못된 헤더 또는 손상된 프레임에서 한 바이트씩 재동기화한다.
                del self._buffer[0]

    def _find_next_header(self) -> int | None:
        for index, byte in enumerate(self._buffer):
            if byte in (ACK_HEADER, TELEMETRY_HEADER):
                return index
        return None


def wait_for_ack(
    serial_port: object,
    expected_sequence: int,
    timeout: float,
    show_telemetry: bool,
) -> tuple[int, bytes] | None:
    decoder = FrameDecoder()
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        waiting = int(getattr(serial_port, "in_waiting", 0))
        chunk = serial_port.read(waiting or 1)
        if not chunk:
            continue

        for frame in decoder.feed(chunk):
            if frame.kind == "telemetry":
                if show_telemetry:
                    print(f"RX TELEMETRY: {frame.raw.hex(' ').upper()}")
                continue

            sequence = frame.raw[1]
            result = frame.raw[2]
            if sequence != expected_sequence:
                print(
                    f"RX ACK 무시: sequence={sequence}, "
                    f"result={ACK_RESULTS.get(result, f'UNKNOWN_{result}')}",
                    file=sys.stderr,
                )
                continue
            return result, frame.raw

    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Comet LoRa UART 명령을 보내고 ACK를 확인합니다."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"LoRa가 연결된 직렬 포트 (기본값: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=BAUD,
        help=f"보레이트 (기본값: {BAUD})",
    )
    parser.add_argument(
        "--command",
        type=parse_command,
        help="명령 이름 또는 번호 (예: FORCE_ASCENT, 7, 0x07)",
    )
    parser.add_argument(
        "--sequence",
        type=parse_u8,
        help="0~255 sequence. 생략하면 현재 monotonic 시간에서 생성",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="각 송신 후 ACK 대기 시간(초, 기본값: 2.0)",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="ACK가 없을 때 추가 재전송 횟수(기본값: 2)",
    )
    parser.add_argument(
        "--show-telemetry",
        action="store_true",
        help="ACK 대기 중 수신한 유효 텔레메트리도 표시",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="직렬 포트를 열지 않고 송신 패킷만 표시",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="실제 서보 동작/비행 상태 변경 위험을 확인하고 송신",
    )
    parser.add_argument(
        "--list-commands",
        action="store_true",
        help="지원 명령 목록을 출력하고 종료",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.list_commands:
        for name, value in COMMANDS.items():
            print(f"{value:2d} (0x{value:02X})  {name}")
        return 0

    interactive = args.command is None
    if interactive:
        args.command = prompt_for_command()
        if args.command is None:
            return 2
    if args.timeout <= 0:
        print("오류: --timeout은 0보다 커야 합니다.", file=sys.stderr)
        return 2
    if args.retries < 0:
        print("오류: --retries는 0 이상이어야 합니다.", file=sys.stderr)
        return 2

    command_name, command_value = args.command
    sequence = (
        args.sequence
        if args.sequence is not None
        else (time.monotonic_ns() // 1_000_000) & 0xFF
    )
    packet = make_command_packet(sequence, command_value)

    print(
        f"COMMAND={command_name}({command_value}), SEQUENCE={sequence}\n"
        f"TX: {packet.hex(' ').upper()}"
    )

    if args.dry_run:
        return 0
    if not args.yes:
        if interactive:
            warning = (
                "장치 동작 없는 ACK 통신 시험"
                if command_value == 0
                else "실제 서보 또는 비행 상태를 변경할 수 있는 명령"
            )
            try:
                answer = input(
                    f"\n{args.port}로 {warning}을 송신합니다. "
                    "계속하려면 YES를 입력하세요: "
                ).strip()
            except (EOFError, KeyboardInterrupt):
                print("\n송신을 취소했습니다.")
                return 2
            if answer.upper() != "YES":
                print("송신을 취소했습니다.")
                return 2
        else:
            print(
                "송신 중단: 이 명령은 실제 서보 또는 비행 상태를 변경할 수 있습니다. "
                "위험을 확인했다면 --yes를 추가하세요.",
                file=sys.stderr,
            )
            return 2

    try:
        import serial
    except ImportError:
        print(
            "오류: pyserial이 필요합니다. "
            "'python -m pip install pyserial'로 설치하세요.",
            file=sys.stderr,
        )
        return 2

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=min(0.1, args.timeout),
            write_timeout=args.timeout,
        ) as serial_port:
            # 실행 전에 쌓인 텔레메트리/과거 ACK를 제거한다.
            serial_port.reset_input_buffer()

            for attempt in range(1, args.retries + 2):
                print(f"송신 시도 {attempt}/{args.retries + 1}")
                serial_port.write(packet)
                serial_port.flush()

                ack = wait_for_ack(
                    serial_port,
                    expected_sequence=sequence,
                    timeout=args.timeout,
                    show_telemetry=args.show_telemetry,
                )
                if ack is None:
                    print(f"ACK timeout ({args.timeout:g}초)", file=sys.stderr)
                    continue

                result, raw = ack
                result_name = ACK_RESULTS.get(result, f"UNKNOWN_{result}")
                print(f"RX ACK: {raw.hex(' ').upper()}")
                print(f"ACK RESULT: {result_name} ({result})")
                if command_value == 0 and result == 1:
                    print("LINK TEST: PASS (기체가 INVALID_COMMAND로 정상 응답)")
                    return 0
                return 0 if result == 0 else 1

    except Exception as exc:
        # pyserial의 포트/timeout 예외를 포함해 사용자가 바로 원인을 볼 수 있게 한다.
        print(f"직렬 통신 오류: {exc}", file=sys.stderr)
        return 2

    print("최종 결과: 유효한 ACK를 받지 못했습니다.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
