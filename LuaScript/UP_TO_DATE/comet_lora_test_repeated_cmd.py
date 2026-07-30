from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass

import serial


# ============================================================
# Configuration
# ============================================================

PORT = "COM18"
BAUD = 9600

TELEMETRY_HEADER = 0xA5
COMMAND_HEADER = 0x5A

TELEMETRY_LENGTH = 36
COMMAND_LENGTH = 4

# telemetry 완전 수신 후 CMD 묶음 송신까지 대기시간
COMMAND_DELAY_AFTER_TELEMETRY_S = 0.005

# 동일 sequence + command 패킷 반복 송신
COMMAND_REPEAT_COUNT = 3
COMMAND_REPEAT_INTERVAL_S = 0.005

# telemetry 내 embedded ACK 대기시간
ACK_TIMEOUT_S = 2.0

ACK_RESULT_NONE = 0xFF


COMMANDS = {
    "main_lock": 1,
    "main_release": 2,
    "candidate_lock": 3,
    "candidate_release": 4,
    "lock_all": 5,
    "release_all": 6,
    "force_ascent": 7,
    "force_deploy": 8,
    "force_descent": 9,
    "force_landed": 10,
}

COMMAND_NAMES = {
    value: name.upper()
    for name, value in COMMANDS.items()
}

ACK_RESULT_NAMES = {
    0: "OK",
    1: "INVALID_COMMAND",
    2: "NOT_ALLOWED",
    3: "EXECUTION_FAILED",
    ACK_RESULT_NONE: "NONE",
}

STATE_NAMES = {
    0: "READY",
    1: "ASCENT",
    2: "DEPLOY",
    3: "DESCENT",
    4: "LANDED",
}


# ============================================================
# Protocol
# ============================================================

def calculate_crc8(data: bytes) -> int:
    """CRC-8/ATM, polynomial 0x07, initial value 0x00."""
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
    body = bytes([
        COMMAND_HEADER,
        sequence & 0xFF,
        command & 0xFF,
    ])

    return body + bytes([calculate_crc8(body)])


def parse_telemetry(packet: bytes) -> dict:
    if len(packet) != TELEMETRY_LENGTH:
        raise ValueError("Invalid telemetry length")

    if packet[0] != TELEMETRY_HEADER:
        raise ValueError("Invalid telemetry header")

    if calculate_crc8(packet[:-1]) != packet[-1]:
        raise ValueError("Telemetry CRC error")

    values = struct.unpack(
        "<BBBBBIiiihhhhhhBBB",
        packet,
    )

    return {
        "sequence": values[1],
        "state": values[2],
        "status_flags": values[3],
        "fault_flags": values[4],
        "time_ms": values[5],
        "latitude": values[6] / 10_000_000.0,
        "longitude": values[7] / 10_000_000.0,
        "altitude": values[8] / 100.0,
        "velocity_n": values[9] / 10.0,
        "velocity_e": values[10] / 10.0,
        "velocity_d": values[11] / 10.0,
        "roll": values[12] / 100.0,
        "pitch": values[13] / 100.0,
        "yaw": values[14] / 100.0,
        "ack_sequence": values[15],
        "ack_result": values[16],
    }


# ============================================================
# Command state
# ============================================================

@dataclass
class CommandState:
    sequence: int
    command: int
    packet: bytes

    waiting_for_telemetry: bool = True
    send_due_time: float | None = None
    sent_time: float | None = None


# ============================================================
# Test program
# ============================================================

class CometLoRaTest:
    def __init__(self) -> None:
        self.serial = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            write_timeout=0.1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        self.running = True
        self.lock = threading.Lock()
        self.rx_buffer = bytearray()

        # 첫 telemetry의 기존 ACK sequence 다음 값부터 시작한다.
        self.sequence_initialized = False
        self.next_sequence = 0

        # 송신 슬롯 대기 또는 telemetry 내 ACK 대기 중인 CMD
        self.active_command: CommandState | None = None

        # timeout 후 아직 ACK가 확인되지 않은 마지막 CMD
        self.last_timeout_command: CommandState | None = None

    def start(self) -> None:
        print(f"[LINK] Connected: {PORT} @ {BAUD}")
        print("[LINK] Waiting for telemetry")

        threading.Thread(
            target=self.receive_loop,
            daemon=True,
        ).start()

        threading.Thread(
            target=self.scheduler_loop,
            daemon=True,
        ).start()

        self.cli_loop()

    # --------------------------------------------------------
    # Receive
    # --------------------------------------------------------

    def receive_loop(self) -> None:
        while self.running:
            try:
                data = self.serial.read(
                    self.serial.in_waiting or 1
                )
            except serial.SerialException as exc:
                print(f"\n[ERROR] Serial receive failed: {exc}")
                self.running = False
                return

            if not data:
                continue

            with self.lock:
                self.rx_buffer.extend(data)
                self.process_rx_buffer()

    def process_rx_buffer(self) -> None:
        while self.rx_buffer:
            if self.rx_buffer[0] != TELEMETRY_HEADER:
                del self.rx_buffer[0]
                continue

            if len(self.rx_buffer) < TELEMETRY_LENGTH:
                return

            packet = bytes(
                self.rx_buffer[:TELEMETRY_LENGTH]
            )

            try:
                telemetry = parse_telemetry(packet)
            except ValueError as exc:
                print(f"\n[RX] {exc}")

                # 재동기화를 위해 헤더 1바이트만 폐기한다.
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:TELEMETRY_LENGTH]
            self.handle_telemetry(telemetry)

    def handle_telemetry(self, telemetry: dict) -> None:
        now = time.monotonic()

        if not self.sequence_initialized:
            if telemetry["ack_result"] == ACK_RESULT_NONE:
                self.next_sequence = 0
            else:
                self.next_sequence = (
                    telemetry["ack_sequence"] + 1
                ) % 256

            self.sequence_initialized = True

            print(
                f"\n[LINK] Command sequence initialized: "
                f"{self.next_sequence}"
            )

        state = STATE_NAMES.get(
            telemetry["state"],
            f"UNKNOWN({telemetry['state']})",
        )

        ack_result_name = ACK_RESULT_NAMES.get(
            telemetry["ack_result"],
            f"UNKNOWN({telemetry['ack_result']})",
        )

        print(
            f"\r[TEL] "
            f"seq={telemetry['sequence']:3d} "
            f"state={state:<7} "
            f"time={telemetry['time_ms']:8d}ms "
            f"alt={telemetry['altitude']:8.2f}m "
            f"VD={telemetry['velocity_d']:7.1f}m/s "
            f"status=0x{telemetry['status_flags']:02X} "
            f"fault=0x{telemetry['fault_flags']:02X} "
            f"ack={telemetry['ack_sequence']:3d}/{ack_result_name}",
            end="",
            flush=True,
        )

        self.handle_embedded_ack(telemetry, now)

        # CMD는 다음 telemetry를 완전히 받은 뒤 지정된 지연 후 송신한다.
        command = self.active_command

        if (
            command is not None
            and command.waiting_for_telemetry
        ):
            command.waiting_for_telemetry = False
            command.send_due_time = (
                now + COMMAND_DELAY_AFTER_TELEMETRY_S
            )

    def handle_embedded_ack(
        self,
        telemetry: dict,
        now: float,
    ) -> None:
        ack_sequence = telemetry["ack_sequence"]
        ack_result = telemetry["ack_result"]

        if ack_result == ACK_RESULT_NONE:
            return

        # 정상 ACK
        command = self.active_command

        if (
            command is not None
            and command.sent_time is not None
            and ack_sequence == command.sequence
        ):
            result_name = ACK_RESULT_NAMES.get(
                ack_result,
                f"UNKNOWN({ack_result})",
            )

            latency_ms = (
                now - command.sent_time
            ) * 1000.0

            print(
                f"\n[ACK] "
                f"seq={ack_sequence} "
                f"command={COMMAND_NAMES[command.command]} "
                f"result={result_name} "
                f"latency={latency_ms:.1f}ms"
            )

            self.active_command = None
            self.last_timeout_command = None
            return

        # timeout 이후 늦게 확인된 ACK
        timed_out = self.last_timeout_command

        if (
            timed_out is not None
            and timed_out.sent_time is not None
            and ack_sequence == timed_out.sequence
        ):
            result_name = ACK_RESULT_NAMES.get(
                ack_result,
                f"UNKNOWN({ack_result})",
            )

            latency_ms = (
                now - timed_out.sent_time
            ) * 1000.0

            print(
                f"\n[LATE ACK] "
                f"seq={ack_sequence} "
                f"command={COMMAND_NAMES[timed_out.command]} "
                f"result={result_name} "
                f"latency={latency_ms:.1f}ms"
            )

            self.last_timeout_command = None

    # --------------------------------------------------------
    # Command
    # --------------------------------------------------------

    def request_command(self, command: int) -> None:
        with self.lock:
            if not self.sequence_initialized:
                print(
                    "\n[CMD] Rejected: wait for the first telemetry"
                )
                return

            if self.active_command is not None:
                print(
                    "\n[CMD] Rejected: another CMD is active"
                )
                return

            # timeout된 명령의 결과가 아직 불명확하면 새 sequence를 막는다.
            if self.last_timeout_command is not None:
                print(
                    "\n[CMD] Rejected: previous CMD is unresolved"
                )
                print(
                    "[CMD] Use 'resend' or 'clear_timeout'"
                )
                return

            sequence = self.next_sequence
            self.next_sequence = (
                self.next_sequence + 1
            ) % 256

            self.active_command = CommandState(
                sequence=sequence,
                command=command,
                packet=make_command_packet(
                    sequence,
                    command,
                ),
            )

            print(
                f"\n[CMD] Queued "
                f"seq={sequence} "
                f"command={COMMAND_NAMES[command]}"
            )
            print(
                "[CMD] Waiting for next complete telemetry"
            )

    def resend(self) -> None:
        with self.lock:
            if self.active_command is not None:
                print(
                    "\n[CMD] Rejected: another CMD is active"
                )
                return

            if self.last_timeout_command is None:
                print("\n[CMD] No timed-out CMD")
                return

            previous = self.last_timeout_command

            # 같은 sequence와 command를 그대로 사용한다.
            self.active_command = CommandState(
                sequence=previous.sequence,
                command=previous.command,
                packet=previous.packet,
            )

            print(
                f"\n[CMD] Manual resend queued "
                f"seq={previous.sequence} "
                f"command={COMMAND_NAMES[previous.command]}"
            )

    def clear_timeout(self) -> None:
        with self.lock:
            if self.active_command is not None:
                print(
                    "\n[CMD] Rejected: another CMD is active"
                )
                return

            if self.last_timeout_command is None:
                print("\n[CMD] No unresolved timed-out CMD")
                return

            sequence = self.last_timeout_command.sequence
            command = self.last_timeout_command.command

            self.last_timeout_command = None

            print(
                f"\n[CMD] Timed-out CMD cleared "
                f"seq={sequence} "
                f"command={COMMAND_NAMES[command]}"
            )
            print(
                "[WARN] A delayed execution or ACK may still arrive"
            )

    def scheduler_loop(self) -> None:
        while self.running:
            with self.lock:
                self.send_command_if_due()
                self.check_ack_timeout()

            time.sleep(0.001)

    def send_command_if_due(self) -> None:
        command = self.active_command

        if command is None:
            return

        if command.send_due_time is None:
            return

        if command.sent_time is not None:
            return

        if time.monotonic() < command.send_due_time:
            return

        # timeout과 latency는 첫 번째 패킷 송신 시점부터 계산한다.
        command.sent_time = time.monotonic()

        try:
            for repeat_index in range(COMMAND_REPEAT_COUNT):
                written = self.serial.write(command.packet)

                if written != COMMAND_LENGTH:
                    raise serial.SerialException(
                        f"Partial CMD write: "
                        f"{written}/{COMMAND_LENGTH}"
                    )

                self.serial.flush()

                if repeat_index < COMMAND_REPEAT_COUNT - 1:
                    time.sleep(
                        COMMAND_REPEAT_INTERVAL_S
                    )

        except (
            serial.SerialException,
            serial.SerialTimeoutException,
        ) as exc:
            print(f"\n[ERROR] CMD send failed: {exc}")
            self.active_command = None
            return

        raw = " ".join(
            f"{byte:02X}"
            for byte in command.packet
        )

        print(
            f"\n[CMD] Sent "
            f"seq={command.sequence} "
            f"command={COMMAND_NAMES[command.command]} "
            f"repeat={COMMAND_REPEAT_COUNT} "
            f"interval={COMMAND_REPEAT_INTERVAL_S * 1000:.1f}ms "
            f"raw={raw}"
        )

    def check_ack_timeout(self) -> None:
        command = self.active_command

        if command is None or command.sent_time is None:
            return

        elapsed = time.monotonic() - command.sent_time

        if elapsed < ACK_TIMEOUT_S:
            return

        print(
            f"\n[TIMEOUT] Embedded ACK timeout "
            f"seq={command.sequence} "
            f"command={COMMAND_NAMES[command.command]}"
        )
        print("[TIMEOUT] Enter 'resend' to retry manually")

        self.last_timeout_command = command
        self.active_command = None

    # --------------------------------------------------------
    # CLI
    # --------------------------------------------------------

    def cli_loop(self) -> None:
        print(
            "\n\nCommands:"
            "\n  main_lock"
            "\n  main_release"
            "\n  candidate_lock"
            "\n  candidate_release"
            "\n  lock_all"
            "\n  release_all"
            "\n  force_ascent"
            "\n  force_deploy"
            "\n  force_descent"
            "\n  force_landed"
            "\n  resend"
            "\n  clear_timeout"
            "\n  quit"
        )

        try:
            while self.running:
                user_input = input("\ncomet> ").strip().lower()

                if user_input in COMMANDS:
                    self.request_command(
                        COMMANDS[user_input]
                    )

                elif user_input == "resend":
                    self.resend()

                elif user_input == "clear_timeout":
                    self.clear_timeout()

                elif user_input in {"quit", "exit"}:
                    break

                elif user_input:
                    print("[CLI] Unknown command")

        except KeyboardInterrupt:
            print("\n[CLI] Interrupted")

        finally:
            self.running = False
            self.serial.close()
            print("[LINK] Closed")


def main() -> None:
    try:
        test = CometLoRaTest()
        test.start()

    except serial.SerialException as exc:
        print(f"[ERROR] Could not open {PORT}: {exc}")


if __name__ == "__main__":
    main()
