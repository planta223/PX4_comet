from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass

import serial


# ============================================================
# User configuration
# ============================================================

PORT = "COM18"
BAUD = 57600

TELEMETRY_HEADER = 0xA5
COMMAND_HEADER = 0x5A

TELEMETRY_LENGTH = 36
COMMAND_LENGTH = 4

# 실행 중 "delay 20" 명령으로 변경 가능
DEFAULT_COMMAND_DELAY_MS = 10.0

# CMD 송신 후 embedded ACK 대기시간
ACK_TIMEOUT_S = 1.0

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
    """CRC-8/ATM: polynomial 0x07, initial value 0x00."""
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
        raise ValueError(
            f"Invalid telemetry length: {len(packet)}"
        )

    if packet[0] != TELEMETRY_HEADER:
        raise ValueError(
            f"Invalid telemetry header: 0x{packet[0]:02X}"
        )

    expected_crc = calculate_crc8(packet[:-1])
    received_crc = packet[-1]

    if expected_crc != received_crc:
        raise ValueError(
            "Telemetry CRC error: "
            f"expected=0x{expected_crc:02X}, "
            f"received=0x{received_crc:02X}"
        )

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
    telemetry_received_time: float | None = None
    send_due_time: float | None = None
    sent_time: float | None = None


# ============================================================
# Ground station test
# ============================================================

class CometLoRaGroundTest:
    def __init__(self) -> None:
        self.serial = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            write_timeout=0.2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        self.running = True
        self.lock = threading.Lock()
        self.rx_buffer = bytearray()

        self.command_delay_s = (
            DEFAULT_COMMAND_DELAY_MS / 1000.0
        )

        self.sequence_initialized = False
        self.next_sequence = 0

        self.active_command: CommandState | None = None
        self.last_timeout_command: CommandState | None = None

        self.valid_telemetry_count = 0
        self.telemetry_crc_error_count = 0
        self.discarded_rx_byte_count = 0

        self.last_telemetry_host_time: float | None = None

    # --------------------------------------------------------
    # Start / stop
    # --------------------------------------------------------

    def start(self) -> None:
        print(f"[LINK] Connected: {PORT} @ {BAUD}")
        print(
            f"[CONFIG] CMD delay: "
            f"{self.command_delay_s * 1000.0:.1f} ms"
        )
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
                # 데이터가 있으면 현재 OS 버퍼의 데이터를 한꺼번에 읽고,
                # 없으면 1바이트를 최대 timeout 동안 기다린다.
                read_size = self.serial.in_waiting or 1
                received = self.serial.read(read_size)

            except serial.SerialException as exc:
                print(f"\n[ERROR] Serial receive failed: {exc}")
                self.running = False
                return

            if not received:
                continue

            with self.lock:
                self.rx_buffer.extend(received)
                self.process_rx_buffer()

    def process_rx_buffer(self) -> None:
        while self.rx_buffer:
            # telemetry header까지 재동기화
            if self.rx_buffer[0] != TELEMETRY_HEADER:
                del self.rx_buffer[0]
                self.discarded_rx_byte_count += 1
                continue

            if len(self.rx_buffer) < TELEMETRY_LENGTH:
                return

            packet = bytes(
                self.rx_buffer[:TELEMETRY_LENGTH]
            )

            try:
                telemetry = parse_telemetry(packet)

            except ValueError as exc:
                self.telemetry_crc_error_count += 1
                print(f"\n[RX ERROR] {exc}")

                # 다음 0xA5를 찾을 수 있도록 헤더 1바이트만 폐기
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:TELEMETRY_LENGTH]
            self.valid_telemetry_count += 1
            self.handle_telemetry(telemetry)

    def handle_telemetry(self, telemetry: dict) -> None:
        now = time.monotonic()

        period_ms: float | None = None
        if self.last_telemetry_host_time is not None:
            period_ms = (
                now - self.last_telemetry_host_time
            ) * 1000.0

        self.last_telemetry_host_time = now

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

        state_name = STATE_NAMES.get(
            telemetry["state"],
            f"UNKNOWN({telemetry['state']})",
        )

        ack_name = ACK_RESULT_NAMES.get(
            telemetry["ack_result"],
            f"UNKNOWN({telemetry['ack_result']})",
        )

        period_text = (
            "   N/A"
            if period_ms is None
            else f"{period_ms:6.1f}"
        )

        print(
            f"\r[TEL] "
            f"seq={telemetry['sequence']:3d} "
            f"period={period_text}ms "
            f"state={state_name:<7} "
            f"time={telemetry['time_ms']:10d}ms "
            f"alt={telemetry['altitude']:8.2f}m "
            f"VD={telemetry['velocity_d']:7.1f}m/s "
            f"status=0x{telemetry['status_flags']:02X} "
            f"fault=0x{telemetry['fault_flags']:02X} "
            f"ack={telemetry['ack_sequence']:3d}/{ack_name}",
            end="",
            flush=True,
        )

        self.handle_embedded_ack(telemetry, now)

        command = self.active_command

        # CLI에서 명령을 요청한 뒤 도착한 첫 번째 정상 telemetry를
        # 송신 슬롯 기준으로 사용한다.
        if (
            command is not None
            and command.waiting_for_telemetry
        ):
            command.waiting_for_telemetry = False
            command.telemetry_received_time = now
            command.send_due_time = (
                now + self.command_delay_s
            )

            print(
                f"\n[SLOT] Complete telemetry received; "
                f"CMD scheduled after "
                f"{self.command_delay_s * 1000.0:.1f} ms"
            )

    def handle_embedded_ack(
        self,
        telemetry: dict,
        now: float,
    ) -> None:
        command = self.active_command

        if command is None or command.sent_time is None:
            return

        ack_sequence = telemetry["ack_sequence"]
        ack_result = telemetry["ack_result"]

        if ack_result == ACK_RESULT_NONE:
            return

        if ack_sequence != command.sequence:
            return

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
            f"latency={latency_ms:.1f} ms"
        )

        self.active_command = None
        self.last_timeout_command = None

    # --------------------------------------------------------
    # Command
    # --------------------------------------------------------

    def request_command(self, command: int) -> None:
        with self.lock:
            if not self.sequence_initialized:
                print(
                    "\n[CMD] Rejected: "
                    "wait for first valid telemetry"
                )
                return

            if self.active_command is not None:
                print(
                    "\n[CMD] Rejected: another CMD is active"
                )
                return

            if self.last_timeout_command is not None:
                print(
                    "\n[CMD] Rejected: previous CMD unresolved"
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

            self.active_command = CommandState(
                sequence=previous.sequence,
                command=previous.command,
                packet=previous.packet,
            )

            print(
                f"\n[CMD] Resend queued "
                f"seq={previous.sequence} "
                f"command={COMMAND_NAMES[previous.command]}"
            )
            print(
                "[CMD] Waiting for next complete telemetry"
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

            previous = self.last_timeout_command
            self.last_timeout_command = None

            print(
                f"\n[CMD] Timed-out CMD cleared "
                f"seq={previous.sequence} "
                f"command={COMMAND_NAMES[previous.command]}"
            )
            print(
                "[WARN] Delayed execution or ACK may still arrive"
            )

    # --------------------------------------------------------
    # Scheduler
    # --------------------------------------------------------

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

        now = time.monotonic()

        if now < command.send_due_time:
            return

        try:
            written = self.serial.write(command.packet)
            self.serial.flush()

        except (
            serial.SerialException,
            serial.SerialTimeoutException,
        ) as exc:
            print(f"\n[ERROR] CMD send failed: {exc}")
            self.active_command = None
            return

        if written != COMMAND_LENGTH:
            print(
                f"\n[ERROR] Partial CMD write: "
                f"{written}/{COMMAND_LENGTH}"
            )
            self.active_command = None
            return

        command.sent_time = time.monotonic()

        actual_delay_ms: float | None = None

        if command.telemetry_received_time is not None:
            actual_delay_ms = (
                command.sent_time
                - command.telemetry_received_time
            ) * 1000.0

        actual_delay_text = (
            "N/A"
            if actual_delay_ms is None
            else f"{actual_delay_ms:.2f}"
        )

        raw = " ".join(
            f"{byte:02X}"
            for byte in command.packet
        )

        print(
            f"\n[CMD] Sent "
            f"seq={command.sequence} "
            f"command={COMMAND_NAMES[command.command]} "
            f"configured_delay="
            f"{self.command_delay_s * 1000.0:.1f}ms "
            f"actual_delay={actual_delay_text}ms "
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

        self.last_timeout_command = command
        self.active_command = None

    # --------------------------------------------------------
    # CLI configuration / diagnostics
    # --------------------------------------------------------

    def set_delay(self, user_input: str) -> None:
        parts = user_input.split()

        if len(parts) != 2:
            print("\n[CLI] Usage: delay <milliseconds>")
            return

        try:
            delay_ms = float(parts[1])
        except ValueError:
            print("\n[CLI] Invalid delay")
            return

        if delay_ms < 0.0 or delay_ms > 60.0:
            print("\n[CLI] Delay range: 0 to 60 ms")
            return

        with self.lock:
            if self.active_command is not None:
                print(
                    "\n[CLI] Cannot change delay "
                    "while CMD is active"
                )
                return

            self.command_delay_s = delay_ms / 1000.0

        print(
            f"\n[CONFIG] CMD delay set to "
            f"{delay_ms:.1f} ms"
        )

    def show_stats(self) -> None:
        with self.lock:
            active_text = (
                "none"
                if self.active_command is None
                else (
                    f"seq={self.active_command.sequence}, "
                    f"command="
                    f"{COMMAND_NAMES[self.active_command.command]}"
                )
            )

            print(
                "\n[STATS]"
                f"\n  valid telemetry : "
                f"{self.valid_telemetry_count}"
                f"\n  telemetry errors: "
                f"{self.telemetry_crc_error_count}"
                f"\n  discarded bytes : "
                f"{self.discarded_rx_byte_count}"
                f"\n  CMD delay       : "
                f"{self.command_delay_s * 1000.0:.1f} ms"
                f"\n  active command  : {active_text}"
            )

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
            "\n  delay <ms>"
            "\n  stats"
            "\n  resend"
            "\n  clear_timeout"
            "\n  quit"
        )

        try:
            while self.running:
                user_input = input(
                    "\ncomet> "
                ).strip().lower()

                if user_input in COMMANDS:
                    self.request_command(
                        COMMANDS[user_input]
                    )

                elif user_input.startswith("delay"):
                    self.set_delay(user_input)

                elif user_input == "stats":
                    self.show_stats()

                elif user_input == "resend":
                    self.resend()

                elif user_input == "clear_timeout":
                    self.clear_timeout()

                elif user_input in {"quit", "exit"}:
                    break

                elif user_input:
                    print("\n[CLI] Unknown command")

        except KeyboardInterrupt:
            print("\n[CLI] Interrupted")

        finally:
            self.running = False

            try:
                self.serial.close()
            except serial.SerialException:
                pass

            print("[LINK] Closed")


def main() -> None:
    try:
        test = CometLoRaGroundTest()
        test.start()

    except serial.SerialException as exc:
        print(f"[ERROR] Could not open {PORT}: {exc}")


if __name__ == "__main__":
    main()
