#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pymavlink import mavutil


# ---------------------------------------------------------------------
# COMET definitions
# ---------------------------------------------------------------------

STATE_NAMES = {
    0: "READY",
    1: "ASCENT",
    2: "DEPLOY",
    3: "DESCENT",
    4: "LANDED",
}

VALID_BITS = {
    0x01: "POS",
    0x02: "VEL",
    0x04: "ATT",
    0x08: "ACC",
    0x10: "ALT",
}

FAULT_BITS = {
    0x01: "ACCEL_SENSOR_INVALID",
    0x02: "ALTITUDE_INVALID",
    0x04: "VELOCITY_INVALID",
    0x08: "ASCENT_TIMEOUT",
    0x10: "DEPLOY_COMMAND_FAILED",
    0x20: "LORA_UNAVAILABLE",
}

# comet_config.lua와 맞춰 두십시오.
LAUNCH_ACCEL_THRESHOLD_MPS2 = 20.0
DEPLOY_MIN_ALTITUDE_M = 50.0
DEPLOY_MIN_DESCENT_SPEED_MPS = 5.0
LANDED_MAX_ALTITUDE_M = 5.0
LANDED_MAX_ABS_VERTICAL_SPEED_MPS = 1.5
LANDED_MIN_ACCEL_MPS2 = 8.0
LANDED_MAX_ACCEL_MPS2 = 12.0

EXPECTED_NAMES = {
    "C_ACC",
    "C_ALT",
    "C_VD",
    "C_STATE",
    "C_VALID",
    "C_FAULT",
    "C_MAXALT",
    "C_DEPLOY",
}


# ---------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------

@dataclass
class LatestData:
    acceleration_mps2: Optional[float] = None
    altitude_m: Optional[float] = None  # startup barometric reference-relative, up-positive
    velocity_d_mps: Optional[float] = None
    state: Optional[int] = None
    valid_flags: int = 0
    fault_flags: int = 0
    maximum_altitude_m: Optional[float] = None
    deployed: bool = False

    last_packet_monotonic: float = 0.0
    last_complete_sample_monotonic: float = 0.0
    received_message_count: int = 0


class TimeSeries:
    def __init__(self, maxlen: int) -> None:
        self.time_s: Deque[float] = deque(maxlen=maxlen)
        self.value: Deque[float] = deque(maxlen=maxlen)

    def append(self, time_s: float, value: Optional[float]) -> None:
        if value is None or not math.isfinite(value):
            return
        self.time_s.append(time_s)
        self.value.append(value)

    def data(self) -> tuple[list[float], list[float]]:
        return list(self.time_s), list(self.value)


# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Receive COMET NAMED_VALUE_FLOAT messages over a SiK MAVLink link "
            "and display live transition sensor data."
        )
    )
    parser.add_argument(
        "--port",
        default="COM19",
        help="SiK serial port; default: COM19",
    )
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument(
        "--window",
        type=float,
        default=30.0,
        help="Visible plot window in seconds",
    )
    parser.add_argument(
        "--plot-hz",
        type=float,
        default=10.0,
        help="Display refresh rate",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Link timeout indication in seconds",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="CSV output path; default is timestamped file",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------
# MAVLink parsing
# ---------------------------------------------------------------------

def decode_name(raw_name: object) -> str:
    if isinstance(raw_name, bytes):
        return raw_name.decode("ascii", errors="ignore").rstrip("\x00")
    return str(raw_name).rstrip("\x00")


def update_latest_from_message(latest: LatestData, message: object) -> Optional[str]:
    if message.get_type() != "NAMED_VALUE_FLOAT":
        return None

    name = decode_name(message.name)
    if name not in EXPECTED_NAMES:
        return None

    value = float(message.value)
    if not math.isfinite(value):
        return None

    if name == "C_ACC":
        latest.acceleration_mps2 = value

    elif name == "C_ALT":
        latest.altitude_m = value

    elif name == "C_VD":
        latest.velocity_d_mps = value

    elif name == "C_STATE":
        latest.state = int(round(value))

    elif name == "C_VALID":
        latest.valid_flags = int(round(value)) & 0xFF

    elif name == "C_FAULT":
        latest.fault_flags = int(round(value)) & 0xFF

    elif name == "C_MAXALT":
        latest.maximum_altitude_m = value

    elif name == "C_DEPLOY":
        latest.deployed = value >= 0.5

    latest.last_packet_monotonic = time.monotonic()
    latest.received_message_count += 1
    return name


# ---------------------------------------------------------------------
# Status formatting
# ---------------------------------------------------------------------

def format_optional(value: Optional[float], digits: int = 2) -> str:
    if value is None or not math.isfinite(value):
        return "N/A"
    return f"{value:.{digits}f}"


def valid_status(flags: int) -> str:
    items = [
        f"{name}:{'OK' if flags & bit else 'NO'}"
        for bit, name in VALID_BITS.items()
    ]
    return "  ".join(items)


def fault_status(flags: int) -> str:
    if flags == 0:
        return "NONE"

    active = [
        name for bit, name in FAULT_BITS.items()
        if flags & bit
    ]
    unknown = flags & ~sum(FAULT_BITS.keys())
    if unknown:
        active.append(f"UNKNOWN_0x{unknown:02X}")
    return ", ".join(active)


# ---------------------------------------------------------------------
# CSV
# ---------------------------------------------------------------------

def create_csv_writer(path: Path) -> tuple[object, csv.DictWriter]:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", newline="", encoding="utf-8")
    fields = [
        "host_unix_s",
        "elapsed_s",
        "message_name",
        "acceleration_mps2",
        "altitude_m",
        "velocity_d_mps",
        "state",
        "state_name",
        "valid_flags",
        "fault_flags",
        "maximum_altitude_m",
        "deployed",
    ]
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    handle.flush()
    return handle, writer


def write_csv_row(
    writer: csv.DictWriter,
    start_monotonic: float,
    latest: LatestData,
    message_name: str,
) -> None:
    writer.writerow({
        "host_unix_s": f"{time.time():.6f}",
        "elapsed_s": f"{time.monotonic() - start_monotonic:.6f}",
        "message_name": message_name,
        "acceleration_mps2": (
            "" if latest.acceleration_mps2 is None
            else f"{latest.acceleration_mps2:.6f}"
        ),
        "altitude_m": (
            "" if latest.altitude_m is None
            else f"{latest.altitude_m:.6f}"
        ),
        "velocity_d_mps": (
            "" if latest.velocity_d_mps is None
            else f"{latest.velocity_d_mps:.6f}"
        ),
        "state": "" if latest.state is None else latest.state,
        "state_name": STATE_NAMES.get(latest.state, "UNKNOWN"),
        "valid_flags": latest.valid_flags,
        "fault_flags": latest.fault_flags,
        "maximum_altitude_m": (
            "" if latest.maximum_altitude_m is None
            else f"{latest.maximum_altitude_m:.6f}"
        ),
        "deployed": int(latest.deployed),
    })


# ---------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------

def set_dynamic_limits(
    axis,
    values: list[float],
    minimum_span: float,
    include_values: tuple[float, ...] = (),
) -> None:
    candidates = list(values) + list(include_values)
    if not candidates:
        return

    low = min(candidates)
    high = max(candidates)
    span = max(high - low, minimum_span)
    margin = span * 0.12
    axis.set_ylim(low - margin, high + margin)


def main() -> int:
    args = parse_args()

    if args.plot_hz <= 0:
        print("[ERROR] --plot-hz must be greater than zero", file=sys.stderr)
        return 2

    if args.window <= 0:
        print("[ERROR] --window must be greater than zero", file=sys.stderr)
        return 2

    csv_path = args.csv or Path(
        time.strftime("comet_sik_debug_%Y%m%d_%H%M%S.csv")
    )

    print(f"[LINK] Opening {args.port} @ {args.baud}")
    connection = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        autoreconnect=True,
        source_system=255,
    )

    print("[LINK] Waiting for HEARTBEAT")
    heartbeat = connection.wait_heartbeat(timeout=15)
    if heartbeat is None:
        print("[ERROR] HEARTBEAT timeout", file=sys.stderr)
        connection.close()
        return 1

    print(
        "[LINK] Connected "
        f"sysid={connection.target_system}, "
        f"compid={connection.target_component}"
    )
    print(
        "[LINK] Waiting for C_ACC, "
        "C_ALT(barometric-relative), C_VD, C_STATE ..."
    )

    csv_handle, csv_writer = create_csv_writer(csv_path)
    print(f"[LOG] CSV: {csv_path.resolve()}")

    start_monotonic = time.monotonic()
    latest = LatestData()

    max_points = max(300, int(args.window * max(args.plot_hz, 10.0) * 4))
    acceleration_series = TimeSeries(max_points)
    altitude_series = TimeSeries(max_points)
    velocity_series = TimeSeries(max_points)
    state_series = TimeSeries(max_points)

    # Dashboard layout
    figure = plt.figure(figsize=(13, 8))
    try:
        figure.canvas.manager.set_window_title("COMET SiK Live Debug")
    except Exception:
        pass

    grid = figure.add_gridspec(
        4,
        2,
        height_ratios=[1.0, 1.0, 1.0, 0.65],
        width_ratios=[3.2, 1.4],
        hspace=0.42,
        wspace=0.25,
    )

    ax_acc = figure.add_subplot(grid[0, 0])
    ax_alt = figure.add_subplot(grid[1, 0])
    ax_vd = figure.add_subplot(grid[2, 0])
    ax_state = figure.add_subplot(grid[3, 0])
    ax_info = figure.add_subplot(grid[:, 1])
    ax_info.axis("off")

    # Lines
    acc_line, = ax_acc.plot([], [], label="C_ACC")
    alt_line, = ax_alt.plot([], [], label="C_ALT")
    vd_line, = ax_vd.plot([], [], label="C_VD")
    state_line, = ax_state.step([], [], where="post", label="C_STATE")

    # Thresholds
    ax_acc.axhline(
        LAUNCH_ACCEL_THRESHOLD_MPS2,
        linestyle="--",
        label="Launch threshold",
    )
    ax_acc.axhline(
        LANDED_MIN_ACCEL_MPS2,
        linestyle=":",
        label="Landed min",
    )
    ax_acc.axhline(
        LANDED_MAX_ACCEL_MPS2,
        linestyle=":",
        label="Landed max",
    )

    ax_alt.axhline(
        DEPLOY_MIN_ALTITUDE_M,
        linestyle="--",
        label="Deploy min altitude",
    )
    ax_alt.axhline(
        LANDED_MAX_ALTITUDE_M,
        linestyle=":",
        label="Landed max altitude",
    )

    ax_vd.axhline(
        DEPLOY_MIN_DESCENT_SPEED_MPS,
        linestyle="--",
        label="Deploy descent threshold",
    )
    ax_vd.axhline(
        LANDED_MAX_ABS_VERTICAL_SPEED_MPS,
        linestyle=":",
        label="Landed +VD limit",
    )
    ax_vd.axhline(
        -LANDED_MAX_ABS_VERTICAL_SPEED_MPS,
        linestyle=":",
        label="Landed -VD limit",
    )

    # Axis formatting
    ax_acc.set_ylabel("Acceleration [m/s²]")
    ax_alt.set_ylabel("Barometric relative altitude [m]\nUp positive")
    ax_vd.set_ylabel("VD [m/s]\nDown positive")
    ax_state.set_ylabel("State")
    ax_state.set_xlabel("Elapsed time [s]")
    ax_state.set_yticks(list(STATE_NAMES.keys()))
    ax_state.set_yticklabels(list(STATE_NAMES.values()))
    ax_state.set_ylim(-0.5, 4.5)

    for axis in (ax_acc, ax_alt, ax_vd, ax_state):
        axis.grid(True)
        axis.legend(loc="upper left")
        axis.set_xlim(0.0, args.window)

    info_text = ax_info.text(
        0.02,
        0.98,
        "Waiting for COMET data...",
        va="top",
        ha="left",
        family="monospace",
        fontsize=10,
        transform=ax_info.transAxes,
    )

    footer_text = figure.text(
        0.01,
        0.01,
        "",
        ha="left",
        va="bottom",
        family="monospace",
        fontsize=9,
    )

    running = True
    rows_since_flush = 0

    def stop_handler(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)

    def receive_available_messages() -> None:
        nonlocal rows_since_flush

        # Do not block the GUI. Drain currently available COM messages.
        for _ in range(200):
            message = connection.recv_match(
                type=["NAMED_VALUE_FLOAT"],
                blocking=False,
            )
            if message is None:
                break

            if message.get_type() == "BAD_DATA":
                continue

            name = update_latest_from_message(latest, message)
            if name is None:
                continue

            elapsed = time.monotonic() - start_monotonic

            if name == "C_ACC":
                acceleration_series.append(
                    elapsed,
                    latest.acceleration_mps2,
                )

            elif name == "C_ALT":
                altitude_series.append(
                    elapsed,
                    latest.altitude_m,
                )

            elif name == "C_VD":
                velocity_series.append(
                    elapsed,
                    latest.velocity_d_mps,
                )

            elif name == "C_STATE":
                state_series.append(
                    elapsed,
                    float(latest.state)
                    if latest.state is not None
                    else None,
                )

            write_csv_row(
                csv_writer,
                start_monotonic,
                latest,
                name,
            )
            rows_since_flush += 1

    def update_plot(_frame_index: int):
        nonlocal rows_since_flush

        if not running:
            plt.close(figure)
            return ()

        receive_available_messages()

        now = time.monotonic()
        elapsed = now - start_monotonic
        left = max(0.0, elapsed - args.window)
        right = max(args.window, elapsed)

        acc_t, acc_y = acceleration_series.data()
        alt_t, alt_y = altitude_series.data()
        vd_t, vd_y = velocity_series.data()
        state_t, state_y = state_series.data()

        acc_line.set_data(acc_t, acc_y)
        alt_line.set_data(alt_t, alt_y)
        vd_line.set_data(vd_t, vd_y)
        state_line.set_data(state_t, state_y)

        for axis in (ax_acc, ax_alt, ax_vd, ax_state):
            axis.set_xlim(left, right)

        set_dynamic_limits(
            ax_acc,
            acc_y,
            minimum_span=5.0,
            include_values=(
                LAUNCH_ACCEL_THRESHOLD_MPS2,
                LANDED_MIN_ACCEL_MPS2,
                LANDED_MAX_ACCEL_MPS2,
            ),
        )
        set_dynamic_limits(
            ax_alt,
            alt_y,
            minimum_span=10.0,
            include_values=(
                DEPLOY_MIN_ALTITUDE_M,
                LANDED_MAX_ALTITUDE_M,
            ),
        )
        set_dynamic_limits(
            ax_vd,
            vd_y,
            minimum_span=4.0,
            include_values=(
                DEPLOY_MIN_DESCENT_SPEED_MPS,
                LANDED_MAX_ABS_VERTICAL_SPEED_MPS,
                -LANDED_MAX_ABS_VERTICAL_SPEED_MPS,
            ),
        )

        packet_age = (
            math.inf
            if latest.last_packet_monotonic <= 0
            else now - latest.last_packet_monotonic
        )
        link_ok = packet_age <= args.timeout
        link_status = "OK" if link_ok else "TIMEOUT"

        state_name = STATE_NAMES.get(latest.state, "UNKNOWN")
        state_number = "N/A" if latest.state is None else str(latest.state)

        info = (
            f"LINK      : {link_status}\n"
            f"Packet age: "
            f"{'N/A' if not math.isfinite(packet_age) else f'{packet_age:.2f} s'}\n"
            f"Messages  : {latest.received_message_count}\n"
            f"\n"
            f"STATE     : {state_name} ({state_number})\n"
            f"DEPLOY CMD: {'YES' if latest.deployed else 'NO'}\n"
            f"\n"
            f"ACC       : {format_optional(latest.acceleration_mps2)} m/s²\n"
            f"ALT BARO  : {format_optional(latest.altitude_m)} m\n"
            f"VD        : {format_optional(latest.velocity_d_mps)} m/s\n"
            f"MAX ALT   : {format_optional(latest.maximum_altitude_m)} m\n"
            f"\n"
            f"VALID     : 0x{latest.valid_flags:02X}\n"
            f"{valid_status(latest.valid_flags)}\n"
            f"\n"
            f"FAULT     : 0x{latest.fault_flags:02X}\n"
            f"{fault_status(latest.fault_flags)}\n"
            f"\n"
            f"Thresholds\n"
            f"Launch ACC >= {LAUNCH_ACCEL_THRESHOLD_MPS2:.1f}\n"
            f"Deploy ALT >= {DEPLOY_MIN_ALTITUDE_M:.1f}\n"
            f"Deploy VD  >= {DEPLOY_MIN_DESCENT_SPEED_MPS:.1f}\n"
        )
        info_text.set_text(info)

        footer_text.set_text(
            f"Port={args.port}  Baud={args.baud}  "
            f"CSV={csv_path.name}  "
            f"Ctrl+C or close window to stop"
        )

        if rows_since_flush >= 50:
            csv_handle.flush()
            rows_since_flush = 0

        return (
            acc_line,
            alt_line,
            vd_line,
            state_line,
            info_text,
            footer_text,
        )

    interval_ms = max(20, int(round(1000.0 / args.plot_hz)))
    animation = FuncAnimation(
        figure,
        update_plot,
        interval=interval_ms,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        csv_handle.flush()
        csv_handle.close()
        connection.close()
        print("\n[LINK] Closed")
        print(f"[LOG] Saved: {csv_path.resolve()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
