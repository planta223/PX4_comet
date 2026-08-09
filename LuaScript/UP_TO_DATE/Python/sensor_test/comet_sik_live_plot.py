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
from pymavlink import mavutil

G0_MPS2 = 9.80665

# comet_config.lua와 동기화할 전이 임계값
LAUNCH_ACCEL_THRESHOLD_MPS2 = 25.0
DEPLOY_MIN_ALTITUDE_M = 30.0
DEPLOY_MIN_DESCENT_SPEED_MPS = 1.0
LANDED_MAX_ALTITUDE_M = 5.0
LANDED_MAX_ABS_VD_MPS = 1.5
LANDED_MIN_ACCEL_MPS2 = 8.0
LANDED_MAX_ACCEL_MPS2 = 12.0


@dataclass
class LatestSample:
    host_time_s: float = 0.0
    boot_time_ms: Optional[int] = None
    acceleration_mps2: Optional[float] = None
    acceleration_source: str = "NONE"
    altitude_m: Optional[float] = None
    velocity_d_mps: Optional[float] = None
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None


class TimeSeries:
    def __init__(self, maxlen: int) -> None:
        self.t: Deque[float] = deque(maxlen=maxlen)
        self.y: Deque[float] = deque(maxlen=maxlen)

    def append(self, t: float, y: Optional[float]) -> None:
        if y is None or not math.isfinite(y):
            return
        self.t.append(t)
        self.y.append(y)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Parse SiK MAVLink telemetry and plot COMET transition sensors."
    )
    parser.add_argument("--port", required=True, help="SiK port, e.g. COM7 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--window", type=float, default=30.0)
    parser.add_argument("--plot-hz", type=float, default=10.0)
    parser.add_argument("--position-hz", type=float, default=10.0)
    parser.add_argument("--imu-hz", type=float, default=20.0)
    parser.add_argument("--attitude-hz", type=float, default=10.0)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--no-request-rates", action="store_true")
    return parser.parse_args()


def request_message_interval(connection: mavutil.mavfile, message_id: int, hz: float) -> None:
    if hz <= 0:
        return
    interval_us = int(round(1_000_000.0 / hz))
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        float(interval_us),
        0, 0, 0, 0, 0,
    )


def configure_message_rates(connection: mavutil.mavfile, position_hz: float, imu_hz: float, attitude_hz: float) -> None:
    requests = (
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, position_hz),
        (mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, imu_hz),
        (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, imu_hz),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, attitude_hz),
    )
    for message_id, hz in requests:
        request_message_interval(connection, message_id, hz)
        time.sleep(0.05)


def normalize_yaw_deg(yaw_deg: float) -> float:
    return ((yaw_deg + 180.0) % 360.0) - 180.0


def update_sample_from_message(sample: LatestSample, message: object) -> bool:
    message_type = message.get_type()
    updated = False

    if message_type == "HIGHRES_IMU":
        ax = float(message.xacc)
        ay = float(message.yacc)
        az = float(message.zacc)
        if all(math.isfinite(v) for v in (ax, ay, az)):
            sample.acceleration_mps2 = math.sqrt(ax * ax + ay * ay + az * az)
            sample.acceleration_source = "HIGHRES_IMU"
            updated = True

    elif message_type == "RAW_IMU":
        # RAW_IMU xacc/yacc/zacc units: mG
        if sample.acceleration_source != "HIGHRES_IMU":
            ax = float(message.xacc) * G0_MPS2 / 1000.0
            ay = float(message.yacc) * G0_MPS2 / 1000.0
            az = float(message.zacc) * G0_MPS2 / 1000.0
            if all(math.isfinite(v) for v in (ax, ay, az)):
                sample.acceleration_mps2 = math.sqrt(ax * ax + ay * ay + az * az)
                sample.acceleration_source = "RAW_IMU"
                updated = True

    elif message_type == "GLOBAL_POSITION_INT":
        sample.boot_time_ms = int(message.time_boot_ms)
        sample.altitude_m = float(message.relative_alt) / 1000.0
        sample.velocity_d_mps = float(message.vz) / 100.0
        updated = True

    elif message_type == "ATTITUDE":
        sample.boot_time_ms = int(message.time_boot_ms)
        sample.roll_deg = math.degrees(float(message.roll))
        sample.pitch_deg = math.degrees(float(message.pitch))
        sample.yaw_deg = normalize_yaw_deg(math.degrees(float(message.yaw)))
        updated = True

    if updated:
        sample.host_time_s = time.monotonic()
    return updated


def make_csv_writer(path: Path) -> tuple[object, csv.DictWriter]:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", newline="", encoding="utf-8")
    fields = [
        "host_elapsed_s", "host_unix_s", "message_type", "boot_time_ms",
        "acceleration_mps2", "acceleration_source", "altitude_m",
        "velocity_d_mps", "roll_deg", "pitch_deg", "yaw_deg",
    ]
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    handle.flush()
    return handle, writer


def write_csv_row(writer: csv.DictWriter, start: float, sample: LatestSample, message_type: str) -> None:
    def value(v: Optional[float]) -> str:
        return "" if v is None else f"{v:.6f}"

    writer.writerow({
        "host_elapsed_s": f"{time.monotonic() - start:.6f}",
        "host_unix_s": f"{time.time():.6f}",
        "message_type": message_type,
        "boot_time_ms": "" if sample.boot_time_ms is None else sample.boot_time_ms,
        "acceleration_mps2": value(sample.acceleration_mps2),
        "acceleration_source": sample.acceleration_source,
        "altitude_m": value(sample.altitude_m),
        "velocity_d_mps": value(sample.velocity_d_mps),
        "roll_deg": value(sample.roll_deg),
        "pitch_deg": value(sample.pitch_deg),
        "yaw_deg": value(sample.yaw_deg),
    })


def configure_plot_window(title: str, ylabel: str):
    figure = plt.figure()
    axis = figure.add_axes([0.12, 0.12, 0.82, 0.80])
    figure.canvas.manager.set_window_title(title)
    axis.set_title(title)
    axis.set_xlabel("Elapsed time [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True)
    line, = axis.plot([], [], label="Measured")
    axis.legend(loc="upper left")
    return figure, axis, line


def update_axis(axis, line, series: TimeSeries, now_s: float, window_s: float, minimum_span: float) -> None:
    if not series.t:
        return
    x = list(series.t)
    y = list(series.y)
    line.set_data(x, y)
    axis.set_xlim(max(0.0, now_s - window_s), max(window_s, now_s))
    y_min = min(y)
    y_max = max(y)
    span = max(y_max - y_min, minimum_span)
    margin = span * 0.15
    axis.set_ylim(y_min - margin, y_max + margin)


def main() -> int:
    args = parse_args()
    if args.plot_hz <= 0:
        print("[ERROR] --plot-hz must be positive", file=sys.stderr)
        return 2

    csv_path = args.csv or Path(time.strftime("comet_sik_%Y%m%d_%H%M%S.csv"))

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

    print(f"[LINK] Connected: sysid={connection.target_system}, compid={connection.target_component}")

    if not args.no_request_rates:
        configure_message_rates(connection, args.position_hz, args.imu_hz, args.attitude_hz)
        print("[LINK] Requested MAVLink message intervals")

    file_handle, csv_writer = make_csv_writer(csv_path)
    print(f"[LOG] CSV: {csv_path.resolve()}")

    max_points = max(100, int(args.window * max(args.imu_hz, args.position_hz, args.plot_hz) * 2))
    acceleration_series = TimeSeries(max_points)
    altitude_series = TimeSeries(max_points)
    velocity_d_series = TimeSeries(max_points)

    acceleration_figure, acceleration_axis, acceleration_line = configure_plot_window(
        "COMET acceleration magnitude", "Acceleration [m/s²]"
    )
    acceleration_axis.axhline(LAUNCH_ACCEL_THRESHOLD_MPS2, linestyle="--", label="Launch threshold")
    acceleration_axis.axhline(LANDED_MIN_ACCEL_MPS2, linestyle=":", label="Landed minimum")
    acceleration_axis.axhline(LANDED_MAX_ACCEL_MPS2, linestyle=":", label="Landed maximum")
    acceleration_axis.legend(loc="upper left")

    altitude_figure, altitude_axis, altitude_line = configure_plot_window(
        "COMET home-relative altitude", "Altitude [m]"
    )
    altitude_axis.axhline(DEPLOY_MIN_ALTITUDE_M, linestyle="--", label="Deploy minimum")
    altitude_axis.axhline(LANDED_MAX_ALTITUDE_M, linestyle=":", label="Landed maximum")
    altitude_axis.legend(loc="upper left")

    velocity_figure, velocity_axis, velocity_line = configure_plot_window(
        "COMET down velocity", "VD [m/s], down-positive"
    )
    velocity_axis.axhline(DEPLOY_MIN_DESCENT_SPEED_MPS, linestyle="--", label="Deploy descent threshold")
    velocity_axis.axhline(LANDED_MAX_ABS_VD_MPS, linestyle=":", label="Landed +VD limit")
    velocity_axis.axhline(-LANDED_MAX_ABS_VD_MPS, linestyle=":", label="Landed -VD limit")
    velocity_axis.legend(loc="upper left")

    plt.show(block=False)

    start = time.monotonic()
    last_plot = 0.0
    last_console = 0.0
    rows_since_flush = 0
    running = True

    def stop_handler(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    sample = LatestSample()

    try:
        while running:
            if not all(plt.fignum_exists(fig.number) for fig in (acceleration_figure, altitude_figure, velocity_figure)):
                break

            message = connection.recv_match(
                type=["HIGHRES_IMU", "RAW_IMU", "GLOBAL_POSITION_INT", "ATTITUDE"],
                blocking=True,
                timeout=0.05,
            )

            if message is not None and message.get_type() != "BAD_DATA":
                message_type = message.get_type()
                if update_sample_from_message(sample, message):
                    elapsed = time.monotonic() - start
                    if message_type in {"HIGHRES_IMU", "RAW_IMU"}:
                        acceleration_series.append(elapsed, sample.acceleration_mps2)
                    elif message_type == "GLOBAL_POSITION_INT":
                        altitude_series.append(elapsed, sample.altitude_m)
                        velocity_d_series.append(elapsed, sample.velocity_d_mps)

                    write_csv_row(csv_writer, start, sample, message_type)
                    rows_since_flush += 1

            now = time.monotonic()
            elapsed = now - start

            if now - last_plot >= 1.0 / args.plot_hz:
                update_axis(acceleration_axis, acceleration_line, acceleration_series, elapsed, args.window, 5.0)
                update_axis(altitude_axis, altitude_line, altitude_series, elapsed, args.window, 5.0)
                update_axis(velocity_axis, velocity_line, velocity_d_series, elapsed, args.window, 2.0)

                acceleration_figure.canvas.draw_idle()
                altitude_figure.canvas.draw_idle()
                velocity_figure.canvas.draw_idle()
                plt.pause(0.001)
                last_plot = now

            if now - last_console >= 1.0:
                def fmt(v: Optional[float]) -> str:
                    return "N/A" if v is None else f"{v:.2f}"

                print(
                    "\r[DATA] "
                    f"A={fmt(sample.acceleration_mps2)} ({sample.acceleration_source})  "
                    f"ALT={fmt(sample.altitude_m)} m  "
                    f"VD={fmt(sample.velocity_d_mps)} m/s  "
                    f"RPY=({fmt(sample.roll_deg)}, {fmt(sample.pitch_deg)}, {fmt(sample.yaw_deg)})",
                    end="",
                    flush=True,
                )
                last_console = now

            if rows_since_flush >= 50:
                file_handle.flush()
                rows_since_flush = 0

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[LINK] Closing")
        file_handle.flush()
        file_handle.close()
        connection.close()
        plt.close("all")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
