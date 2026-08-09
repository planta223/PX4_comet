# -*- coding: utf-8 -*-
"""
LoRa 실통신 파이프라인 (DX-LR03-900T30D, 투명전송 모드).

FC측 comet_main.lua와 커스텀 바이너리 프로토콜로 통신한다. LoRa 모듈
자체는 프레이밍을 안 해주므로(투명전송), 헤더 바이트 + CRC-8로 우리가
직접 연속 바이트스트림에서 프레임을 잘라낸다 (_FrameParser).

두 프레임 (config.py의 LORA_* 상수와 comet_config.lua의 CONFIG.LORA에 대응):
  - TELEMETRY (0xA5, 36B, FC->GCS): 절대 위경도, 기압 상대고도, NED 속도,
    자세, 상태/고장 비트와 마지막 명령 ACK를 포함한다.
  - COMMAND   (0x5A, 4B, GCS->FC): sequence와 명령 코드.

별도 ACK 프레임은 없다. FC가 처리한 마지막 명령의 sequence/result가 이후
telemetry에 반복되므로, 명령 대기 중에도 telemetry를 정상 파싱·기록한다.

sik.py와 동일하게 결과는 큐에 push만 한다 (누가 구독하는지 모름).
커맨드 송신은 시리얼 포트를 소유한 이 파이프라인 스레드만 수행한다 -
GUI 스레드는 command_queue에 요청만 넣는다 (enqueue_command). 시리얼
포트에 두 스레드가 동시에 쓰면 레이스 컨디션이 생기기 때문이다.
"""

import csv
import os
import queue
import struct
import time
from datetime import datetime

import serial

import config
import shared_state
import stats

# 5x uint8, uint32, 3x int32, 6x int16, ack_seq, ack_result, crc8.
_TELEMETRY_STRUCT = struct.Struct("<5BIiii6h3B")
assert _TELEMETRY_STRUCT.size == config.LORA_TELEMETRY_LENGTH


def _crc8(data) -> int:
    """comet_main.lua의 calculate_crc8()과 동일 (poly=0x07, init=0x00, 반사/최종XOR 없음)."""
    crc = 0x00
    poly = config.LORA_CRC_POLYNOMIAL
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class _FrameParser:
    """
    투명전송 스트림(프레이밍 없음)에서 헤더 바이트 + CRC-8로 완전한
    프레임을 잘라낸다. 헤더가 아닌 바이트나 CRC 불일치는 1바이트씩만
    버리고 재동기화한다 (오탐 프레임 하나 때문에 뒤따라오는 진짜 프레임
    까지 놓치지 않도록).
    """

    def __init__(self, cfg):
        self._buf = bytearray()
        self._frame_lengths = {
            cfg.LORA_TELEMETRY_HEADER: cfg.LORA_TELEMETRY_LENGTH,
        }
        # 헤더는 맞았는데 CRC가 틀린 경우만 센다(=수신은 됐는데 깨진 패킷).
        # 헤더 자체가 안 맞는 바이트는 그냥 노이즈/재동기화라 카운트 안 함.
        self.crc_fail_count = 0

    def feed(self, data) -> list:
        self._buf.extend(data)
        frames = []
        while self._buf:
            header = self._buf[0]
            frame_len = self._frame_lengths.get(header)
            if frame_len is None:
                del self._buf[0]
                continue
            if len(self._buf) < frame_len:
                break
            candidate = bytes(self._buf[:frame_len])
            if _crc8(candidate[:-1]) == candidate[-1]:
                frames.append(candidate)
                del self._buf[:frame_len]
            else:
                self.crc_fail_count += 1
                del self._buf[0]
        return frames


def parse_lora_packet(raw_bytes) -> dict:
    """
    raw_bytes: 최신 Lua 형식의 완전한 telemetry frame. 방어적으로 길이와
    CRC를 다시 검사해 parser를 우회한 호출도 불완전 프레임을 받지 않는다.
    """
    if len(raw_bytes) != config.LORA_TELEMETRY_LENGTH:
        raise ValueError(f"잘못된 LoRa telemetry 길이: {len(raw_bytes)}")
    if raw_bytes[0] != config.LORA_TELEMETRY_HEADER:
        raise ValueError(f"알 수 없는 LoRa 프레임 헤더: {raw_bytes[0]:#x}")
    if _crc8(raw_bytes[:-1]) != raw_bytes[-1]:
        raise ValueError("LoRa telemetry CRC 불일치")
    return _parse_telemetry(raw_bytes)


def _parse_telemetry(raw_bytes) -> dict:
    (
        _header, sequence, flight_state, status_flags, fault_flags, time_ms,
        latitude_raw, longitude_raw, altitude_raw,
        velocity_n_raw, velocity_e_raw, velocity_d_raw,
        roll_raw, pitch_raw, yaw_raw,
        ack_sequence, ack_result, _crc,
    ) = _TELEMETRY_STRUCT.unpack(raw_bytes)

    state_names = config.LORA_FLIGHT_STATE_NAMES
    state_name = state_names[flight_state] if 0 <= flight_state < len(state_names) else "UNKNOWN"

    # fault_flags 비트 하나하나를 사람이 읽을 이름으로 풀어낸다 - "에러가
    # 났다"만이 아니라 "왜"까지 GCS 화면/CSV에 그대로 남기기 위함
    # (config.py의 LORA_FAULT_FLAG_NAMES 참고).
    faults = [name for bit, name in config.LORA_FAULT_FLAG_NAMES.items() if fault_flags & bit]
    statuses = [name for bit, name in config.LORA_STATUS_FLAG_NAMES.items() if status_flags & bit]

    latitude = latitude_raw / config.LORA_LAT_LON_SCALE
    longitude = longitude_raw / config.LORA_LAT_LON_SCALE
    altitude = altitude_raw / config.LORA_ALTITUDE_SCALE
    position_valid = (
        bool(status_flags & 0x01)
        and not (latitude_raw == 0 and longitude_raw == 0)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    )
    altitude_valid = (
        bool(status_flags & 0x10)
        and abs(altitude) < config.LORA_ALTITUDE_SATURATION_M
    )

    return {
        "frame_type": "telemetry",
        "sequence": sequence,
        "seq": sequence,  # 기존 내부 통계 API 호환
        "flight_state": state_name,
        "flight_state_code": flight_state,
        "status_flags": status_flags,
        "statuses": statuses,
        "fault_flags": fault_flags,
        "faults": faults,
        "time_ms": time_ms,
        "latitude": latitude,
        "longitude": longitude,
        "position_valid": position_valid,
        "altitude": altitude,
        "alt": altitude,  # 기존 소비자 키 호환
        "altitude_valid": altitude_valid,
        "velocity_n": velocity_n_raw / config.LORA_VELOCITY_SCALE,
        "velocity_e": velocity_e_raw / config.LORA_VELOCITY_SCALE,
        "velocity_d": velocity_d_raw / config.LORA_VELOCITY_SCALE,
        "roll": roll_raw / config.LORA_ATTITUDE_SCALE,
        "pitch": pitch_raw / config.LORA_ATTITUDE_SCALE,
        "yaw": yaw_raw / config.LORA_ATTITUDE_SCALE,
        "ack_sequence": ack_sequence,
        "ack_result": ack_result,
        "ack_result_name": config.LORA_ACK_RESULT_NAMES.get(ack_result, "UNKNOWN"),
    }


def _build_command_frame(command_name: str, seq: int) -> bytes:
    command_code = config.LORA_COMMAND[command_name]
    payload = bytes([config.LORA_COMMAND_HEADER, seq & 0xFF, command_code])
    return payload + bytes([_crc8(payload)])


def _ensure_csv_writer(cfg, output_dir=None):
    output_dir = output_dir or cfg.CSV_OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)
    filename = datetime.now().strftime(cfg.LORA_CSV_FILENAME_PATTERN)
    filepath = os.path.join(output_dir, filename)
    columns = [
        "timestamp", "sequence", "flight_state", "status_flags", "fault_flags", "time_ms",
        "latitude", "longitude", "pos_n", "pos_e", "altitude",
        "velocity_n", "velocity_e", "velocity_d", "roll", "pitch", "yaw",
        "ack_sequence", "ack_result", "ack_result_name", "faults",
    ]

    # 같은 날짜의 구형 CSV가 있으면 서로 다른 스키마의 행을 섞지 않는다.
    if os.path.exists(filepath) and os.path.getsize(filepath) > 0:
        with open(filepath, newline="", encoding="utf-8") as existing:
            existing_header = next(csv.reader(existing), [])
        if existing_header != columns:
            stem, extension = os.path.splitext(filepath)
            filepath = f"{stem}_protocol_v2{extension}"

    is_new = not os.path.exists(filepath) or os.path.getsize(filepath) == 0

    f = open(filepath, mode="a", newline="", encoding="utf-8")
    writer = csv.writer(f)
    if is_new:
        writer.writerow(columns)
    return f, writer


def _ensure_stats_csv_writer(cfg, output_dir=None):
    output_dir = output_dir or cfg.CSV_OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)
    filename = datetime.now().strftime(cfg.LORA_STATS_CSV_FILENAME_PATTERN)
    filepath = os.path.join(output_dir, filename)
    is_new = not os.path.exists(filepath)

    f = open(filepath, mode="a", newline="", encoding="utf-8")
    writer = csv.writer(f)
    if is_new:
        writer.writerow([
            "timestamp", "rate_hz", "jitter_ms", "seq_loss_pct", "crc_fail_count",
            "pos_std_e", "pos_std_n", "pos_std_alt",
        ])
        f.flush()
    return f, writer


def _handle_telemetry_frame(raw_frame, trajectory_queue, status_queue, csv_state, origin, telemetry_stats, cfg):
    """telemetry_stats: {"rate": RateStats, "seq_loss": SeqLossTracker, "pos": Vec3RollingStats, "last_csv_flush": float}."""
    telemetry = parse_lora_packet(raw_frame)
    now = time.time()

    telemetry_stats["rate"].record(now)
    telemetry_stats["seq_loss"].record(telemetry["seq"])

    pos_e = pos_n = None
    if telemetry["position_valid"]:
        origin.set_if_needed(telemetry["latitude"], telemetry["longitude"])
        pos_e, pos_n = origin.to_en(telemetry["latitude"], telemetry["longitude"])

    altitude = telemetry["altitude"] if telemetry["altitude_valid"] else None
    if (
        altitude is not None
        and abs(altitude) > cfg.LORA_ALTITUDE_WARNING_ABS_M
        and now - telemetry_stats.get("last_altitude_warning", 0.0) >= 5.0
    ):
        print(f"[lora] 비현실적 고도 수신(클리핑하지 않음): {altitude:.2f} m")
        telemetry_stats["last_altitude_warning"] = now

    if pos_e is not None and altitude is not None:
        telemetry_stats["pos"].record((pos_e, pos_n, altitude))

    telemetry_stats["last_telemetry_sequence"] = telemetry["sequence"]
    telemetry_stats["last_ack_sequence"] = telemetry["ack_sequence"]
    telemetry_stats["last_ack_result"] = telemetry["ack_result"]

    # sik.py와 동일하게 이 파이프라인은 "지금 기록할 만한 상태인지" 스스로
    # 판단하지 않는다 - 그 판단은 fusion.py 한 곳에서만 하고, 여기는 받은
    # 걸 무조건 큐/CSV로 흘려보내기만 한다. 발사 전 지상 노이즈는
    # flight_log_dir이 비행 시작 시점에 디렉터리를 통째로 바꿔주는 것으로
    # 이미 분리된다.
    trajectory_queue.put({
        "pos_e": pos_e,
        "pos_n": pos_n,
        "position_valid": pos_e is not None,
        "alt": altitude,
        "altitude_valid": altitude is not None,
        "timestamp": now,
    })
    csv_state.csv_writer.writerow([
        now, telemetry["sequence"], telemetry["flight_state"], telemetry["status_flags"],
        telemetry["fault_flags"], telemetry["time_ms"], telemetry["latitude"],
        telemetry["longitude"], pos_n, pos_e, altitude,
        telemetry["velocity_n"], telemetry["velocity_e"], telemetry["velocity_d"],
        telemetry["roll"], telemetry["pitch"], telemetry["yaw"],
        telemetry["ack_sequence"], telemetry["ack_result"],
        telemetry["ack_result_name"], ";".join(telemetry["faults"]),
    ])
    # sik.py와 동일한 이유(CSV_FLUSH_INTERVAL_SEC 주석 참고)로 매 프레임
    # 대신 일정 간격으로만 flush - 반이중 라디오라 명령 송신 중에도 이
    # 함수가 불리므로(ACK 대기 루프) 여기서 막히면 커맨드 재시도
    # 타이밍까지 밀린다.
    if now - telemetry_stats["last_csv_flush"] >= cfg.CSV_FLUSH_INTERVAL_SEC:
        csv_state.csv_file.flush()
        telemetry_stats["last_csv_flush"] = now

    status_queue.put({
        "source": "lora", "last_seen": now, "rocket_state": telemetry["flight_state"],
        "status_flags": telemetry["status_flags"],
        "fault_flags": telemetry["fault_flags"], "faults": telemetry["faults"],
        "ack_sequence": telemetry["ack_sequence"], "ack_result": telemetry["ack_result"],
    })
    return telemetry


def _send_command_and_wait_ack(
    ser, parser, command_name, seq, cfg,
    trajectory_queue, status_queue, csv_state, origin, telemetry_stats,
):
    """
    커맨드 프레임을 전송하고 최대 cfg.LORA_ACK_TIMEOUT_SEC 동안 telemetry의
    ack_sequence가 일치하는 최종 결과를 기다린다. 같은 논리적 커맨드는
    재시도 내내 동일한 seq를 재사용한다
    - comet_main.lua의 process_command_packet()이 (seq, command) 쌍으로
    중복 실행을 막고 캐시된 ACK를 재전송하기 때문에, 재시도마다 새
    seq를 쓰면 FC가 이미 처리한 커맨드를 새 커맨드로 인식해 중복 실행할
    수 있다.

    대기 중 도착하는 텔레메트리 프레임도 놓치지 않고 정상 처리한다
    (CSV/링크 last_seen 포함). 송신 직전 OS 수신 버퍼를 먼저 비우고 그때의
    telemetry sequence를 baseline으로 잡아, 버퍼에 남아 있던 예전 ACK를
    이번 명령 결과로 오인하지 않는다.
    """
    frame = _build_command_frame(command_name, seq)

    for attempt in range(1, cfg.LORA_COMMAND_RETRY_COUNT + 1):
        # 송신 전에 이미 도착해 있던 telemetry는 정상 처리하되 ACK 후보에서는
        # 제외한다. 반이중 링크의 지연 프레임이 다음 명령에 섞이는 것을 방지한다.
        pending = ser.in_waiting
        if pending:
            for raw_frame in parser.feed(ser.read(pending)):
                _handle_telemetry_frame(
                    raw_frame, trajectory_queue, status_queue, csv_state,
                    origin, telemetry_stats, cfg,
                )

        baseline_sequence = telemetry_stats.get("last_telemetry_sequence")
        ser.write(frame)
        deadline = time.time() + cfg.LORA_ACK_TIMEOUT_SEC

        while time.time() < deadline:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue

            for raw_frame in parser.feed(data):
                telemetry = _handle_telemetry_frame(
                    raw_frame, trajectory_queue, status_queue, csv_state,
                    origin, telemetry_stats, cfg,
                )
                delta = None if baseline_sequence is None else (
                    telemetry["sequence"] - baseline_sequence
                ) & 0xFF
                if baseline_sequence is None:
                    # GCS가 FC보다 늦게 재연결되면 첫 frame의 ACK는 이전 GCS
                    # 세션 명령일 수 있다. 첫 frame은 baseline으로만 사용하고,
                    # 반복 송신되는 다음 telemetry부터 이번 명령과 매칭한다.
                    baseline_sequence = telemetry["sequence"]
                    continue
                is_new_telemetry = 0 < delta < 128
                if not is_new_telemetry:
                    continue
                if (
                    telemetry["ack_sequence"] == seq
                    and telemetry["ack_result"] != cfg.LORA_ACK_RESULT_NONE
                ):
                    result = telemetry["ack_result"]
                    return result == 0, telemetry["ack_result_name"], attempt

    return False, None, cfg.LORA_COMMAND_RETRY_COUNT


def enqueue_command(command_queue, command_name: str) -> None:
    """
    GUI 스레드에서 호출한다. 실제 시리얼 송신/ACK 대기는 run_lora_pipeline
    스레드가 command_queue를 드레인하면서 전담한다.
    """
    if command_name not in config.LORA_COMMAND:
        raise ValueError(f"최신 Lua에서 지원하지 않는 LoRa 명령: {command_name}")
    command_queue.put({"command_name": command_name})


def send_state_reset_packet(command_queue) -> None:
    """최신 Lua에는 RESET command code가 없으므로 잘못된 frame 송신을 막는다."""
    raise ValueError("최신 Pixhawk Lua 프로토콜은 READY 상태 리셋 명령을 지원하지 않습니다")


def _open_connection(cfg):
    return serial.Serial(cfg.LORA_SERIAL_PORT, cfg.LORA_BAUDRATE, timeout=0.2)


def run_lora_pipeline(
    trajectory_queue: "queue.Queue", status_queue: "queue.Queue",
    cfg=config, command_queue=None, shared_origin=None, flight_log_dir=None,
):
    """
    별도 스레드에서 실행되는 LoRa 파이프라인 진입 함수.

    trajectory_queue에는 {pos_e, pos_n, alt, timestamp}를 push한다
    (main.py의 _poll_lora가 기대하는 스키마 그대로).
    command_queue가 주어지면 {"command_name": ...} 요청을 드레인해서
    커맨드 송신 + ACK 대기/재시도까지 이 스레드가 전담한다.

    shared_origin: shared_state.SharedOrigin 인스턴스 - SiK 파이프라인과 같은
    lat/lon ENU 원점을 공유하기 위해 main.py가 하나 만들어 양쪽에 주입한다.
    안 넘기면(단독 실행/테스트) 이 파이프라인만의 원점으로 새로 만든다.

    flight_log_dir: shared_state.FlightLogDir 인스턴스 - sik.py와 동일하게
    "발사 인가" 시점에 새 디렉터리로 CSV를 전환한다.

    연결이 끊기면(라디오 전원, 케이블 등) cfg.LORA_RECONNECT_DELAY_SEC
    간격으로 계속 재연결을 시도한다.
    """
    if shared_origin is None:
        shared_origin = shared_state.SharedOrigin()

    csv_state = shared_state.RotatingCsvState(cfg, _ensure_csv_writer, _ensure_stats_csv_writer, flight_log_dir, log_prefix="lora")
    try:
        while True:
            try:
                ser = _open_connection(cfg)
                _receive_loop(ser, trajectory_queue, status_queue, command_queue, csv_state, cfg, shared_origin)
            except (serial.SerialException, OSError) as exc:
                print(f"[lora] 연결 끊김/오류 - {cfg.LORA_RECONNECT_DELAY_SEC}s 후 재연결 시도: {exc}")
                time.sleep(cfg.LORA_RECONNECT_DELAY_SEC)
    finally:
        csv_state.close()


def _receive_loop(ser, trajectory_queue, status_queue, command_queue, csv_state, cfg, shared_origin):
    parser = _FrameParser(cfg)
    origin = shared_origin
    next_cmd_seq = 0

    # 패킷 품질/위치 안정성 정량 통계 (stats.py)
    telemetry_stats = {
        "rate": stats.RateStats(cfg.STATS_RATE_WINDOW),
        "seq_loss": stats.SeqLossTracker(),
        "pos": stats.Vec3RollingStats(cfg.STATS_ROLLING_WINDOW),
        "last_csv_flush": 0.0,
        "last_telemetry_sequence": None,
        "last_ack_sequence": None,
        "last_ack_result": cfg.LORA_ACK_RESULT_NONE,
        "last_altitude_warning": 0.0,
    }
    last_stats_report = 0.0

    while True:
        now = time.time()

        if csv_state.rotate_if_needed():
            telemetry_stats["last_csv_flush"] = 0.0

        if now - last_stats_report >= cfg.STATS_REPORT_INTERVAL_SEC:
            last_stats_report = now
            rate_hz = telemetry_stats["rate"].rate_hz()
            jitter_ms = telemetry_stats["rate"].jitter_ms()
            seq_loss_pct = telemetry_stats["seq_loss"].loss_pct()
            crc_fail_count = parser.crc_fail_count
            pos_std = telemetry_stats["pos"].stddev()
            print(
                f"[lora-stats] rate={rate_hz:.1f}Hz jitter={jitter_ms:.1f}ms seq_loss={seq_loss_pct:.1f}% "
                f"crc_fail={crc_fail_count} pos_std(e,n,alt)=({pos_std[0]:.3f},{pos_std[1]:.3f},{pos_std[2]:.3f})"
            )
            csv_state.stats_csv_writer.writerow([now, rate_hz, jitter_ms, seq_loss_pct, crc_fail_count, *pos_std])
            csv_state.stats_csv_file.flush()

        if command_queue is not None:
            try:
                request = command_queue.get_nowait()
            except queue.Empty:
                request = None

            if request is not None:
                command_name = request["command_name"]
                seq = next_cmd_seq
                # telemetry ACK에는 command code가 없으므로, 마지막으로 관측한
                # 유효 ACK sequence와 같은 번호는 wraparound 때 한 번 건너뛴다.
                if (
                    telemetry_stats["last_ack_result"] != cfg.LORA_ACK_RESULT_NONE
                    and seq == telemetry_stats["last_ack_sequence"]
                ):
                    seq = (seq + 1) % 256
                next_cmd_seq = (seq + 1) % 256

                success, result_name, attempts = _send_command_and_wait_ack(
                    ser, parser, command_name, seq, cfg,
                    trajectory_queue, status_queue, csv_state, origin, telemetry_stats,
                )
                status_queue.put({
                    "source": "lora",
                    "command_result": {
                        "command_name": command_name,
                        "sequence": seq,
                        "success": success,
                        "result": result_name,
                        "attempts": attempts,
                    },
                })

        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue

        for raw_frame in parser.feed(data):
            _handle_telemetry_frame(raw_frame, trajectory_queue, status_queue, csv_state, origin, telemetry_stats, cfg)
