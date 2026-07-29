# -*- coding: utf-8 -*-
"""
LoRa 실통신 파이프라인 (DX-LR03-900T30D, 투명전송 모드).

FC측 comet_main.lua와 커스텀 바이너리 프로토콜로 통신한다. LoRa 모듈
자체는 프레이밍을 안 해주므로(투명전송), 헤더 바이트 + CRC-8로 우리가
직접 연속 바이트스트림에서 프레임을 잘라낸다 (_FrameParser).

3가지 프레임 (config.py의 LORA_* 상수와 comet_config.lua의 CONFIG.LORA에
정확히 대응):
  - TELEMETRY (0xA5, 17B, FC->GCS): 위치/비행상태/fault_flags만 (속도/자세는
    SiK/mavlink 전용 - fusion.py 참고). 위치는 절대 위경도가 아니라 HOME
    기준 north/east 오프셋(cm 정밀도, int16)이라 origin 변환 없이 그대로
    로컬 미터 좌표로 쓴다. comet_lora.lua의 send_telemetry()와 바이트
    단위로 동일한 레이아웃 (정확한 필드/스케일은 config.py의
    LORA_TELEMETRY_LENGTH 주석 참고).
  - COMMAND   (0x5A, 4B,  GCS->FC): 서보 lock/release, READY 리셋 등.
  - ACK       (0xA6, 4B,  FC->GCS): COMMAND에 대한 응답.

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

# comet_lora.lua send_telemetry()의 바이트 레이아웃과 정확히 일치.
# 자세한 필드 설명은 config.py의 LORA_TELEMETRY_LENGTH 주석 참고.
_TELEMETRY_STRUCT = struct.Struct("<4BI2hiB")


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
            cfg.LORA_ACK_HEADER: cfg.LORA_ACK_LENGTH,
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
    raw_bytes: _FrameParser가 CRC 검증까지 끝낸 완전한 프레임
    (텔레메트리 21B 또는 ACK 4B). 헤더 값으로 종류를 구분해 언패킹한다.
    """
    header = raw_bytes[0]
    if header == config.LORA_TELEMETRY_HEADER:
        return _parse_telemetry(raw_bytes)
    if header == config.LORA_ACK_HEADER:
        return _parse_ack(raw_bytes)
    raise ValueError(f"알 수 없는 LoRa 프레임 헤더: {header:#x}")


def _parse_telemetry(raw_bytes) -> dict:
    (
        _header, seq, flight_state, fault_flags, _time_ms,
        north_raw, east_raw, alt_raw,
        _crc,
    ) = _TELEMETRY_STRUCT.unpack(raw_bytes)

    state_names = config.LORA_FLIGHT_STATE_NAMES
    state_name = state_names[flight_state] if 0 <= flight_state < len(state_names) else "UNKNOWN"

    # fault_flags 비트 하나하나를 사람이 읽을 이름으로 풀어낸다 - "에러가
    # 났다"만이 아니라 "왜"까지 GCS 화면/CSV에 그대로 남기기 위함
    # (config.py의 LORA_FAULT_FLAG_NAMES 참고).
    faults = [name for bit, name in config.LORA_FAULT_FLAG_NAMES.items() if fault_flags & bit]

    return {
        "frame_type": "telemetry",
        "seq": seq,
        "flight_state": state_name,
        "fault_flags": fault_flags,
        "faults": faults,
        # comet_sensor.lua가 이미 HOME 기준으로 계산해서 보내므로(절대
        # 위경도 아님) origin 변환 없이 그대로 로컬 미터 좌표로 쓴다.
        "pos_n": north_raw / 100.0,
        "pos_e": east_raw / 100.0,
        "alt": alt_raw / 100.0,
    }


def _parse_ack(raw_bytes) -> dict:
    _header, seq, result, _crc = raw_bytes
    return {
        "frame_type": "ack",
        "seq": seq,
        "result": result,
        "result_name": config.LORA_ACK_RESULT_NAMES.get(result, "UNKNOWN"),
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
    is_new = not os.path.exists(filepath)

    f = open(filepath, mode="a", newline="", encoding="utf-8")
    writer = csv.writer(f)
    if is_new:
        writer.writerow([
            "timestamp", "seq", "flight_state",
            "pos_n", "pos_e", "alt",
            "fault_flags", "faults",
        ])
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

    # comet_sensor.lua가 이미 HOME 기준 로컬 좌표로 계산해서 보내므로
    # (절대 위경도가 아님) origin(SharedOrigin) 변환이 필요 없다 - 파라미터는
    # sik.py 파이프라인 호출 규약과 맞추기 위해 그대로 받아만 두고 안 쓴다.
    pos_n, pos_e = telemetry["pos_n"], telemetry["pos_e"]
    telemetry_stats["pos"].record((pos_e, pos_n, telemetry["alt"]))

    # sik.py와 동일하게 이 파이프라인은 "지금 기록할 만한 상태인지" 스스로
    # 판단하지 않는다 - 그 판단은 fusion.py 한 곳에서만 하고, 여기는 받은
    # 걸 무조건 큐/CSV로 흘려보내기만 한다. 발사 전 지상 노이즈는
    # flight_log_dir이 비행 시작 시점에 디렉터리를 통째로 바꿔주는 것으로
    # 이미 분리된다.
    trajectory_queue.put({
        "pos_e": pos_e,
        "pos_n": pos_n,
        "alt": telemetry["alt"],
        "timestamp": now,
    })
    csv_state.csv_writer.writerow([
        now, telemetry["seq"], telemetry["flight_state"],
        telemetry["pos_n"], telemetry["pos_e"], telemetry["alt"],
        telemetry["fault_flags"], ";".join(telemetry["faults"]),
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
        "fault_flags": telemetry["fault_flags"], "faults": telemetry["faults"],
    })


def _send_command_and_wait_ack(
    ser, parser, command_name, seq, cfg,
    trajectory_queue, status_queue, csv_state, origin, telemetry_stats,
):
    """
    커맨드 프레임을 전송하고 최대 cfg.LORA_ACK_TIMEOUT_SEC 동안 seq가
    일치하는 ACK를 기다린다. 실패하면 cfg.LORA_COMMAND_RETRY_COUNT회까지
    재시도한다. 같은 논리적 커맨드는 재시도 내내 동일한 seq를 재사용한다
    - comet_main.lua의 process_command_packet()이 (seq, command) 쌍으로
    중복 실행을 막고 캐시된 ACK를 재전송하기 때문에, 재시도마다 새
    seq를 쓰면 FC가 이미 처리한 커맨드를 새 커맨드로 인식해 중복 실행할
    수 있다.

    comet_lora.lua는 명령 하나당 같은 seq로 ACK를 두 번 보낸다: CRC 검증
    직후 즉시 RECEIVED(중간 응답), 그리고 상태머신이 실제로 처리한 뒤
    최종 결과(OK/NOT_ALLOWED/EXECUTION_FAILED/INVALID_COMMAND). RECEIVED는
    최종 응답이 아니므로 여기서 리턴하지 않고 같은 데드라인 안에서 계속
    기다린다 - RECEIVED만 보고 성공 처리하면 FC가 두 번째(진짜) ACK를 마저
    보내는 도중에 다음 커맨드를 반이중 라디오로 전송하게 돼 충돌이 나고,
    뒤이은 커맨드가 FC에 아예 안 닿는 문제가 있었다.

    대기 중 도착하는 텔레메트리 프레임도 놓치지 않고 정상 처리한다
    (반이중 하드웨어라 명령 송신 중에는 새 텔레메트리가 안 오지만, 응답
    ACK 직후 텔레메트리가 같은 버퍼에 이어 도착할 수 있다).
    """
    frame = _build_command_frame(command_name, seq)

    for attempt in range(1, cfg.LORA_COMMAND_RETRY_COUNT + 1):
        ser.write(frame)
        deadline = time.time() + cfg.LORA_ACK_TIMEOUT_SEC

        while time.time() < deadline:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue

            for raw_frame in parser.feed(data):
                if raw_frame[0] == cfg.LORA_TELEMETRY_HEADER:
                    _handle_telemetry_frame(raw_frame, trajectory_queue, status_queue, csv_state, origin, telemetry_stats, cfg)
                    continue

                parsed = parse_lora_packet(raw_frame)
                if parsed["seq"] == seq:
                    if parsed["result"] == cfg.LORA_ACK_RESULT_RECEIVED:
                        continue  # 중간 응답 - 최종 결과 ACK를 계속 기다린다
                    return True, parsed["result_name"], attempt

    return False, None, cfg.LORA_COMMAND_RETRY_COUNT


def enqueue_command(command_queue, command_name: str) -> None:
    """
    GUI 스레드에서 호출한다. 실제 시리얼 송신/ACK 대기는 run_lora_pipeline
    스레드가 command_queue를 드레인하면서 전담한다.
    """
    command_queue.put({"command_name": command_name})


def send_state_reset_packet(command_queue) -> None:
    """로켓 상태를 READY로 리셋하는 RESET 커맨드를 큐에 넣는다."""
    enqueue_command(command_queue, "RESET")


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
                next_cmd_seq = (next_cmd_seq + 1) % 256

                success, result_name, attempts = _send_command_and_wait_ack(
                    ser, parser, command_name, seq, cfg,
                    trajectory_queue, status_queue, csv_state, origin, telemetry_stats,
                )
                status_queue.put({
                    "source": "lora",
                    "command_result": {
                        "command_name": command_name,
                        "success": success,
                        "result": result_name,
                        "attempts": attempts,
                    },
                })

        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue

        for raw_frame in parser.feed(data):
            if raw_frame[0] != cfg.LORA_TELEMETRY_HEADER:
                continue  # 커맨드 대기 중이 아닐 때 도착한 ACK는 지연 응답 - 무시
            _handle_telemetry_frame(raw_frame, trajectory_queue, status_queue, csv_state, origin, telemetry_stats, cfg)
