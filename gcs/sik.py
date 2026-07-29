# -*- coding: utf-8 -*-
"""
SiK/pymavlink 수신 파이프라인.

역할: pymavlink 커넥션에서 필요한 필드만 선별 수신(RAW_IMU, GPS_RAW_INT,
RC_CHANNELS, STATUSTEXT, LOCAL_POSITION_NED, SCALED_IMU, SCALED_PRESSURE)
-> kalman.py 필터 갱신 -> 결과를 큐에 push -> CSV 기록.

칼만필터 관측은 수평/수직을 분리해서 각자 도착하는 대로 갱신한다:
GPS_RAW_INT -> kf.update_horizontal(pos_e, pos_n) (~5Hz),
SCALED_PRESSURE -> kf.update_vertical(alt) (~10Hz, GPS보다 빠르고 정확).
같이 기다렸다 한 번에 update하면 기압계의 빠른 갱신률을 버리게 되므로
일부러 나눴다.

LOCAL_POSITION_NED/SCALED_IMU는 온보드 EKF3가 이미 계산한 위치/속도/가속도를
그대로 읽어 GUI에 표시하기 위한 것으로, 우리 자체 6D 칼만필터(kalman.py)와는
별개의 값이다 (필터링하지 않고 FC가 보고하는 값 그대로 전달).

이 모듈은 큐를 누가 구독하는지 알지 못한다 (data/시각화 계층 분리).
나중에 main.py의 구독 루프 자리에 ROS publisher를 추가해도 이 파일은
손댈 필요가 없다.
"""

import csv
import math
import os
import queue
import time
from datetime import datetime

from pymavlink import mavutil

import config
import shared_state
import stats
from kalman import KalmanFilter6D

# 다른 모듈(main.py)이 FC reboot 등에 사용할 수 있도록 커넥션을 노출한다.
_connection = None

_ACCEL_MG_TO_MPS2 = 9.80665 / 1000.0
_GRAVITY_MPS2 = 9.80665


def _body_accel_to_enu(accel_body, roll, pitch, yaw):
    """
    RAW_IMU의 body-frame(X=전방, Y=우, Z=하) 가속도를 ENU 월드 프레임으로
    회전시키고 중력을 제거한다.

    가속도계는 "specific force"(진짜 가속도 - 중력)를 측정한다 - 정지 상태
    에서도 지면 반작용력 때문에 body Z(아래) 축으로 약 -1g가 찍힌다
    (실측: accel_z ≈ -9.7). 이걸 회전/중력보정 없이 그대로 ENU 가속도로
    취급해 kf.predict()에 넣으면, 정지 상태에서도 "위로 1g 가속 중"이라고
    착각해 속도/위치가 시간이 지날수록 무한정 드리프트한다 (실제로 SiK만
    연결하고 GPS 3D fix가 없는 상태에서 위치가 수백 미터씩 튀는 현상으로
    관측됨 - update_horizontal이 안 불려서 이 드리프트를 잡아줄 방법이 없었음).
    """
    ax, ay, az = accel_body
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # body(전방,우,하) -> NED, 표준 R = Rz(yaw)*Ry(pitch)*Rx(roll) (viz3d.body_to_world와 동일 회전식)
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    north = r00 * ax + r01 * ay + r02 * az
    east = r10 * ax + r11 * ay + r12 * az
    down = r20 * ax + r21 * ay + r22 * az

    # 진짜 가속도(NED) = specific force + 중력(NED에서 down 방향 +g).
    # 정지 시 down이 -g였다면 true_down = -g + g = 0 (드리프트 원인 제거).
    true_down = down + _GRAVITY_MPS2

    return east, north, -true_down  # ENU: east, north, up(=-down)


def get_connection():
    """현재 활성화된 SiK mavlink 커넥션을 반환한다 (없으면 None)."""
    return _connection


def _open_connection(cfg):
    conn = mavutil.mavlink_connection(
        cfg.SIK_CONNECTION_STRING,
        baud=cfg.SIK_BAUDRATE,
    )
    if not conn.wait_heartbeat(timeout=cfg.SIK_HEARTBEAT_TIMEOUT_SEC):
        raise TimeoutError(f"heartbeat 수신 실패 (timeout={cfg.SIK_HEARTBEAT_TIMEOUT_SEC}s)")
    return conn


def _silence_default_streams(conn):
    """
    FC가 SR params 기반으로 기본으로 흘려보내는 모든 스트림을 끈다.
    이걸 먼저 하지 않으면 우리가 요청하지 않은 메시지들도 계속 SiK
    대역폭을 잡아먹어서, 정작 필요한 메시지의 실질 갱신율이 낮아진다.
    """
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        0,  # rate - stop이므로 의미 없음
        0,  # start_stop = 0 (stop)
    )


def _request_message_rates(conn, cfg):
    """
    기본 스트림을 끄고, config.SIK_REQUESTED_MESSAGES에 나열된 메시지만
    선택적으로 요청한다. 받고 싶은 메시지/주기를 바꾸려면 config.py의
    SIK_REQUESTED_MESSAGES만 수정하면 되고 이 함수는 손댈 필요 없다.

    [되돌림] 한때 COMMAND_ACK를 기다렸다가 못 받으면 재전송하는 로직이
    있었는데, 실제 비행 로그로 확인해보니 그 방식으로 바꾼 뒤부터 RAW_IMU가
    연결 초반 잠깐만 들어오고 곧 완전히 끊기는 회귀가 있었다(그 전에는
    잘 됐음). ACK 대기 중 conn.recv_match(type="COMMAND_ACK", ...)가 그
    사이 도착하는 RAW_IMU 등 실제 텔레메트리를 계속 버리는 구조였고, 동일
    SET_MESSAGE_INTERVAL을 FC에 짧은 간격으로 반복 전송하는 것도 FC의
    스트림 스케줄러에 안 좋은 영향을 줬을 가능성이 있어 원래의 단순
    fire-and-forget 전송으로 되돌린다.
    """
    _silence_default_streams(conn)

    for msg_name, rate_hz in cfg.SIK_REQUESTED_MESSAGES:
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_name}")
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1e6 / rate_hz),
            0, 0, 0, 0, 0,
        )


def _ensure_csv_writer(cfg, output_dir=None):
    output_dir = output_dir or cfg.CSV_OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)
    filename = datetime.now().strftime(cfg.SIK_CSV_FILENAME_PATTERN)
    filepath = os.path.join(output_dir, filename)
    is_new = not os.path.exists(filepath)

    f = open(filepath, mode="a", newline="", encoding="utf-8")
    writer = csv.writer(f)
    if is_new:
        writer.writerow([
            "timestamp",
            "raw_lat", "raw_lon", "raw_alt_m",
            "gps_fix_type", "gps_satellites", "gps_hdop",
            "raw_accel_e", "raw_accel_n", "raw_accel_z",
            "attitude_roll", "attitude_pitch", "attitude_yaw",
            "baro_press_hpa", "baro_alt_m",
            "battery_voltage_v", "battery_current_a", "cpu_load_pct",
            "kf_pos_e", "kf_pos_n", "kf_alt",
            "kf_vel_e", "kf_vel_n", "kf_vel_z",
        ])
    return f, writer


def _ensure_stats_csv_writer(cfg, output_dir=None):
    output_dir = output_dir or cfg.CSV_OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)
    filename = datetime.now().strftime(cfg.SIK_STATS_CSV_FILENAME_PATTERN)
    filepath = os.path.join(output_dir, filename)
    is_new = not os.path.exists(filepath)

    f = open(filepath, mode="a", newline="", encoding="utf-8")
    writer = csv.writer(f)
    if is_new:
        writer.writerow([
            "timestamp", "rate_hz", "jitter_ms", "link_loss_pct",
            "pos_std_e", "pos_std_n", "pos_std_alt",
            "vel_std_e", "vel_std_n", "vel_std_z",
        ])
        f.flush()
    return f, writer


class _OriginTracker:
    """
    lat/lon 원점은 shared_origin(shared_state.SharedOrigin)에 위임한다 - SiK와
    LoRa가 같은 원점을 공유해야 3D 뷰에서 두 궤적이 같은 좌표계에 그려진다.
    alt0(상대고도 기준)는 SiK 자체 GPS의 첫 유효값을 그대로 쓴다 (LoRa는
    이미 HOME 기준 상대고도를 보내므로 이 값을 공유할 필요가 없다).
    """

    def __init__(self, shared_origin):
        self._shared = shared_origin
        self.alt0 = None

    def set_if_needed(self, lat, lon, alt):
        self._shared.set_if_needed(lat, lon)
        if self.alt0 is None:
            self.alt0 = alt

    def to_enu(self, lat, lon, alt):
        pos_e, pos_n = self._shared.to_en(lat, lon)
        rel_alt = alt - self.alt0
        return pos_e, pos_n, rel_alt


class _BaroOriginTracker:
    """첫 기압값을 기준(0m)으로 삼아 상대 고도로 환산한다."""

    def __init__(self):
        self.press0_hpa = None

    def set_if_needed(self, press_hpa):
        if self.press0_hpa is None:
            self.press0_hpa = press_hpa

    def to_relative_altitude(self, press_hpa):
        # 국제표준대기 근사식. press0을 기준(0m)으로 잡으므로 해수면
        # 기압(QNH) 보정 없이도 "발사대 기준 상대 고도"로 바로 쓸 수 있다.
        return 44330.0 * (1.0 - (press_hpa / self.press0_hpa) ** (1.0 / 5.255))


def run_sik_pipeline(trajectory_queue: "queue.Queue", status_queue: "queue.Queue", cfg=config, shared_origin=None, flight_log_dir=None):
    """
    별도 스레드에서 실행되는 SiK 파이프라인 진입 함수.

    trajectory_queue에는 {pos_e, pos_n, alt, timestamp} 형태로 push한다.
    status_queue에는 링크 생존/RC 채널/로켓 상태 등 GUI 표시용 부가 정보를
    push한다 (파서 로직은 여기서만 수행하고 GUI는 값만 표시).

    shared_origin: shared_state.SharedOrigin 인스턴스 - LoRa 파이프라인과 같은
    lat/lon ENU 원점을 공유하기 위해 main.py가 하나 만들어 양쪽에 주입한다.
    안 넘기면(단독 실행/테스트) 이 파이프라인만의 원점으로 새로 만든다.

    flight_log_dir: shared_state.FlightLogDir 인스턴스 - main.py가 "발사 인가"
    시점에 새 디렉터리를 요청하면(request_new_dir) 연결을 안 끊고도 다음
    루프에서 그 디렉터리로 CSV 파일을 새로 연다. 안 넘기면(단독 실행/테스트)
    cfg.CSV_OUTPUT_DIR에 계속 쓴다.

    연결이 끊기면(FC 리부트, 케이블 순단, SiK 라디오 순간 오류 등) 스레드가
    죽지 않고 cfg.SIK_RECONNECT_DELAY_SEC 간격으로 계속 재연결을 시도한다.
    재연결할 때마다 메시지 요청도 다시 보낸다 - FC가 리부트되면 이전에
    보낸 SET_MESSAGE_INTERVAL 설정은 사라지기 때문.
    """
    global _connection

    if shared_origin is None:
        shared_origin = shared_state.SharedOrigin()

    csv_state = shared_state.RotatingCsvState(cfg, _ensure_csv_writer, _ensure_stats_csv_writer, flight_log_dir, log_prefix="sik")

    # 발사 직전 GCS에서 FC 리부트 명령을 주는 절차가 있는데, 리부트는 전원만
    # 재시작될 뿐 로켓의 물리적 자세는 그대로다. 그런데 재연결 직후 FC가 새
    # ATTITUDE를 다시 보내기 전까지 몇 개의 RAW_IMU 샘플이 먼저 도착할 수
    # 있고, 그 순간 자세를 (0,0,0)(수평)으로 잘못 가정하면 중력보정이
    # 틀어져 위조 가속도가 칼만필터에 들어간다(velocity는 GPS/기압계로
    # 직접 관측되지 않아 이 오차가 잘 안 씻겨나감). 그래서 리부트/재연결
    # 전반에 걸쳐 "마지막으로 실제 수신한 자세"를 여기(run_sik_pipeline)에
    # 보관해뒀다가 다음 연결의 초기 자세로 이어서 쓴다 - 칼만필터 자체(위치/
    # 속도)는 기존 설계 그대로 매 연결마다 깨끗하게 새로 시작한다.
    last_attitude_holder = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    try:
        while True:
            try:
                conn = _open_connection(cfg)
                _connection = conn
                _request_message_rates(conn, cfg)
                _receive_loop(conn, trajectory_queue, status_queue, csv_state, cfg, shared_origin, last_attitude_holder)
            except Exception as exc:  # noqa: BLE001 - 어떤 이유로든 끊기면 재연결
                _connection = None
                print(f"[sik] 연결 끊김/오류 - {cfg.SIK_RECONNECT_DELAY_SEC}s 후 재연결 시도: {exc}")
                time.sleep(cfg.SIK_RECONNECT_DELAY_SEC)
    finally:
        csv_state.close()


def _receive_loop(conn, trajectory_queue: "queue.Queue", status_queue: "queue.Queue", csv_state, cfg, shared_origin, last_attitude_holder):
    """
    연결 하나에 대한 실제 수신 루프. 연결이 죽으면(예외 발생) 그대로
    위(run_sik_pipeline)로 전파되어 재연결 로직이 처리한다.
    칼만필터/alt 기준점은 매 연결마다 새로 시작한다 (FC 리부트 후 이어붙이는
    것보다 깨끗하게 다시 잡는 편이 안전). lat/lon 공유 원점(shared_origin)은
    main.py가 소유한 하나의 인스턴스라 재연결과 무관하게 계속 유지된다.

    last_attitude_holder: run_sik_pipeline이 재연결 전반에 걸쳐 들고 있는
    {"roll","pitch","yaw"} dict - 새 연결의 초기 자세를 (0,0,0) 대신 이
    값에서 이어받고, ATTITUDE 수신 시마다 다시 갱신해준다.

    csv_state: shared_state.RotatingCsvState - "발사 인가" 시점에 새 비행
    로그 디렉터리가 요청되면 연결을 안 끊고도 파일 핸들을 통째로 교체한다.
    """
    kf = KalmanFilter6D()
    origin = _OriginTracker(shared_origin)
    baro_origin = _BaroOriginTracker()

    last_predict_time = None
    last_raw_gps = (None, None, None)
    # roll, pitch, yaw (rad) - RAW_IMU 회전/중력보정에 사용. (0,0,0)이 아니라
    # 직전 연결에서 마지막으로 실제 수신한 자세로 시작한다(위 설명 참고).
    last_attitude = (last_attitude_holder["roll"], last_attitude_holder["pitch"], last_attitude_holder["yaw"])
    last_csv_flush = 0.0

    # 아래 4개는 각자 도착 주기가 RAW_IMU(30Hz)보다 느려서, CSV에는 RAW_IMU
    # 행이 기록될 때마다 "마지막으로 받은 값"을 그대로 얹는다(raw_lat/lon과
    # 같은 패턴) - 사후분석용 로깅이 목적이라 굳이 별도 파일로 안 쪼갰다.
    last_gps_quality = (None, None, None)  # fix_type, satellites_visible, hdop
    last_baro = (None, None)  # press_hpa, baro_alt_m
    last_battery = (None, None, None)  # voltage_v, current_a, cpu_load_pct

    # 패킷 품질/위치·속도 안정성 정량 통계 (stats.py) - 연결 하나당 새로 시작
    link_seq_loss = stats.SeqLossTracker()
    raw_imu_rate = stats.RateStats(cfg.STATS_RATE_WINDOW)
    pos_stats = stats.Vec3RollingStats(cfg.STATS_ROLLING_WINDOW)
    vel_stats = stats.Vec3RollingStats(cfg.STATS_ROLLING_WINDOW)
    last_stats_report = 0.0

    # RAW_IMU 워치독 - 이 시간(cfg.SIK_RAW_IMU_WATCHDOG_SEC) 동안 RAW_IMU가
    # 한 통도 안 오면 SET_MESSAGE_INTERVAL을 RAW_IMU 하나만 다시(ACK 대기
    # 없이, 논블로킹으로) 보낸다. 예전엔 연결 시작 시점에 COMMAND_ACK를
    # 기다렸다 재전송하는 방식이었는데, 그 대기 중 recv_match(type=
    # "COMMAND_ACK", ...)가 실제 텔레메트리를 계속 버리는 부작용이 있었고
    # (RAW_IMU가 연결 초반 잠깐 들어오다 완전히 끊기는 회귀로 실측됨), FC
    # 스트림 스케줄러에 짧은 간격으로 같은 명령을 반복 보내는 것도 안전하지
    # 않아 보였다. 이 워치독은 그 대신 "본 수신 루프를 막지 않고, 스트림이
    # 실제로 멈췄을 때만 조용히 한 번 다시 요청"하는 방식이라 부작용이 없다.
    last_raw_imu_msg_time = time.time()
    raw_imu_msg_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_RAW_IMU")

    while True:
        msg = conn.recv_match(blocking=True, timeout=1.0)
        now = time.time()

        if csv_state.rotate_if_needed():
            last_csv_flush = 0.0

        if now - last_raw_imu_msg_time >= cfg.SIK_RAW_IMU_WATCHDOG_SEC:
            print(f"[sik] RAW_IMU {cfg.SIK_RAW_IMU_WATCHDOG_SEC:.0f}초간 미수신 - 재요청")
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                raw_imu_msg_id, int(1e6 / cfg.SIK_IMU_RATE_HZ),
                0, 0, 0, 0, 0,
            )
            last_raw_imu_msg_time = now  # 재요청 직후 다시 같은 경고가 연속으로 안 뜨게

        if msg is None:
            continue

        # mavlink 패킷 자체의 header seq(0~255)로, 메시지 타입과 무관하게
        # 링크 전체의 손실률을 추정한다.
        link_seq_loss.record(msg.get_seq())

        if now - last_stats_report >= cfg.STATS_REPORT_INTERVAL_SEC:
            last_stats_report = now
            rate_hz = raw_imu_rate.rate_hz()
            jitter_ms = raw_imu_rate.jitter_ms()
            loss_pct = link_seq_loss.loss_pct()
            pos_std = pos_stats.stddev()
            vel_std = vel_stats.stddev()
            print(
                f"[sik-stats] rate={rate_hz:.1f}Hz jitter={jitter_ms:.1f}ms loss={loss_pct:.1f}% "
                f"pos_std(e,n,alt)=({pos_std[0]:.3f},{pos_std[1]:.3f},{pos_std[2]:.3f}) "
                f"vel_std(e,n,z)=({vel_std[0]:.3f},{vel_std[1]:.3f},{vel_std[2]:.3f})"
            )
            csv_state.stats_csv_writer.writerow([now, rate_hz, jitter_ms, loss_pct, *pos_std, *vel_std])
            csv_state.stats_csv_file.flush()

        msg_type = msg.get_type()

        if msg_type == "RAW_IMU":
            # body-frame(전방/우/하) specific force - 표시/CSV용으로는 그대로
            # 쓰지만, 칼만필터에는 자세로 회전 + 중력 제거한 ENU 가속도를
            # 넣어야 한다 (_body_accel_to_enu 참고 - 안 그러면 정지 상태에서도
            # 위치가 무한정 드리프트한다).
            last_raw_imu_msg_time = now
            accel_e = msg.xacc * _ACCEL_MG_TO_MPS2
            accel_n = msg.yacc * _ACCEL_MG_TO_MPS2
            accel_z = msg.zacc * _ACCEL_MG_TO_MPS2
            predict_accel_enu = _body_accel_to_enu((accel_e, accel_n, accel_z), *last_attitude)

            dt = 0.0 if last_predict_time is None else (now - last_predict_time)
            last_predict_time = now
            if dt > 0:
                kf.predict(dt, predict_accel_enu)

            state = kf.get_state()

            raw_imu_rate.record(now)
            pos_stats.record((state["pos_e"], state["pos_n"], state["alt"]))
            vel_stats.record((state["vel_e"], state["vel_n"], state["vel_z"]))

            # 이 파이프라인은 "지금이 기록할 만한 상태인지" 스스로 판단하지
            # 않는다 - 로켓 상태 판단은 fusion.py(퓨전 레이어) 한 곳에서만
            # 하고, sik.py는 받은 걸 무조건 큐/CSV로 흘려보내기만 한다
            # (파이프라인/판단/표시 3단 분리). 발사 전 지상 노이즈가 실제
            # 비행 CSV와 안 섞이는 건 flight_log_dir이 비행 시작 시점에
            # 아예 다른 디렉터리로 전환해주는 것으로 이미 해결돼 있다.
            status_payload = {
                "source": "sik",
                "last_seen": now,
                "live_pos_e": state["pos_e"],
                "live_pos_n": state["pos_n"],
                "live_alt": state["alt"],
                "accel_e": accel_e,
                "accel_n": accel_n,
                "accel_z": accel_z,
                "live_vel_e": state["vel_e"],
                "live_vel_n": state["vel_n"],
                "live_vel_z": state["vel_z"],
            }

            trajectory_queue.put({
                "pos_e": state["pos_e"],
                "pos_n": state["pos_n"],
                "alt": state["alt"],
                "timestamp": now,
            })
            csv_state.csv_writer.writerow([
                now,
                last_raw_gps[0], last_raw_gps[1], last_raw_gps[2],
                last_gps_quality[0], last_gps_quality[1], last_gps_quality[2],
                accel_e, accel_n, accel_z,
                last_attitude[0], last_attitude[1], last_attitude[2],
                last_baro[0], last_baro[1],
                last_battery[0], last_battery[1], last_battery[2],
                state["pos_e"], state["pos_n"], state["alt"],
                state["vel_e"], state["vel_n"], state["vel_z"],
            ])
            # 매 샘플(최대 30Hz)마다 flush()하면 그 디스크/동기화 I/O가
            # 수신 루프를 막아 뒤이어 도착하는 시리얼 바이트가 유실될 수
            # 있어(CSV_FLUSH_INTERVAL_SEC 주석 참고) 일정 간격으로만 flush.
            if now - last_csv_flush >= cfg.CSV_FLUSH_INTERVAL_SEC:
                csv_state.csv_file.flush()
                last_csv_flush = now

            status_queue.put(status_payload)

        elif msg_type == "GPS_RAW_INT":
            # fix_type<3(3D fix 미만)이어도 품질 수치 자체는 사후분석에
            # 쓸모 있으니(예: "이 구간엔 fix가 없었다") 기록은 하고, 칼만필터
            # 관측 반영은 여기서 continue로 그대로 건너뛴다.
            satellites = msg.satellites_visible if msg.satellites_visible != 255 else None
            hdop = (msg.eph / 100.0) if msg.eph != 65535 else None
            last_gps_quality = (msg.fix_type, satellites, hdop)

            if msg.fix_type < 3:
                continue
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            last_raw_gps = (lat, lon, alt)

            origin.set_if_needed(lat, lon, alt)
            pos_e, pos_n, rel_alt = origin.to_enu(lat, lon, alt)
            kf.update_horizontal((pos_e, pos_n))

            status_queue.put({
                "source": "sik",
                "last_seen": now,
                "last_gps": {"lat": lat, "lon": lon, "alt": alt},
            })

        elif msg_type == "RC_CHANNELS":
            status_queue.put({
                "source": "sik",
                "last_seen": now,
                "rc_ch8": msg.chan8_raw,
                "rc_ch9": msg.chan9_raw,
            })

        elif msg_type == "SYS_STATUS":
            # 지금까지 SYS_STATUS는 요청만 해두고(SIK_SYS_STATUS_RATE_HZ)
            # 받아서 버리고 있었다 - 배터리 전압은 비행 안전과 직결돼서
            # 로그에는 반드시 남긴다.
            voltage_v = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
            current_a = msg.current_battery / 100.0 if msg.current_battery != -1 else None
            cpu_load_pct = msg.load / 10.0
            last_battery = (voltage_v, current_a, cpu_load_pct)

        elif msg_type == "STATUSTEXT":
            text_upper = msg.text.upper()
            matched_state = next(
                (
                    state_name
                    for state_name, keywords in cfg.ROCKET_STATE_KEYWORDS.items()
                    if any(kw in text_upper for kw in keywords)
                ),
                None,
            )
            # 원본 mavlink 메시지를 그대로 전달한다 - 실제 파싱(pre-arm
            # 사유 추출)은 mission_features.check_prearm_status()에서
            # 수행하며, sik.py는 여기서 파서 로직을 갖지 않는다.
            # statustext는 GUI 메시지 로그 콘솔에 그대로 표시하기 위한 것.
            payload = {
                "source": "sik",
                "last_seen": now,
                "prearm_msg": msg,
                "statustext": {"text": msg.text, "severity": msg.severity},
            }
            if matched_state is not None:
                payload["rocket_state"] = matched_state
            status_queue.put(payload)

        elif msg_type == "NAMED_VALUE_FLOAT":
            # comet_mavlink.lua가 보내는 COMET_ACK - 서보 잠금/사출 등
            # COMET_CMD(mission_features.send_mavlink_command)의 실행 결과.
            # FC는 추가 모듈 없이 ArduPilot 내장 gcs:send_named_float()로
            # 보내므로 NAMED_VALUE_INT가 아니라 NAMED_VALUE_FLOAT으로 온다
            # (COMET_CMD 쪽, 즉 GCS->FC 방향은 그대로 NAMED_VALUE_INT).
            # name은 mavlink 수신 시 널 패딩된 bytes로 오므로 비교 전에 정리한다.
            name = msg.name
            if isinstance(name, bytes):
                name = name.split(b"\x00", 1)[0].decode("ascii", errors="replace")
            if name == config.MAVLINK_COMET_ACK_NAME:
                sequence, command, result = config.unpack_comet_value(msg.value)
                result_name = config.LORA_ACK_RESULT_NAMES.get(result, f"UNKNOWN({result})")
                print(f"[sik] COMET_ACK seq={sequence} cmd={command} result={result_name}")
                status_queue.put({
                    "source": "sik",
                    "last_seen": now,
                    "comet_ack": {
                        "sequence": sequence,
                        "command": command,
                        "result": result,
                        "result_name": result_name,
                    },
                })

        elif msg_type == "HEARTBEAT":
            status_queue.put({"source": "sik", "last_seen": now})

        elif msg_type == "LOCAL_POSITION_NED":
            # EKF3가 계산한 위치/속도 (NED) - 그대로 ENU로 축만 바꿔 전달.
            status_queue.put({
                "source": "sik",
                "last_seen": now,
                "fc_pos_e": msg.y,
                "fc_pos_n": msg.x,
                "fc_alt": -msg.z,
                "fc_vel_e": msg.vy,
                "fc_vel_n": msg.vx,
                "fc_vel_z": -msg.vz,
            })

        elif msg_type == "SCALED_IMU":
            # FC가 보고하는 보정된 가속도 (body frame, m/s^2).
            # mavlink에는 "EKF3 가속도"라는 별도 필드가 없어 이 값을 쓴다.
            status_queue.put({
                "source": "sik",
                "last_seen": now,
                "fc_accel_x": msg.xacc * _ACCEL_MG_TO_MPS2,
                "fc_accel_y": msg.yacc * _ACCEL_MG_TO_MPS2,
                "fc_accel_z": msg.zacc * _ACCEL_MG_TO_MPS2,
            })

        elif msg_type == "SCALED_PRESSURE":
            # 자체 칼만필터의 수직(고도) 관측 - GPS(update_horizontal, ~5Hz)와
            # 독립적으로 도착하는 대로 바로 반영해 기압계의 빠른 갱신률을
            # 그대로 활용한다.
            baro_origin.set_if_needed(msg.press_abs)
            baro_alt = baro_origin.to_relative_altitude(msg.press_abs)
            kf.update_vertical(baro_alt)
            # R_vertical 실측 검증(칼만필터 튜닝)을 나중에 CSV로 할 수 있게
            # 원값도 같이 남긴다 - 전에는 필터링된 결과만 남고 이 raw 값
            # 자체는 어디에도 기록이 안 됐다.
            last_baro = (msg.press_abs, baro_alt)

        elif msg_type == "ATTITUDE":
            # roll/pitch/yaw는 mavlink 스펙상 이미 라디안 단위.
            # 3D 뷰에서 로켓 와이어프레임을 회전시키는 데 그대로 쓴다.
            # RAW_IMU의 body->ENU 가속도 회전에도 최신값을 계속 사용한다.
            last_attitude = (msg.roll, msg.pitch, msg.yaw)
            # run_sik_pipeline이 재연결 전반에 걸쳐 들고 있는 holder에도 매번
            # 다시 써준다 - 이걸 안 하면 리부트/재연결 시 holder가 초기값
            # (0,0,0)에서 영영 안 바뀌어서, "리부트 직전 마지막 실측 자세를
            # 이어받는다"는 설계 의도 자체가 무력화된다.
            last_attitude_holder["roll"] = msg.roll
            last_attitude_holder["pitch"] = msg.pitch
            last_attitude_holder["yaw"] = msg.yaw
            status_queue.put({
                "source": "sik",
                "last_seen": now,
                "attitude_roll": msg.roll,
                "attitude_pitch": msg.pitch,
                "attitude_yaw": msg.yaw,
            })
