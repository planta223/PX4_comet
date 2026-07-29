# -*- coding: utf-8 -*-
"""
GCS GUI (Dear PyGui).

SiK/LoRa 큐를 논블로킹으로 구독해서 화면을 갱신한다. 파서/필터 로직은
전부 sik.py/kalman.py/lora.py/mission_features.py에 있고, 여기서는
값을 표시만 한다.

큐 구독 루프 자리(_drain_queue/_poll_* 함수)가 나중에 ROS(rclpy)
publisher로 교체될 자리다 - sik.py/kalman.py는 손댈 필요 없다.
"""

import csv
import json
import math
import os
import queue
import threading
import time
import webbrowser
from datetime import datetime

import dearpygui.dearpygui as dpg

import config
import fusion
import lora
import mission_features
import shared_state
import sik
import stats
import viz3d

# ---------------------------------------------------------------------------
# 큐 (data/시각화 계층 분리 - sik.py/lora.py는 이 큐들을 누가 구독하는지 모른다)
# ---------------------------------------------------------------------------
trajectory_queue = queue.Queue(maxsize=config.TRAJECTORY_QUEUE_MAXSIZE)   # SiK 궤적
sik_status_queue = queue.Queue(maxsize=config.TRAJECTORY_QUEUE_MAXSIZE)  # SiK 링크/RC/prearm
lora_trajectory_queue = queue.Queue(maxsize=config.LORA_QUEUE_MAXSIZE)   # LoRa 궤적
lora_status_queue = queue.Queue(maxsize=config.LORA_QUEUE_MAXSIZE)       # LoRa 링크
lora_command_queue = queue.Queue(maxsize=config.LORA_QUEUE_MAXSIZE)      # LoRa 커맨드 요청 (GUI -> lora-pipeline)

# SiK/LoRa 파이프라인이 같은 ENU 원점(lat/lon)을 공유하도록 하나만 만들어
# 양쪽 스레드에 주입한다 - 안 그러면 두 스레드가 각자 "첫 fix"를 원점으로
# 잡아서 서로 다른 좌표계에 궤적이 그려지는 문제가 생긴다.
shared_origin = shared_state.SharedOrigin()

# FC Reboot 또는 ASCENT 전이(_start_new_flight_log) 시점에 이 값을 새 비행
# 디렉터리로 갱신해서, sik.py/lora.py 파이프라인이 연결을 안 끊고도 그
# 순간부터 새 폴더에 CSV를 쓰게 한다 (테스트/대기 중 쌓인 로그와 실제 비행
# 로그를 구분하기 위함).
flight_log_dir = shared_state.FlightLogDir(config.CSV_OUTPUT_DIR)

# 상태 전이/메시지 로그/서보 명령/링크 전환/리부트 이벤트를 모으는 저빈도
# 이벤트 로거 - flight_log_dir과 같이 움직여서 새 비행 폴더가 열리면 자동으로
# 그 비행 폴더의 events.jsonl에 쓰인다. flush 간격은 CSV와 동일한 이유로
# config.CSV_FLUSH_INTERVAL_SEC를 그대로 맞춘다.
event_logger = shared_state.EventLogger(flight_log_dir, flush_interval_sec=config.CSV_FLUSH_INTERVAL_SEC)

# ---------------------------------------------------------------------------
# GUI 상태 (메인 스레드에서만 갱신)
# ---------------------------------------------------------------------------
_start_time = time.time()

_sik_traj_e, _sik_traj_n, _sik_traj_alt = [], [], []
_lora_traj_e, _lora_traj_n, _lora_traj_alt = [], [], []
_MAX_POINTS = 2000

# 온보드 ATTITUDE (roll/pitch/yaw, rad) - 3D 뷰의 로켓 와이어프레임 회전에 사용
_attitude = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

# 발사 전(READY)에도 항상 갱신되는 "현재 위치" - 궤적 트레일(_sik_traj_*)은
# 기록 허용 상태에서만 쌓이지만, 로켓의 지금 위치는 발사 전에도 표시한다.
_live_pos = {"pos_e": 0.0, "pos_n": 0.0, "alt": 0.0}

# 3D 궤적 뷰 카메라 상태 (좌클릭 드래그로 회전, 휠로 줌)
_cam_azimuth = math.radians(config.SCENE3D_INITIAL_AZIMUTH_DEG)
_cam_elevation = math.radians(config.SCENE3D_INITIAL_ELEVATION_DEG)
_cam_distance = config.SCENE3D_INITIAL_DISTANCE_M
_prev_mouse_pos = None
_rocket_local_segments = viz3d.build_rocket_wireframe(
    config.ROCKET_MODEL_LENGTH_M,
    config.ROCKET_MODEL_DIAMETER_M,
    config.ROCKET_MODEL_FIN_SPAN_M,
    config.ROCKET_MODEL_NOSE_RATIO,
)

_last_sik_seen = None
_last_lora_seen = None
_last_rc8 = None
_last_rc9 = None
_last_gps = None
_prearm_reasons = []
_current_rocket_state = "READY"

# ASCENT로 판단된(진입한) 시각 - FORCE_DEPLOY 안전 타이머(config.
# FORCE_DEPLOY_SAFETY_DELAY_SEC) 계산과 비행 결과 요약(최고고도/비행시간)의
# 기준점으로 쓴다. READY로 돌아오면 다음 발사 사이클을 위해 리셋한다.
_ascent_entered_at = None

# 비행 결과 요약 - ASCENT 진입 이후 판단된 위치의 최고고도를 계속 갱신하고,
# LANDED가 처음 감지된 순간 비행시간(ASCENT~LANDED)을 한 번만 기록해서
# 이후 화면에 그대로 고정 표시한다 (착륙 후 시간이 계속 흘러도 안 늘어남).
_flight_max_altitude_m = 0.0
_flight_duration_sec = None

# 원격 강제전이(FORCE_*) 버튼 그룹의 잠금 해제 체크박스 상태 - 실수 클릭
# 방지용 1차 가드, 상태 일치/타이머는 _update_force_transition_buttons()가
# 별도로 검사한다.
_force_transition_armed = False

# comet_flight_state.lua의 is_command_allowed()와 동일한 "직전 상태에서만
# 허용" 규칙 - (커맨드 이름, 허용되는 현재 상태, 버튼 라벨) 튜플.
FORCE_TRANSITIONS = [
    ("FORCE_ASCENT", "READY", "강제 ASCENT"),
    ("FORCE_DEPLOY", "ASCENT", "강제 DEPLOY (사출)"),
    ("FORCE_DESCENT", "DEPLOY", "강제 DESCENT"),
    ("FORCE_LANDED", "DESCENT", "강제 LANDED"),
]

# 리부트 검증 시퀀스가 "리부트 이후 새로 도착한 패킷인지"를 판단하는 데
# 쓰는 메시지 타입별 최근 수신 시각 - _last_sik_seen은 아무 메시지에나
# 갱신돼서 이 용도로는 너무 뭉뚱그려져 있어 따로 둔다.
_last_attitude_seen = None
_last_raw_imu_seen = None

# "FC Reboot" 버튼 -> 리부트 명령 전송 -> 이 시퀀스로 재연결/재수신 자동
# 검증. 발사 인가 기능 자체가 제거돼서 이 결과는 무언가를 막는 게이트가
# 아니라 리부트 후 상태를 참고용으로 로그에 남기는 용도로만 쓰인다.
_REBOOT_VERIFY_CHECKS = ["sik_link", "raw_imu", "attitude", "lora_link"]
_REBOOT_VERIFY_LABELS = {
    "sik_link": "SiK 링크", "raw_imu": "RAW_IMU", "attitude": "ATTITUDE", "lora_link": "LoRa 링크",
}
_reboot_verify = {
    "active": False,
    "start_time": None,
    "servo_cmd_sent": False,
    "results": {name: None for name in _REBOOT_VERIFY_CHECKS},  # None=대기중, True=통과
}

# 로켓 상태 전이(READY->ASCENT->...) 타임라인을 events.jsonl에 남기기 위한
# 이전 프레임 상태 - _update_fusion()에서만 비교/갱신한다.
_last_rocket_state_for_log = None

# 이번 "발사 사이클"에 이미 새 비행 로그 폴더를 만들었는지 - FC Reboot 또는
# ASCENT 전이, 둘 중 먼저 일어나는 쪽이 폴더를 만들고, 어느 한쪽이 먼저
# 만들었으면 다른 쪽은 또 안 만든다(안 그러면 한 비행 데이터가 폴더 두
# 개로 쪼개짐). READY로 돌아오면 다시 False로 리셋된다.
_flight_log_active = False

# SiK 자체 칼만필터가 계산한 가속도/속도 (RAW_IMU 기반) - 퓨전 모듈 입력용.
# None이면 아직 한 번도 안 받은 것 (SiK 죽음/미연결과 구분).
_sik_live_accel = {"accel_e": None, "accel_n": None, "accel_z": None}
_sik_live_vel = {"vel_e": None, "vel_n": None, "vel_z": None}

# SiK STATUSTEXT / LoRa 텔레메트리가 각자 보고하는 로켓 상태 - 퓨전 모듈이
# 이 둘을 보고 최종 _current_rocket_state를 판단한다 (fusion.py 참고).
_sik_rocket_state = None
_lora_rocket_state = None

# LoRa 텔레메트리의 fault_flags 원본 비트값 - ERROR는 더 이상 별도 로켓
# 상태가 아니라 이 비트필드로만 표현한다 (config.LORA_FAULT_FLAG_NAMES로
# 이름을 풀어서 "왜" 에러인지 보여준다). _last_fault_flags_for_snapshot은
# 0 -> 0이 아님으로 막 바뀐 프레임에서만 로그를 남기기 위한 이전 값.
_lora_fault_flags = 0
_last_fault_flags_for_snapshot = 0

# LoRa가 마지막으로 보낸 위치 샘플 (퓨전의 "위치는 항상 LoRa 우선" 입력,
# 그리고 SiK 죽었을 때 폴백 칼만필터의 관측 입력으로도 쓰인다).
_last_lora_pos_sample = None

# fusion.DataFusion 판단 결과 - main()의 _update_fusion()이 매 프레임 갱신한다.
_fusion = fusion.DataFusion()
_fused = {
    "position": (0.0, 0.0, 0.0), "position_source": None,
    "attitude": (0.0, 0.0, 0.0), "attitude_source": None,
    "accel": None, "accel_source": None,
    "velocity": (0.0, 0.0, 0.0), "velocity_source": None,
    "rocket_state": "READY", "rocket_state_source": None,
    "recording_enabled": False,
}

# 고도-시간 2D 플롯 (판단된 위치의 alt 기준)
_alt_plot_t, _alt_plot_alt = [], []

# SiK vs LoRa 위치 비교 정량 통계 (stats.py) - 둘 다 살아있을 때만 기록.
# CSV 파일/writer는 main() 시작 시 _ensure_fusion_compare_csv()가 열고,
# flight_log_dir 세대가 바뀌면(발사 인가) _update_fusion_compare_stats()가
# 다음 보고 주기에 알아서 비행 폴더 것으로 다시 연다 - sik.py/lora.py의
# 로테이션과 동일한 패턴.
_fusion_compare_stats = stats.Vec3RollingStats(config.STATS_ROLLING_WINDOW)
_last_fusion_compare_report = 0.0
_fusion_compare_csv_file = None
_fusion_compare_csv_writer = None
_fusion_compare_dir_generation = None


# LANDED 상태로 막 전환된 순간에만 1회성 이벤트(스냅샷 저장, 로그 등)를
# 실행하기 위한 이전 상태 기록
_last_rocket_state_for_snapshot = None

# SiK/LoRa 링크 생존->DEAD 전환을 감지하기 위한 이전 프레임 생존 여부.
# None이면 "아직 판단 보류"(앱 시작 직후, last_seen 자체가 없는 상태) - 이때는
# 전환 메시지를 내지 않는다 (시작하자마자 "끊김" 오탐이 뜨는 걸 방지).
_sik_alive_prev = None
_lora_alive_prev = None

# 메시지 로그 콘솔 (Mission Planner 메시지창과 동일 - STATUSTEXT 표시)
_message_log_widget_ids = []
_MAV_SEVERITY_COLORS = {
    0: (255, 60, 60, 255),    # EMERGENCY
    1: (255, 60, 60, 255),    # ALERT
    2: (255, 60, 60, 255),    # CRITICAL
    3: (255, 110, 110, 255),  # ERROR
    4: (255, 200, 0, 255),    # WARNING
    5: (210, 210, 210, 255),  # NOTICE
    6: (210, 210, 210, 255),  # INFO
    7: (140, 140, 140, 255),  # DEBUG
}


def _drain_queue(q: "queue.Queue") -> list:
    items = []
    while True:
        try:
            items.append(q.get_nowait())
        except queue.Empty:
            break
    return items


def _thread_wrapper(fn, *args):
    try:
        fn(*args)
    except NotImplementedError as exc:
        print(f"[{fn.__module__}] 스텁 미구현 - 스레드 종료: {exc}")
    except Exception as exc:  # noqa: BLE001 - 파이프라인 스레드 보호용
        print(f"[{fn.__module__}] 파이프라인 예외로 종료: {exc}")


def _launch_pipeline_threads():
    threading.Thread(
        target=_thread_wrapper,
        args=(sik.run_sik_pipeline, trajectory_queue, sik_status_queue, config, shared_origin, flight_log_dir),
        daemon=True,
        name="sik-pipeline",
    ).start()

    threading.Thread(
        target=_thread_wrapper,
        args=(lora.run_lora_pipeline, lora_trajectory_queue, lora_status_queue, config, lora_command_queue, shared_origin, flight_log_dir),
        daemon=True,
        name="lora-pipeline",
    ).start()


def _poll_sik():
    global _last_sik_seen, _last_rc8, _last_rc9, _last_gps, _sik_rocket_state, _prearm_reasons
    global _last_attitude_seen, _last_raw_imu_seen

    for item in _drain_queue(trajectory_queue):
        # sik.py는 이제 로켓 상태를 모르고 무조건 흘려보내기만 하므로, "이걸
        # 실제 비행 궤적 트레일로 보여줄지"는 여기(소비자)에서 fusion.py가
        # 판단한 recording_enabled로 거른다. CSV 자체는 이미 flight_log_dir이
        # 비행 시작 시점에 디렉터리를 통째로 바꿔주므로 별도로 안 거른다.
        if not _fused["recording_enabled"]:
            continue

        _sik_traj_e.append(item["pos_e"])
        _sik_traj_n.append(item["pos_n"])
        _sik_traj_alt.append(item["alt"])
        del _sik_traj_e[:-_MAX_POINTS]
        del _sik_traj_n[:-_MAX_POINTS]
        del _sik_traj_alt[:-_MAX_POINTS]

    for item in _drain_queue(sik_status_queue):
        if "last_seen" in item:
            _last_sik_seen = item["last_seen"]
        if "rc_ch8" in item:
            _last_rc8 = item["rc_ch8"]
        if "rc_ch9" in item:
            _last_rc9 = item["rc_ch9"]
        if "last_gps" in item:
            _last_gps = item["last_gps"]
        if "rocket_state" in item:
            _sik_rocket_state = item["rocket_state"]
        if "prearm_msg" in item:
            reasons = mission_features.check_prearm_status(item["prearm_msg"])
            if reasons:
                _prearm_reasons = reasons
        if "accel_e" in item:
            _sik_live_accel["accel_e"] = item["accel_e"]
            _sik_live_accel["accel_n"] = item["accel_n"]
            _sik_live_accel["accel_z"] = item["accel_z"]
            _last_raw_imu_seen = item["last_seen"]
        if "live_vel_e" in item:
            _sik_live_vel["vel_e"] = item["live_vel_e"]
            _sik_live_vel["vel_n"] = item["live_vel_n"]
            _sik_live_vel["vel_z"] = item["live_vel_z"]
        if "attitude_roll" in item:
            _attitude["roll"] = item["attitude_roll"]
            _attitude["pitch"] = item["attitude_pitch"]
            _attitude["yaw"] = item["attitude_yaw"]
            _last_attitude_seen = item["last_seen"]
        if "statustext" in item:
            _append_message_log(item["statustext"]["text"], item["statustext"]["severity"])
        if "live_pos_e" in item:
            _live_pos["pos_e"] = item["live_pos_e"]
            _live_pos["pos_n"] = item["live_pos_n"]
            _live_pos["alt"] = item["live_alt"]


def _poll_lora():
    global _last_lora_seen, _lora_rocket_state, _last_lora_pos_sample
    global _lora_fault_flags

    for item in _drain_queue(lora_trajectory_queue):
        # 퓨전 모듈 입력용 위치 샘플은 recording_enabled와 무관하게 항상
        # 최신으로 유지한다 (fusion.py가 위치 판단에 그대로 쓰므로) - 궤적
        # "트레일" 표시만 아래에서 걸러진다.
        _last_lora_pos_sample = {
            "pos_e": item["pos_e"], "pos_n": item["pos_n"],
            "alt": item["alt"], "timestamp": item["timestamp"],
        }

        if not _fused["recording_enabled"]:
            continue

        _lora_traj_e.append(item["pos_e"])
        _lora_traj_n.append(item["pos_n"])
        _lora_traj_alt.append(item["alt"])
        del _lora_traj_e[:-_MAX_POINTS]
        del _lora_traj_n[:-_MAX_POINTS]
        del _lora_traj_alt[:-_MAX_POINTS]

    for item in _drain_queue(lora_status_queue):
        if "last_seen" in item:
            _last_lora_seen = item["last_seen"]
        if "rocket_state" in item:
            _lora_rocket_state = item["rocket_state"]
        if "fault_flags" in item:
            _lora_fault_flags = item["fault_flags"]
        if "command_result" in item:
            result = item["command_result"]
            event_logger.log("lora_command_result", **result)
            # RESET은 READY 리셋 버튼, FORCE_*는 강제전이 버튼, 나머지
            # (RELEASE_ALL/LOCK_ALL)는 서보 사출/잠금 버튼의 LoRa 폴백
            # 결과 - 각자 자기 상태 텍스트로 라우팅한다.
            if result["command_name"] == "RESET":
                target_tag = "reset_status_text"
            elif result["command_name"].startswith("FORCE_"):
                target_tag = "force_transition_status_text"
            else:
                target_tag = "servo_group_status_text"
            if result["success"]:
                dpg.set_value(
                    target_tag,
                    f"{result['command_name']} -> {result['result']} ({result['attempts']}회 시도)",
                )
            else:
                dpg.set_value(
                    target_tag,
                    f"{result['command_name']} 실패 - 응답 없음 ({result['attempts']}회 재시도)",
                )


def _ensure_fusion_compare_csv(output_dir):
    global _fusion_compare_csv_file, _fusion_compare_csv_writer
    os.makedirs(output_dir, exist_ok=True)
    filename = datetime.now().strftime(config.FUSION_COMPARE_STATS_CSV_FILENAME_PATTERN)
    filepath = os.path.join(output_dir, filename)
    is_new = not os.path.exists(filepath)

    _fusion_compare_csv_file = open(filepath, mode="a", newline="", encoding="utf-8")
    _fusion_compare_csv_writer = csv.writer(_fusion_compare_csv_file)
    if is_new:
        _fusion_compare_csv_writer.writerow([
            "timestamp", "diff_mean_e", "diff_mean_n", "diff_mean_alt",
            "diff_std_e", "diff_std_n", "diff_std_alt",
        ])
        _fusion_compare_csv_file.flush()


def _open_fusion_compare_csv():
    """main() 시작 시 1회 호출 - 그 시점의 flight_log_dir 위치(대기 중이면 기본 logs/)로 연다."""
    global _fusion_compare_dir_generation
    output_dir, generation = flight_log_dir.current()
    _ensure_fusion_compare_csv(output_dir)
    _fusion_compare_dir_generation = generation


def _update_fusion_compare_stats(now, sik_alive, lora_alive, sik_pos):
    """
    SiK/LoRa가 둘 다 살아있을 때만 원시 위치 차이(SiK - LoRa)를 기록한다 -
    "SiK/LoRa가 서로 얼마나 일치하는지"를 정량화하는 용도 (둘 중 하나만
    있으면 비교 자체가 불가능하니 그 주기는 건너뛴다).
    """
    global _last_fusion_compare_report, _fusion_compare_dir_generation

    output_dir, generation = flight_log_dir.current()
    if generation != _fusion_compare_dir_generation:
        _fusion_compare_csv_file.close()
        _ensure_fusion_compare_csv(output_dir)
        _fusion_compare_dir_generation = generation

    if sik_alive and lora_alive and _last_lora_pos_sample is not None:
        diff = (
            sik_pos[0] - _last_lora_pos_sample["pos_e"],
            sik_pos[1] - _last_lora_pos_sample["pos_n"],
            sik_pos[2] - _last_lora_pos_sample["alt"],
        )
        _fusion_compare_stats.record(diff)

    if now - _last_fusion_compare_report >= config.STATS_REPORT_INTERVAL_SEC:
        _last_fusion_compare_report = now
        mean = _fusion_compare_stats.mean()
        std = _fusion_compare_stats.stddev()
        print(
            f"[fusion-compare] SiK-LoRa 위치차 평균(e,n,alt)=({mean[0]:.3f},{mean[1]:.3f},{mean[2]:.3f}) "
            f"표준편차=({std[0]:.3f},{std[1]:.3f},{std[2]:.3f})"
        )
        if _fusion_compare_csv_writer is not None:
            _fusion_compare_csv_writer.writerow([now, *mean, *std])
            _fusion_compare_csv_file.flush()


def _update_fusion():
    """
    fusion.DataFusion을 매 프레임 호출해 SiK/LoRa 중 어느 값을 GUI에 올릴지
    판단한다. _current_rocket_state는 이제 이 함수에서만 설정된다 (기존에
    _poll_sik이 직접 덮어쓰던 것을 대체).
    """
    global _current_rocket_state, _fused, _last_rocket_state_for_log, _flight_log_active
    global _ascent_entered_at, _flight_max_altitude_m, _flight_duration_sec

    now = time.time()
    sik_alive = fusion.is_link_alive(_last_sik_seen, config.SIK_LINK_TIMEOUT_SEC, now)
    lora_alive = fusion.is_link_alive(_last_lora_seen, config.LORA_LINK_TIMEOUT_SEC, now)

    sik_pos = (_live_pos["pos_e"], _live_pos["pos_n"], _live_pos["alt"])
    sik_attitude_rad = (_attitude["roll"], _attitude["pitch"], _attitude["yaw"])
    sik_accel = None
    if _sik_live_accel["accel_e"] is not None:
        sik_accel = (_sik_live_accel["accel_e"], _sik_live_accel["accel_n"], _sik_live_accel["accel_z"])
    sik_vel = None
    if _sik_live_vel["vel_e"] is not None:
        sik_vel = (_sik_live_vel["vel_e"], _sik_live_vel["vel_n"], _sik_live_vel["vel_z"])

    _fused = _fusion.update(
        now, sik_alive, lora_alive,
        sik_pos, sik_attitude_rad, sik_accel, sik_vel, _sik_rocket_state,
        _last_lora_pos_sample, _lora_rocket_state,
    )
    _current_rocket_state = _fused["rocket_state"]
    if _last_rocket_state_for_log is not None and _current_rocket_state != _last_rocket_state_for_log:
        event_logger.log("rocket_state_transition", from_state=_last_rocket_state_for_log, to_state=_current_rocket_state)
        # FC Reboot를 안 거치고(예: 리부트 없이 지상 테스트하다 바로 ASCENT로
        # 넘어가는 경우) 바로 ASCENT가 되는 경우를 위한 두 번째 트리거 -
        # _start_new_flight_log이 이미 시작된 건 알아서 걸러준다.
        if _current_rocket_state == "ASCENT":
            _start_new_flight_log(trigger="ascent")
            _ascent_entered_at = now
            _flight_max_altitude_m = 0.0
        elif _current_rocket_state == "READY":
            _flight_log_active = False
            _ascent_entered_at = None
            _flight_max_altitude_m = 0.0
            _flight_duration_sec = None
    _last_rocket_state_for_log = _current_rocket_state

    if _ascent_entered_at is not None:
        _flight_max_altitude_m = max(_flight_max_altitude_m, _fused["position"][2])

    _alt_plot_t.append(now - _start_time)
    _alt_plot_alt.append(_fused["position"][2])
    del _alt_plot_t[:-_MAX_POINTS]
    del _alt_plot_alt[:-_MAX_POINTS]

    _update_fusion_compare_stats(now, sik_alive, lora_alive, sik_pos)


def _update_camera_from_mouse():
    """
    3D 뷰 좌클릭 드래그 -> 궤도 회전. 매 프레임 폴링 방식으로 처리해서
    큐 폴링 루프와 동일한 스타일을 유지한다 (별도 이벤트 콜백 없음).
    """
    global _cam_azimuth, _cam_elevation, _prev_mouse_pos

    mx, my = dpg.get_mouse_pos(local=False)
    if dpg.is_item_hovered("scene3d_drawlist") and dpg.is_mouse_button_down(dpg.mvMouseButton_Left):
        if _prev_mouse_pos is not None:
            dx = mx - _prev_mouse_pos[0]
            dy = my - _prev_mouse_pos[1]
            _cam_azimuth -= math.radians(dx * config.SCENE3D_ORBIT_SPEED_DEG_PER_PX)
            _cam_elevation += math.radians(dy * config.SCENE3D_ORBIT_SPEED_DEG_PER_PX)
            elev_limit = math.radians(85.0)
            _cam_elevation = max(-elev_limit, min(elev_limit, _cam_elevation))
    _prev_mouse_pos = (mx, my)


def _on_mouse_wheel(sender, app_data):
    """휠 스크롤로 3D 뷰 줌 (드로잉 영역 위에 있을 때만)."""
    global _cam_distance
    if not dpg.is_item_hovered("scene3d_drawlist"):
        return
    factor = (1.0 - config.SCENE3D_ZOOM_STEP_RATIO) if app_data > 0 else (1.0 + config.SCENE3D_ZOOM_STEP_RATIO)
    _cam_distance = max(config.SCENE3D_MIN_DISTANCE_M, min(config.SCENE3D_MAX_DISTANCE_M, _cam_distance * factor))


_SCREEN_COORD_MARGIN_FACTOR = 20  # 캔버스 크기의 이 배수를 넘는 좌표는 화면 밖으로 간주하고 버린다


def _draw_segment_3d(cam: "viz3d.Camera", width: float, height: float, p1_world, p2_world, color, thickness):
    """
    카메라가 대상에 아주 가까이 있으면(예: 궤적 데이터가 아직 없어 auto-fit
    거리가 최소값으로 클램프된 상태) near-plane 클리핑을 통과한 점이라도
    화면 가장자리 근처에서 원근 나눗셈이 거의 0으로 나뉘어 픽셀 좌표가
    수만 단위로 튀는 경우가 있다. dpg.draw_line에 이런 극단적인 좌표를
    넘기면 그 프레임의 draw_layer 전체가 깨져서 아무것도 안 그려지는
    문제가 있어, 캔버스 크기 대비 비정상적으로 먼 좌표는 그냥 버린다.
    """
    a = cam.project_to_screen(p1_world, width, height)
    b = cam.project_to_screen(p2_world, width, height)
    if a is None or b is None:
        return
    margin_x = width * _SCREEN_COORD_MARGIN_FACTOR
    margin_y = height * _SCREEN_COORD_MARGIN_FACTOR
    for x, y in (a, b):
        if abs(x) > margin_x or abs(y) > margin_y:
            return
    dpg.draw_line(a, b, color=color, thickness=thickness, parent="scene3d_layer")


def _draw_grid(cam: "viz3d.Camera", width: float, height: float):
    half = config.SCENE3D_GRID_HALF_EXTENT_M
    step = config.SCENE3D_GRID_STEP_M
    n_lines = int(half / step)
    color = (100, 100, 100, 140)
    for i in range(-n_lines, n_lines + 1):
        offset = i * step
        _draw_segment_3d(cam, width, height, (-half, 0.0, offset), (half, 0.0, offset), color, 1.0)
        _draw_segment_3d(cam, width, height, (offset, 0.0, -half), (offset, 0.0, half), color, 1.0)


def _draw_trajectory_3d(cam: "viz3d.Camera", width: float, height: float, es, ups, ns, color, dashed=False):
    n = len(es)
    if n < 2:
        return
    step = 2 if dashed else 1
    for i in range(0, n - 1, step):
        p1 = (es[i], ups[i], ns[i])
        p2 = (es[i + 1], ups[i + 1], ns[i + 1])
        _draw_segment_3d(cam, width, height, p1, p2, color, 2.0)


def _draw_rocket(cam: "viz3d.Camera", width: float, height: float):
    """로켓 3D 모형은 퓨전 모듈이 판단한 위치/자세를 그대로 따라간다."""
    pos_e, pos_n, pos_alt = _fused["position"]
    roll, pitch, yaw = _fused["attitude"]

    color = (255, 200, 0, 255)
    for local_p1, local_p2 in _rocket_local_segments:
        w1 = viz3d.body_to_world(local_p1, roll, pitch, yaw, pos_e, pos_n, pos_alt)
        w2 = viz3d.body_to_world(local_p2, roll, pitch, yaw, pos_e, pos_n, pos_alt)
        _draw_segment_3d(cam, width, height, w1, w2, color, 2.0)


def _update_3d_scene():
    # dearpygui 2.3.1에서 drawlist에 width=-1(자동채움)을 쓰면 child_window
    # 안에 있을 때 draw_layer 내용이 전혀 렌더링되지 않는 버그가 확인됨
    # (재현 스크립트로 검증: 고정폭은 정상, -1은 완전히 안 보임). 그래서
    # drawlist는 고정폭으로 만들어두고(_build_gui), 매 프레임 부모
    # (right_column)의 실제 폭을 읽어 configure_item으로 강제 갱신한다.
    parent_w, _parent_h = dpg.get_item_rect_size("right_column")
    width = int(parent_w - 20) if parent_w and parent_w > 40 else 1000
    height = 550
    dpg.configure_item("scene3d_drawlist", width=width, height=height)
    aspect = width / height

    # 카메라는 판단된 위치를 수평(E/N)으로만 따라간다 - 고도(Y)는 지면(0.0)에
    # 고정해서, 로켓이 상승해도 카메라가 같이 따라 올라가 "얼마나 높이
    # 올라갔는지" 감각을 잃는 일이 없도록 한다. 줌(카메라 거리)은 항상
    # 수동(휠 스크롤, _cam_distance)으로만 조절한다.
    pos_e, pos_n, _pos_alt = _fused["position"]
    target = (pos_e, 0.0, pos_n)
    cam_distance = _cam_distance

    eye = viz3d.orbit_eye_position(target, _cam_azimuth, _cam_elevation, cam_distance)
    cam = viz3d.Camera(
        eye, target, up=(0.0, 1.0, 0.0),
        fov_rad=math.radians(config.SCENE3D_FOV_DEG),
        aspect=aspect, near=config.SCENE3D_NEAR, far=config.SCENE3D_FAR,
    )

    dpg.delete_item("scene3d_layer", children_only=True)

    _draw_grid(cam, width, height)
    _draw_trajectory_3d(cam, width, height, _sik_traj_e, _sik_traj_alt, _sik_traj_n, color=(0, 220, 0, 255))
    _draw_trajectory_3d(cam, width, height, _lora_traj_e, _lora_traj_alt, _lora_traj_n, color=(220, 0, 220, 255), dashed=True)
    _draw_rocket(cam, width, height)


def _append_message_log(text: str, severity: int):
    """STATUSTEXT 한 줄을 메시지 로그 콘솔에 추가하고 맨 아래로 스크롤한다."""
    event_logger.log("message", text=text, severity=severity)
    color = _MAV_SEVERITY_COLORS.get(severity, (210, 210, 210, 255))
    widget_id = dpg.add_text(text, color=color, parent="message_log_window")
    _message_log_widget_ids.append(widget_id)
    if len(_message_log_widget_ids) > config.MESSAGE_LOG_MAX_LINES:
        dpg.delete_item(_message_log_widget_ids.pop(0))
    dpg.set_y_scroll("message_log_window", dpg.get_y_scroll_max("message_log_window"))


def _save_trajectory_snapshot():
    # 발사 인가 전이면 flight_log_dir이 아직 기본 logs/를 가리키고 있어서
    # 예전처럼 logs/snapshots/에 남고, 인가 후라면 그 비행 폴더 밑
    # snapshots/에 남는다 - 특정 비행의 스냅샷을 그 비행 폴더 안에서 바로
    # 찾을 수 있게(예전엔 logs/snapshots/에 전부 섞여 있었음).
    flight_dir, _generation = flight_log_dir.current()
    output_dir = os.path.join(flight_dir, "snapshots")
    os.makedirs(output_dir, exist_ok=True)
    filename = datetime.now().strftime(config.SNAPSHOT_FILENAME_PATTERN)
    filepath = os.path.join(output_dir, filename)
    dpg.output_frame_buffer(file=filepath)
    dpg.set_value("snapshot_status_text", f"착륙 감지 - 화면 스냅샷 저장됨: {filepath}")
    _append_message_log(f"착륙 스냅샷 저장됨: {filepath}", severity=6)


def _check_landed_snapshot():
    """
    LANDED 상태로 막 전환된 프레임, 또는 fault_flags가 0 -> 0이 아님으로 막
    바뀐 프레임에서만 1회성 이벤트를 실행한다.
    LANDED: 비행시간(ASCENT~지금) 고정 기록 + 스냅샷 저장 + (GPS 있으면)
    회수 좌표 지도를 브라우저로 자동 표시. 최고고도는 _update_fusion()이
    ASCENT 진입 이후 매 프레임 갱신해온 값을 그대로 쓴다(여기선 안 건드림).
    fault_flags: 메시지 로그에 강조 표시 - "왜"인지 이름까지 남긴다
    (DEPLOY_COMMAND_FAILED면 낙하산 사출 명령을 보냈는데 실패했다는 뜻).
    """
    global _last_rocket_state_for_snapshot, _last_fault_flags_for_snapshot, _flight_duration_sec
    if _current_rocket_state == "LANDED" and _last_rocket_state_for_snapshot != "LANDED":
        if _ascent_entered_at is not None:
            _flight_duration_sec = time.time() - _ascent_entered_at
        _save_trajectory_snapshot()
        if _last_gps is not None:
            webbrowser.open(mission_features.get_recovery_location(_last_gps))
    if _lora_fault_flags != 0 and _last_fault_flags_for_snapshot == 0:
        names = [name for bit, name in config.LORA_FAULT_FLAG_NAMES.items() if _lora_fault_flags & bit]
        _append_message_log(f"FC 오류 플래그 발생: {', '.join(names)}", severity=2)
    _last_rocket_state_for_snapshot = _current_rocket_state
    _last_fault_flags_for_snapshot = _lora_fault_flags


def _check_link_transition(label: str, last_seen, timeout_sec: float, prev_alive):
    """
    링크 생존 여부의 이전/현재 프레임을 비교해 전환 시에만 메시지 로그에 남긴다.
    prev_alive가 None이면(아직 판단 보류) 메시지 없이 현재 상태만 반환한다.
    """
    if last_seen is None:
        return prev_alive
    alive = (time.time() - last_seen) <= timeout_sec
    if prev_alive is not None and alive != prev_alive:
        if alive:
            _append_message_log(f"{label} 링크 복귀", severity=6)
        else:
            _append_message_log(f"{label} 링크 끊김 감지", severity=4)
    return alive


def _check_link_transitions():
    global _sik_alive_prev, _lora_alive_prev
    _sik_alive_prev = _check_link_transition("SiK", _last_sik_seen, config.SIK_LINK_TIMEOUT_SEC, _sik_alive_prev)
    _lora_alive_prev = _check_link_transition("LoRa", _last_lora_seen, config.LORA_LINK_TIMEOUT_SEC, _lora_alive_prev)


def _make_servo_send_callback(channel: int):
    def _callback():
        status_tag = f"servo_status_{channel}"
        conn = sik.get_connection()
        if conn is None:
            dpg.set_value(status_tag, "SiK 연결 없음 - 전송 실패")
            return
        pwm = dpg.get_value(f"servo_slider_{channel}")
        mission_features.set_servo(conn, channel, pwm)
        event_logger.log("servo_command", channel=channel, pwm_us=pwm, path="sik")
        dpg.set_value(status_tag, f"CH{channel} -> {pwm}us 전송됨")
    return _callback


def _make_all_servo_preset_callback(pwm_key: str, lora_command_name: str):
    """
    "사출(전체)"/"잠금(전체)" 버튼용 - comet_main.lua의 servo.lock_all()/
    release_all()과 같은 개념을 SiK/LoRa 어느 경로로 보내든 동일한 명령
    코드(config.LORA_COMMAND, 예: RELEASE_ALL=6/LOCK_ALL=5)로 전송한다.

    예전에는 SiK 연결 시 MAV_CMD_DO_SET_SERVO로 각 채널에 직접 PWM을
    씀 - 이건 ArduPilot 펌웨어가 comet_*.lua 상태머신을 거치지 않고 즉시
    처리하는 명령이라, READY 상태에서 실수로 눌러도 검증 없이 서보가
    움직이고 comet_command_queue/ACK/로그 어디에도 남지 않는 문제가 있었다
    (SiK만 상태머신을 우회하는 구조). 이제는 SiK도 comet_mavlink.lua가
    파싱하는 COMET_CMD(NAMED_VALUE_INT)로 같은 명령 코드를 보내
    LoRa와 완전히 동일한 큐/상태검증/ACK 경로를 거친다.

    SiK 연결이 없으면 LoRa로 동일 명령을 폴백 전송한다. SiK 연결 판정은
    기존 _make_servo_send_callback/_on_reboot_fc와 동일하게
    sik.get_connection()이 있는지로 본다.
    """
    command_code = config.LORA_COMMAND[lora_command_name]

    def _callback():
        conn = sik.get_connection()
        if conn is not None:
            sequence = mission_features.send_mavlink_command(conn, command_code)

            # 실제 상태 반영은 comet_mavlink.lua의 ACK(sik.py가 수신)로
            # 확인해야 하지만, 버튼을 누른 즉시 의도한 프리셋 PWM을
            # 슬라이더에 표시해 조작 피드백을 준다 (전송 여부와 무관한
            # 표시 전용 - 실제 서보 반영 확인은 ACK/텔레메트리로 판단할 것).
            for servo in config.SERVO_CHANNELS:
                ch = servo["channel"]
                pwm = servo.get(pwm_key, 1500)
                dpg.set_value(f"servo_slider_{ch}", pwm)
                dpg.set_value(f"servo_status_{ch}", f"CH{ch} -> {lora_command_name} 전송됨 (SiK seq={sequence})")

            event_logger.log("servo_command", channel="all", lora_command=lora_command_name, path="sik", sequence=sequence)
            dpg.set_value("servo_group_status_text", f"SiK 경유 {lora_command_name} 전송 (seq={sequence})")
        else:
            mission_features.send_lora_command(lora_command_queue, lora_command_name)
            event_logger.log("servo_command", channel="all", lora_command=lora_command_name, path="lora")
            dpg.set_value("servo_group_status_text", f"SiK 없음 - LoRa {lora_command_name} 전송 중...")
    return _callback


def _format_link_status(label: str, last_seen, timeout_sec: float) -> str:
    if last_seen is None:
        return f"{label}: 수신 없음"
    elapsed = time.time() - last_seen
    alive = elapsed <= timeout_sec
    return f"{label}: {'생존' if alive else 'DEAD'} (마지막 수신 {elapsed:.1f}s 전)"


def _format_reboot_verify_text(checks, elapsed=None, done=False, timed_out=False):
    parts = [f"{_REBOOT_VERIFY_LABELS[name]}:{'OK' if checks[name] else '...'}" for name in _REBOOT_VERIFY_CHECKS]
    line = " / ".join(parts)
    if done:
        return f"[검증 완료] {line} (서보 잠금 명령 전송됨)"
    if timed_out:
        failed = [_REBOOT_VERIFY_LABELS[n] for n in _REBOOT_VERIFY_CHECKS if not checks[n]]
        return f"[검증 실패/타임아웃] 응답 없음: {', '.join(failed)}"
    return f"[검증 중 {elapsed:.0f}s] {line}"


def _start_reboot_verify():
    """
    FC 리부트 직후 SiK/LoRa 링크와 FC 자세(ATTITUDE)가 다시 붙는지 자동으로
    지켜보고 events.jsonl에 결과를 남기는 진단용 시퀀스. 발사 인가 게이트
    역할은 하지 않고(그 기능 자체가 제거됨) 리부트 후 상태를 로그로 남기는
    용도만 한다.
    """
    global _reboot_verify
    _reboot_verify = {
        "active": True,
        "start_time": time.time(),
        "servo_cmd_sent": False,
        "results": {name: None for name in _REBOOT_VERIFY_CHECKS},
    }


def _update_reboot_verify():
    if not _reboot_verify["active"]:
        return

    start = _reboot_verify["start_time"]
    now = time.time()
    checks = _reboot_verify["results"]

    # 리부트 시작 이후에 실제로 새로 도착한 패킷인지(리부트 전 캐시된 값을
    # 통과로 오인하지 않도록) last_seen이 start 이후인지로 판단한다.
    if not checks["sik_link"] and _last_sik_seen is not None and _last_sik_seen >= start:
        checks["sik_link"] = True
    if not checks["raw_imu"] and _last_raw_imu_seen is not None and _last_raw_imu_seen >= start:
        checks["raw_imu"] = True
    if not checks["attitude"] and _last_attitude_seen is not None and _last_attitude_seen >= start:
        checks["attitude"] = True
    if not checks["lora_link"] and _last_lora_seen is not None and _last_lora_seen >= start:
        checks["lora_link"] = True

    # SiK 재연결이 확인되는 즉시 서보를 잠금 각도로 세팅 - ACK가 없는 fire-
    # and-forget 전송이라 "명령을 보냈다"까지만 보장하고, 실제로 물리적으로
    # 잠겼는지는 (서보 각도 피드백 하드웨어가 없는 지금은) 육안으로 확인해야 한다.
    if not _reboot_verify["servo_cmd_sent"] and checks["sik_link"]:
        conn = sik.get_connection()
        if conn is not None:
            for servo in config.SERVO_CHANNELS:
                mission_features.set_servo(conn, servo["channel"], servo.get("pwm_min", 1000))
            _reboot_verify["servo_cmd_sent"] = True
            event_logger.log("reboot_verify_servo_lock_sent", channels=[s["channel"] for s in config.SERVO_CHANNELS])

    elapsed = now - start
    all_done = all(checks.values())
    timed_out = elapsed >= config.REBOOT_VERIFY_TIMEOUT_SEC

    if all_done:
        _reboot_verify["active"] = False
        dpg.set_value("reboot_status_text", _format_reboot_verify_text(checks, done=True))
        event_logger.log("reboot_verify_result", passed=True, elapsed_sec=elapsed, checks=checks)
    elif timed_out:
        _reboot_verify["active"] = False
        dpg.set_value("reboot_status_text", _format_reboot_verify_text(checks, timed_out=True))
        event_logger.log("reboot_verify_result", passed=False, elapsed_sec=elapsed, checks=checks)
    else:
        dpg.set_value("reboot_status_text", _format_reboot_verify_text(checks, elapsed=elapsed))


def _write_flight_manifest(flight_dir):
    """
    비행 디렉터리 안에 manifest.json을 하나 남긴다 - 사후분석 도구가 CSV
    숫자만 보고는 알 수 없는 "이 비행 때 무슨 설정/검증 상태였는지"를 담는다.
    실패해도(디스크 문제 등) 비행 진행 자체를 막으면 안 되므로 예외를 삼킨다.
    """
    origin_lat, origin_lon = shared_origin.get_origin()
    manifest = {
        "flight_dir": flight_dir,
        "started_at": datetime.now().isoformat(),
        "reboot_verify": _reboot_verify["results"],
        "origin_lat": origin_lat,
        "origin_lon": origin_lon,
        "kf_observation_noise_horizontal": config.KF_OBSERVATION_NOISE_HORIZONTAL.diagonal().tolist(),
        "kf_observation_noise_vertical": config.KF_OBSERVATION_NOISE_VERTICAL.diagonal().tolist(),
        "servo_channels": config.SERVO_CHANNELS,
        "sik_connection_string": config.SIK_CONNECTION_STRING,
        "sik_baudrate": config.SIK_BAUDRATE,
        "lora_serial_port": config.LORA_SERIAL_PORT,
        "lora_baudrate": config.LORA_BAUDRATE,
    }
    try:
        os.makedirs(flight_dir, exist_ok=True)
        with open(os.path.join(flight_dir, "manifest.json"), "w", encoding="utf-8") as f:
            json.dump(manifest, f, ensure_ascii=False, indent=2)
    except OSError as exc:
        print(f"[main] manifest.json 기록 실패 (비행 진행에는 지장 없음): {exc}")


def _start_new_flight_log(trigger: str):
    """
    새 비행 로그 폴더를 시작한다 - FC Reboot(검증 시작)과 ASCENT 전이 중
    먼저 일어나는 쪽에서 호출된다(_flight_log_active로 중복 생성 방지).
    발사 인가 기능은 제거됐고, 이 로그 폴더 생성 로직은 원래부터 그 기능과
    분리돼 있었다.
    """
    global _flight_log_active
    if _flight_log_active:
        return
    _flight_log_active = True

    new_dir = os.path.join(config.CSV_OUTPUT_DIR, "flight_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    flight_log_dir.request_new_dir(new_dir)
    _write_flight_manifest(new_dir)
    event_logger.log("flight_log_started", flight_dir=new_dir, trigger=trigger)


def _on_reboot_fc():
    conn = sik.get_connection()
    if conn is None:
        dpg.set_value("reboot_status_text", "SiK 연결 없음 - 리부트 실패")
        event_logger.log("reboot_requested", success=False, reason="sik 연결 없음")
        return
    mission_features.reboot_fc(conn)
    event_logger.log("reboot_requested", success=True)
    _start_reboot_verify()
    _start_new_flight_log(trigger="reboot")
    dpg.set_value("reboot_status_text", "FC 리부트 명령 전송됨 - 재연결/자세 검증 대기 중...")


def _on_state_reset():
    global _flight_log_active
    mission_features.send_state_reset(lora_command_queue)
    dpg.set_value("reset_status_text", "전송 중... (LoRa)")

    # 상태를 READY로 되돌리는 것 = 다음 비행 준비 시작이니 _flight_log_active도
    # 같이 풀어서 다음 리부트/ASCENT 때 새 비행 폴더가 다시 만들어지게 한다.
    _flight_log_active = False


def _on_toggle_force_transition_armed(sender, app_data):
    global _force_transition_armed
    _force_transition_armed = app_data
    event_logger.log("force_transition_armed_toggled", armed=app_data)


def _make_force_transition_callback(command_name: str):
    """FORCE_* 버튼 콜백 - 활성화/상태/타이머 가드는 버튼 enabled 여부로
    이미 걸려 있으므로(_update_force_transition_buttons), 여기선 그냥
    커맨드를 큐에 넣기만 한다."""
    def callback():
        lora.enqueue_command(lora_command_queue, command_name)
        dpg.set_value("force_transition_status_text", f"{command_name} 전송 중... (LoRa)")
        event_logger.log("force_transition_sent", command_name=command_name)
    return callback


def _load_korean_font():
    """
    Dear PyGui 기본 폰트는 한글 글리프가 없어 "???"로 깨진다.
    config.KOREAN_FONT_PATH(기본: 맑은 고딕)를 로드해 전체 기본 폰트로 바인딩한다.
    """
    if not os.path.exists(config.KOREAN_FONT_PATH):
        print(f"[main] 한글 폰트를 찾을 수 없음: {config.KOREAN_FONT_PATH} - 한글이 깨질 수 있습니다.")
        return None
    with dpg.font_registry():
        font_id = dpg.add_font(config.KOREAN_FONT_PATH, config.KOREAN_FONT_SIZE)
    return font_id


def _build_gui():
    dpg.create_context()
    dpg.create_viewport(title="ROK2 GCS", width=1600, height=900)

    korean_font = _load_korean_font()
    if korean_font is not None:
        dpg.bind_font(korean_font)

    with dpg.theme() as highlight_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (46, 150, 79, 255))
    with dpg.theme() as normal_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (60, 60, 60, 255))
    with dpg.theme() as error_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (200, 40, 40, 255))

    with dpg.handler_registry():
        dpg.add_mouse_wheel_handler(callback=_on_mouse_wheel)

    with dpg.window(label="ROK2 GCS", tag="main_window"):
        with dpg.group(horizontal=True):
            # -----------------------------------------------------------
            # 왼쪽 컬럼: 로켓 정보 / 메시지 로그 / 제어 파트 (그룹핑)
            # -----------------------------------------------------------
            with dpg.child_window(width=480, tag="left_column"):
                dpg.add_text("로켓 정보")
                dpg.add_separator()
                dpg.add_text("State Machine Timeline")
                with dpg.group(horizontal=True):
                    for state in config.ROCKET_STATES:
                        dpg.add_button(label=state, tag=f"state_box_{state}", enabled=False)
                dpg.add_text("", tag="recording_status_text")
                dpg.add_text("", tag="snapshot_status_text")
                with dpg.group(horizontal=True):
                    dpg.add_text("SiK: 수신 없음", tag="sik_link_text")
                    dpg.add_spacer(width=30)
                    dpg.add_text("LoRa: 수신 없음", tag="lora_link_text")
                dpg.add_text("판단된 로켓 상태 (SiK/LoRa 중 자동 선택 - fusion.py)")
                # ASCENT 진입 이후 실시간으로 올라가다가 LANDED 시점에
                # _flight_duration_sec으로 고정된다 (착륙창의 요약과 같은 값).
                dpg.add_text("비행시간(T+): --", tag="flight_timer_text")
                dpg.add_text("위치(E/N/Alt): --", tag="fc_pos_text")
                dpg.add_text("속도(E/N/Z): --", tag="fc_vel_text")
                dpg.add_text("가속도(X/Y/Z, body frame): --", tag="fc_accel_text")
                dpg.add_text("자세(R/P/Y): --", tag="fused_attitude_text")
                dpg.add_text("원격 전개 (RC ch8/9) - 표시 전용, 제어 로직 없음")
                dpg.add_text("CH8: --   CH9: --", tag="rc_text")
                dpg.add_separator()

                dpg.add_text("메시지 로그")
                dpg.add_separator()
                dpg.add_text("메시지 로그 (mavlink STATUSTEXT + GCS 자체 이벤트)")
                with dpg.child_window(height=config.MESSAGE_LOG_HEIGHT_PX, tag="message_log_window"):
                    pass
                dpg.add_text("회수 좌표", tag="recovery_label", show=False)
                dpg.add_text("GPS 없음", tag="recovery_text", show=False)
                dpg.add_text("", tag="flight_summary_text", show=False)
                dpg.add_text("Pre-arm 상태", tag="prearm_label")
                dpg.add_text("이상 없음", tag="prearm_text")
                # FC(comet_flight_state.lua)의 fault_flags - Pre-arm(mavlink
                # PREARM 메시지)과는 별개 출처라 텍스트는 따로 두되, 발사 전
                # 운영자가 같이 확인하는 항목이라 Pre-arm 옆에 둔다.
                dpg.add_text("이상 플래그: 없음", tag="fault_flag_text")
                dpg.add_separator()

                dpg.add_text("제어 파트")
                dpg.add_separator()
                dpg.add_text("서보 일괄 제어 - SiK 우선, 연결 없으면 LoRa로 폴백")
                with dpg.group(horizontal=True):
                    deploy_btn = dpg.add_button(
                        label="사출 (전체)", width=140, height=44,
                        callback=_make_all_servo_preset_callback("pwm_max", "RELEASE_ALL"),
                    )
                    dpg.bind_item_theme(deploy_btn, error_theme)
                    lock_btn = dpg.add_button(
                        label="잠금 (전체)", width=140, height=44,
                        callback=_make_all_servo_preset_callback("pwm_min", "LOCK_ALL"),
                    )
                    dpg.bind_item_theme(lock_btn, normal_theme)
                dpg.add_text("", tag="servo_group_status_text")
                dpg.add_text("서보 개별 제어 (세부 조정용, SiK 경유)")
                for servo in config.SERVO_CHANNELS:
                    ch = servo["channel"]
                    with dpg.group(horizontal=True):
                        dpg.add_text(f"{servo['label']} (CH{ch})")
                        dpg.add_slider_int(
                            tag=f"servo_slider_{ch}",
                            default_value=servo.get("pwm_default", 1500),
                            min_value=servo.get("pwm_min", 1000),
                            max_value=servo.get("pwm_max", 2000),
                            width=200,
                        )
                        dpg.add_button(label="전송", callback=_make_servo_send_callback(ch))
                        dpg.add_text("", tag=f"servo_status_{ch}")

                with dpg.collapsing_header(label="기타 제어"):
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="FC Reboot", callback=_on_reboot_fc)
                        dpg.add_text("", tag="reboot_status_text")
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="READY 상태 리셋 (LoRa)", callback=_on_state_reset)
                        dpg.add_text("", tag="reset_status_text")

                # 기본 접힘 - 원격 강제전이는 FC의 정상 안전 조건(고도/속도/
                # 시간 확인)을 건너뛰므로 실수 클릭 방지를 위해 별도 섹션에
                # 분리하고, 잠금 해제 체크박스로 한 번 더 막는다.
                with dpg.collapsing_header(label="원격 강제전이 (위험 - 정상 안전조건 건너뜀)", default_open=False):
                    dpg.add_checkbox(
                        label="강제전이 조작 활성화",
                        tag="force_transition_armed_checkbox", default_value=False,
                        callback=_on_toggle_force_transition_armed,
                    )
                    for command_name, required_state, label in FORCE_TRANSITIONS:
                        btn = dpg.add_button(
                            label=label, tag=f"force_transition_btn_{command_name}", enabled=False,
                            callback=_make_force_transition_callback(command_name),
                        )
                        dpg.bind_item_theme(btn, error_theme)
                    dpg.add_text(
                        f"FORCE_DEPLOY는 ASCENT 진입 후 {config.FORCE_DEPLOY_SAFETY_DELAY_SEC:.0f}초가 지나야 눌립니다.",
                    )
                    dpg.add_text("", tag="force_deploy_timer_text")
                    dpg.add_text("", tag="force_transition_status_text")

            # -----------------------------------------------------------
            # 오른쪽 컬럼: 궤적 3D 뷰 (남은 폭 전부)
            # -----------------------------------------------------------
            with dpg.child_window(width=-1, tag="right_column"):
                dpg.add_text("궤적 3D 뷰 (녹색: SiK 실선 / 보라: LoRa 점선근사 / 노랑: 로켓 자세, 카메라는 판단된 위치를 수평으로 따라감) - 좌클릭 드래그: 회전, 휠: 줌")
                # width=-1(자동채움)을 drawlist에 쓰면 child_window 안에서
                # 렌더링이 안 되는 dearpygui 2.3.1 문제가 있어(재현/확인됨) -
                # 초기값은 임의 고정폭으로 두고, _update_3d_scene()에서 매
                # 프레임 부모 폭에 맞춰 configure_item으로 실제 폭을 갱신한다.
                with dpg.drawlist(width=1000, height=550, tag="scene3d_drawlist"):
                    with dpg.draw_layer(tag="scene3d_layer"):
                        pass

                with dpg.plot(label="고도 (판단된 위치 기준)", height=200, width=-1):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="alt_plot_x_axis")
                    with dpg.plot_axis(dpg.mvYAxis, label="Altitude (m)", tag="alt_plot_y_axis"):
                        dpg.add_line_series([], [], label="Alt", tag="alt_plot_series")

    dpg.set_primary_window("main_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    return highlight_theme, normal_theme, error_theme


def _update_gui():
    if _current_rocket_state in config.RECORDING_ENABLED_STATES:
        dpg.set_value("recording_status_text", f"궤적 표시 중 (상태: {_current_rocket_state})")
    else:
        # RECORDING_ENABLED_STATES가 이제 모든 정상 상태를 포함하므로, 이
        # 분기는 flight_state 바이트가 범위를 벗어나 "UNKNOWN"으로 온
        # 경우에만 탄다. CSV 기록 자체는 이 상태와 무관하게 항상 계속된다.
        dpg.set_value(
            "recording_status_text",
            f"궤적 표시 안 함 (상태: {_current_rocket_state}) - CSV 기록은 계속됨",
        )

    dpg.set_value("sik_link_text", _format_link_status("SiK", _last_sik_seen, config.SIK_LINK_TIMEOUT_SEC))
    dpg.set_value("lora_link_text", _format_link_status("LoRa", _last_lora_seen, config.LORA_LINK_TIMEOUT_SEC))

    if _flight_duration_sec is not None:
        # LANDED 시점에 고정된 값 - 착륙 후에는 더 안 흐른다.
        dpg.set_value("flight_timer_text", f"비행시간(T+): {_flight_duration_sec:.1f}s (착륙 시점 고정)")
    elif _ascent_entered_at is not None:
        dpg.set_value("flight_timer_text", f"비행시간(T+): {time.time() - _ascent_entered_at:.1f}s")
    else:
        dpg.set_value("flight_timer_text", "비행시간(T+): --")

    pos_e, pos_n, pos_alt = _fused["position"]
    vel_e, vel_n, vel_z = _fused["velocity"]
    roll, pitch, yaw = _fused["attitude"]
    src = lambda s: s if s is not None else "--"  # noqa: E731 - 짧은 포맷팅 헬퍼

    dpg.set_value(
        "fc_pos_text",
        f"위치(E/N/Alt) [{src(_fused['position_source'])}]: {pos_e:.2f} / {pos_n:.2f} / {pos_alt:.2f} m",
    )
    dpg.set_value(
        "fc_vel_text",
        f"속도(E/N/Z) [{src(_fused['velocity_source'])}]: {vel_e:.2f} / {vel_n:.2f} / {vel_z:.2f} m/s",
    )
    if _fused["accel"] is not None:
        ax, ay, az = _fused["accel"]
        dpg.set_value(
            "fc_accel_text",
            f"가속도(X/Y/Z) [{src(_fused['accel_source'])}]: {ax:.2f} / {ay:.2f} / {az:.2f} m/s^2",
        )
    else:
        dpg.set_value("fc_accel_text", "가속도(X/Y/Z): --")
    dpg.set_value(
        "fused_attitude_text",
        f"자세(R/P/Y) [{src(_fused['attitude_source'])}]: "
        f"{math.degrees(roll):.1f} / {math.degrees(pitch):.1f} / {math.degrees(yaw):.1f} deg",
    )

    dpg.set_value("alt_plot_series", [_alt_plot_t, _alt_plot_alt])
    dpg.fit_axis_data("alt_plot_x_axis")
    dpg.fit_axis_data("alt_plot_y_axis")

    rc8 = _last_rc8 if _last_rc8 is not None else "--"
    rc9 = _last_rc9 if _last_rc9 is not None else "--"
    dpg.set_value("rc_text", f"CH8: {rc8}   CH9: {rc9}")

    # 회수 좌표 + 비행 결과 요약은 평시엔 숨겨두고, LANDED 상태일 때만 같이
    # 표시한다 (착륙 전에는 볼 필요 없는 정보라 화면을 덜 어수선하게 유지).
    is_landed = _current_rocket_state == "LANDED"
    dpg.configure_item("recovery_label", show=is_landed)
    dpg.configure_item("recovery_text", show=is_landed)
    dpg.configure_item("flight_summary_text", show=is_landed)
    if _last_gps is not None:
        dpg.set_value("recovery_text", mission_features.get_recovery_location(_last_gps))
    else:
        dpg.set_value("recovery_text", "GPS 없음")
    if is_landed:
        duration_str = f"{_flight_duration_sec:.1f}s" if _flight_duration_sec is not None else "--"
        dpg.set_value(
            "flight_summary_text",
            f"비행 결과 - 최고고도: {_flight_max_altitude_m:.1f}m   비행시간(ASCENT~LANDED): {duration_str}",
        )

    dpg.set_value("prearm_text", "\n".join(_prearm_reasons) if _prearm_reasons else "이상 없음")


def _update_state_timeline(highlight_theme, normal_theme, error_theme):
    for state in config.ROCKET_STATES:
        theme = highlight_theme if state == _current_rocket_state else normal_theme
        dpg.bind_item_theme(f"state_box_{state}", theme)

    # ERROR는 타임라인의 한 칸이 아니라 fault_flags 기반 별도 표시다
    # (description_flight_state_machine.txt 참고) - Pre-arm 옆의
    # fault_flag_text로 옮겨서 표시한다 (_update_gui 참고).
    if _lora_fault_flags != 0:
        names = [name for bit, name in config.LORA_FAULT_FLAG_NAMES.items() if _lora_fault_flags & bit]
        dpg.set_value("fault_flag_text", f"이상 플래그 ({len(names)}개): " + ", ".join(names))
        dpg.bind_item_theme("fault_flag_text", error_theme)
    else:
        dpg.set_value("fault_flag_text", "이상 플래그: 없음")
        dpg.bind_item_theme("fault_flag_text", normal_theme)


def _update_force_transition_buttons():
    """
    FORCE_* 버튼의 enabled 여부를 매 프레임 갱신한다 - 잠금 해제 체크박스,
    comet_flight_state.lua의 is_command_allowed()와 동일한 "직전 상태에서만
    허용" 규칙, FORCE_DEPLOY 전용 안전 타이머(config.
    FORCE_DEPLOY_SAFETY_DELAY_SEC) 세 가지를 모두 통과해야 눌린다.
    """
    now = time.time()

    for command_name, required_state, _label in FORCE_TRANSITIONS:
        enabled = _force_transition_armed and _current_rocket_state == required_state

        if command_name == "FORCE_DEPLOY" and enabled:
            elapsed = (now - _ascent_entered_at) if _ascent_entered_at is not None else 0.0
            enabled = elapsed >= config.FORCE_DEPLOY_SAFETY_DELAY_SEC

        dpg.configure_item(f"force_transition_btn_{command_name}", enabled=enabled)

    if _current_rocket_state == "ASCENT" and _ascent_entered_at is not None:
        remaining = config.FORCE_DEPLOY_SAFETY_DELAY_SEC - (now - _ascent_entered_at)
        if remaining > 0:
            dpg.set_value("force_deploy_timer_text", f"FORCE_DEPLOY 남은 대기시간: {remaining:.1f}초")
        else:
            dpg.set_value("force_deploy_timer_text", "FORCE_DEPLOY 대기시간 경과 - 사용 가능")
    else:
        dpg.set_value("force_deploy_timer_text", "")


def _run_frame_step(name: str, fn) -> None:
    """
    프레임 함수 하나를 실행하되, 예외가 나도 트레이스백을 출력만 하고 루프
    전체는 멈추지 않는다. 원래는 예외처리가 전혀 없어서 실제 하드웨어
    데이터로 인한 예외 하나가 렌더 루프 전체(그리드까지)를 조용히 멈춰
    세웠는데, 그래서는 콘솔에 아무 단서도 안 남아 원인 파악이 안 됐다.
    """
    try:
        fn()
    except Exception:  # noqa: BLE001 - 진단용, 루프를 지키기 위해 광범위하게 잡음
        import traceback
        print(f"[main] {name} 에서 예외 발생 (이 프레임만 건너뜀):")
        traceback.print_exc()


def main():
    highlight_theme, normal_theme, error_theme = _build_gui()
    _open_fusion_compare_csv()
    _launch_pipeline_threads()

    while dpg.is_dearpygui_running():
        _run_frame_step("_poll_sik", _poll_sik)
        _run_frame_step("_poll_lora", _poll_lora)
        _run_frame_step("_update_fusion", _update_fusion)
        _run_frame_step("_update_reboot_verify", _update_reboot_verify)
        _run_frame_step("_update_gui", _update_gui)
        _run_frame_step("_update_state_timeline", lambda: _update_state_timeline(highlight_theme, normal_theme, error_theme))
        _run_frame_step("_update_force_transition_buttons", _update_force_transition_buttons)
        _run_frame_step("_check_link_transitions", _check_link_transitions)
        _run_frame_step("_update_camera_from_mouse", _update_camera_from_mouse)
        _run_frame_step("_update_3d_scene", _update_3d_scene)
        dpg.render_dearpygui_frame()
        _run_frame_step("_check_landed_snapshot", _check_landed_snapshot)

    if _fusion_compare_csv_file is not None:
        _fusion_compare_csv_file.close()
    dpg.destroy_context()


if __name__ == "__main__":
    main()
