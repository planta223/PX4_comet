# -*- coding: utf-8 -*-
"""
회수 좌표/지도 링크, pre-arm 상태 파싱, FC 리부트, READY 상태 리셋.

Waypoint 기능과 Param 읽기/쓰기는 이번 범위 제외.
"""

from pymavlink import mavutil

import config
import lora

# GCS -> FC로 보내는 COMET_CMD(NAMED_VALUE_INT) sequence 카운터.
# LoRa 쪽(lora.py)과 별개의 무선 경로라 독립적으로 0~255를 순환시킨다.
_mavlink_cmd_sequence = 0

_GOOGLE_MAPS_URL_FORMAT = "https://www.google.com/maps?q={lat},{lon}"


def get_recovery_location(last_gps: dict) -> str:
    """
    last_gps: {"lat": float, "lon": float, ...} 형태의 마지막 GPS 좌표.
    반환: 구글맵 URL 문자열.
    """
    lat = last_gps["lat"]
    lon = last_gps["lon"]
    return _GOOGLE_MAPS_URL_FORMAT.format(lat=lat, lon=lon)


def check_prearm_status(mavlink_msg) -> list:
    """
    mavlink_msg: STATUSTEXT mavlink 메시지 객체.
    ArduPilot의 pre-arm 실패 메시지는 "PreArm: <reason>" 형식으로 전송되며,
    한 메시지에 여러 사유가 ';'로 구분되어 올 수 있다.

    반환: pre-arm 실패 사유 문자열 리스트. PreArm 메시지가 아니면 빈 리스트.
    """
    text = getattr(mavlink_msg, "text", "")
    if not text.strip().upper().startswith("PREARM"):
        return []

    _, _, remainder = text.partition(":")
    reasons = [r.strip() for r in remainder.split(";") if r.strip()]
    return reasons


def reboot_fc(mavlink_connection) -> None:
    """
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 명령을 SiK/pymavlink 경유로 전송해
    FC를 리부트한다.
    """
    mavlink_connection.mav.command_long_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )


def send_state_reset(lora_command_queue) -> None:
    """
    로켓 상태를 READY로 리셋하는 RESET 커맨드를 LoRa 커맨드 큐에 넣는다.

    실제 시리얼 송신/ACK 대기는 lora.py의 run_lora_pipeline 스레드가
    이 큐를 드레인하면서 전담한다 - GUI 스레드가 시리얼 포트에 직접
    쓰면 수신 스레드와 레이스 컨디션이 생기기 때문이다.
    """
    lora.send_state_reset_packet(lora_command_queue)


def send_lora_command(lora_command_queue, command_name: str) -> None:
    """
    config.LORA_COMMAND에 정의된 커맨드(예: "RELEASE_ALL", "LOCK_ALL")를
    LoRa 커맨드 큐에 넣는다. send_state_reset과 동일한 경로 - 실제 송신/ACK
    대기는 lora.py의 run_lora_pipeline 스레드가 전담한다.
    """
    lora.enqueue_command(lora_command_queue, command_name)


def send_mavlink_command(mavlink_connection, command_code: int) -> int:
    """
    config.LORA_COMMAND에 정의된 것과 동일한 command_code(예: RELEASE_ALL=6,
    LOCK_ALL=5)를 SiK 경유 NAMED_VALUE_INT(COMET_CMD)로 전송한다.

    LoRa와 동일한 명령 코드 체계를 그대로 쓰기 때문에, FC(comet_flight_state.lua)
    쪽에서는 어느 무선 경로로 왔는지와 무관하게 완전히 동일한 comet_command_queue
    큐/상태검증/서보제어/ACK 로직을 거친다 (comet_mavlink.lua 참고).

    반환값: 이번에 사용한 sequence (호출 쪽이 ACK 매칭/로깅에 쓸 수 있게).
    """
    global _mavlink_cmd_sequence
    sequence = _mavlink_cmd_sequence
    _mavlink_cmd_sequence = (_mavlink_cmd_sequence + 1) % 256

    value = config.pack_comet_value(sequence, command_code)

    mavlink_connection.mav.named_value_int_send(
        0,  # time_boot_ms - FC 값과 무관, 정보용이라 0으로 보내도 무방
        config.MAVLINK_COMET_CMD_NAME.encode("ascii"),
        value,
    )

    return sequence


def set_servo(mavlink_connection, channel: int, pwm_us: int) -> None:
    """
    MAV_CMD_DO_SET_SERVO 명령을 SiK/pymavlink 경유로 전송해 지정한 서보
    채널을 주어진 PWM(us) 값으로 움직인다.

    channel: 서보 번호(=출력 채널 번호). 실제로 어떤 채널을 쓸지는
    config.SERVO_CHANNELS에서 정의한다 (이 함수는 채널 번호를 그대로 받아
    전달만 함).
    """
    mavlink_connection.mav.command_long_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel, pwm_us, 0, 0, 0, 0, 0,
    )
