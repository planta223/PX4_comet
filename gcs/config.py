# -*- coding: utf-8 -*-
"""
전역 설정 상수.
포트, baud rate, 큐 사이즈, 칼만필터 파라미터, LoRa 패킷 스펙, CSV 경로 등
모든 수치/경로 상수는 이 파일에서만 정의하고 다른 모듈은 여기서 import해서 쓴다.
"""

import numpy as np

# ---------------------------------------------------------------------------
# GUI 폰트 (한글 표시용)
# ---------------------------------------------------------------------------
# Dear PyGui 기본 폰트는 한글 글리프가 없어 "???"로 깨진다. 맑은 고딕은
# Windows에 기본 내장된 한글 폰트라 별도 설치 없이 바로 쓸 수 있다.
KOREAN_FONT_PATH = r"C:\Windows\Fonts\malgun.ttf"
KOREAN_FONT_SIZE = 16

# ---------------------------------------------------------------------------
# SiK / pymavlink 연결
# ---------------------------------------------------------------------------
# 실제 라디오 사용 시 COM 포트("COM5" 등)나 mavlink 커넥션 문자열
# ("udp:127.0.0.1:14550" 등)을 넣는다. 로그 리플레이로 검증할 때는
# 여기에 .tlog / .bin 파일 경로를 넣으면 pymavlink가 그대로 재생한다.
SIK_CONNECTION_STRING = "COM11"
SIK_BAUDRATE = 57600

# 연결이 끊기면(FC 리부트, 케이블 순단 등) 이 간격(초)마다 재연결을 시도한다.
SIK_RECONNECT_DELAY_SEC = 2.0
# 최초 연결/재연결 시 heartbeat를 이 시간(초) 안에 못 받으면 실패로 보고 재시도한다.
SIK_HEARTBEAT_TIMEOUT_SEC = 10.0

# GCS에서 "FC Reboot" 버튼을 누르면 리부트 명령 전송 후 이 시간(초) 안에
# SiK/LoRa 링크 재연결 + RAW_IMU/ATTITUDE 재수신을 자동 검증한다. 이 시간
# 안에 다 통과 못 하면 타임아웃으로 실패 처리하고 운영자가 직접 확인하게
# 한다 - GPS 3D fix처럼 실내에서 수십 초 걸릴 수 있는 항목은 이 게이트에서
# 일부러 제외했다(무기한 대기 방지).
REBOOT_VERIFY_TIMEOUT_SEC = 15.0

# 이 시간(초) 동안 RAW_IMU가 한 통도 안 오면 sik.py가 그 하나만 다시
# 요청한다(ACK 대기 없이 논블로킹으로 - 자세한 이유는 sik.py의 워치독
# 주석 참고). RAW_IMU 정상 주기(SIK_IMU_RATE_HZ)보다 충분히 길게 잡아서
# 일시적 지터를 오탐하지 않게 한다.
SIK_RAW_IMU_WATCHDOG_SEC = 3.0

# COMET_ACK(NAMED_VALUE_FLOAT) 수신 주기 - 서보 잠금/사출 등 원격 명령의
# 실행 결과 ACK를 comet_mavlink.lua로부터 받기 위한 것. 명령 자체가
# 이벤트성이라 높은 주기가 필요 없다.
#
# comet_mavlink.lua는 ACK를 NAMED_VALUE_INT가 아니라 NAMED_VALUE_FLOAT으로
# 보낸다 - ArduPilot 내장 gcs:send_named_float()를 그대로 쓰기 위함(모듈
# 의존 없음, SCR_HEAP_SIZE 부족 문제 회피). COMET_CMD(GCS->FC)는 여전히
# NAMED_VALUE_INT.
MAVLINK_COMET_ACK_RATE_HZ = 5

# 필요한 필드만 선별 수신하기 위한 메시지 요청 주기 (Hz)
# LOCAL_POSITION_NED/SCALED_IMU(EKF3 비교용, 아래 참고)를 끄면서 확보한
# 대역폭 여유(10Hz)를 그대로 RAW_IMU에 투입했다 - 원래 20Hz였음.
# (총 메시지/초는 기존 58/s과 동일하게 유지됨: 30+5+2+1+10+10=58)
SIK_IMU_RATE_HZ = 30
SIK_GPS_RATE_HZ = 5
SIK_RC_CHANNELS_RATE_HZ = 2
SIK_SYS_STATUS_RATE_HZ = 1

# FC 온보드 EKF3가 직접 계산한 상태값(위치/속도/가속도) 표시용.
# 자체 칼만필터가 같은 값(위치/속도)을 이미 계산하고 있어 순수 비교/디버그
# 용도라 SIK_REQUESTED_MESSAGES에서 뺐다 (대역폭을 SCALED_PRESSURE에 양보).
# 다시 켜고 싶으면 아래 두 줄을 SIK_REQUESTED_MESSAGES에 추가하면 됨
# (sik.py의 LOCAL_POSITION_NED/SCALED_IMU 핸들러는 그대로 남아있음).
SIK_EKF_POS_RATE_HZ = 10
SIK_EKF_ACCEL_RATE_HZ = 10

# 자세(ATTITUDE, roll/pitch/yaw) 수신 주기 - 3D 뷰의 로켓 모형 회전에 사용
SIK_ATTITUDE_RATE_HZ = 10

# 기압계(SCALED_PRESSURE) 수신 주기 - 자체 칼만필터의 고도(수직) 관측에 사용.
# GPS(5Hz)보다 훨씬 빠르지만, predict()가 이미 IMU(20Hz)로 갱신 공백을
# 적분해서 메꿔주므로 그 이상 올려봐야 이득이 작다 - 10Hz면 충분.
SIK_BARO_RATE_HZ = 10

# 실제로 SiK에 요청할 mavlink 메시지와 그 주기(Hz) 목록.
# sik.py는 연결 직후 FC의 기본 스트림을 전부 끄고(REQUEST_DATA_STREAM rate=0),
# 여기 나열된 메시지만 MAV_CMD_SET_MESSAGE_INTERVAL로 개별 요청한다.
# 즉 좁은 SiK 대역폭을 이 목록에 있는 것에만 쓰게 된다 - 항목을 빼거나
# 주기를 올리면(코드 수정 없이) 남은 메시지의 실질 갱신율을 더 높일 수 있다.
SIK_REQUESTED_MESSAGES = [
    ("RAW_IMU", SIK_IMU_RATE_HZ),
    ("GPS_RAW_INT", SIK_GPS_RATE_HZ),
    ("RC_CHANNELS", SIK_RC_CHANNELS_RATE_HZ),
    ("SYS_STATUS", SIK_SYS_STATUS_RATE_HZ),
    ("ATTITUDE", SIK_ATTITUDE_RATE_HZ),
    ("SCALED_PRESSURE", SIK_BARO_RATE_HZ),
    # NAMED_VALUE_FLOAT(COMET_ACK)은 여기 넣지 않는다 - comet_mavlink.lua가
    # gcs:send_named_float()로 그때그때 쏘는 이벤트성 메시지라 FC의 주기적
    # 스트림 슬롯(ap_message)에 대응하는 게 없다. SET_MESSAGE_INTERVAL로
    # 요청해봐야 "No ap_message for mavlink id (251)"만 뜨고 무시되며,
    # 요청 없이도 sik.py는 NAMED_VALUE_FLOAT을 그냥 수신해서 처리한다.
]

# heartbeat 미수신시 링크 dead 판정 (초)
SIK_LINK_TIMEOUT_SEC = 3.0
LORA_LINK_TIMEOUT_SEC = 3.0

# ---------------------------------------------------------------------------
# 큐
# ---------------------------------------------------------------------------
TRAJECTORY_QUEUE_MAXSIZE = 1000
LORA_QUEUE_MAXSIZE = 1000

# ---------------------------------------------------------------------------
# 칼만필터 (6D, SiK 전용) - 상태: [pos_e, pos_n, alt, vel_e, vel_n, vel_z]
# ---------------------------------------------------------------------------
KF_INITIAL_COVARIANCE = np.diag([
    5.0, 5.0, 5.0,      # pos_e, pos_n, alt (m^2)
    2.0, 2.0, 2.0,      # vel_e, vel_n, vel_z (m^2/s^2)
]).astype(float)

# 프로세스 노이즈 Q (가속도 입력 불확실성 기반)
KF_PROCESS_NOISE = np.diag([
    0.05, 0.05, 0.05,   # position
    0.5, 0.5, 0.5,      # velocity
]).astype(float)

# 관측 노이즈 R - GPS(수평)와 기압계(수직)를 독립된 관측으로 분리해서
# 각자 도착하는 대로(GPS 5Hz, 기압계 SIK_BARO_RATE_HZ) 갱신한다.
# 기압계가 GPS 고도보다 훨씬 정확/고속이라 R_vertical을 더 작게 잡는다.
#
# 아래 값은 실제 장착 센서(Pixhawk 4 내장 MS5611 기압계, u-blox M10 GPS)의
# 데이터시트 스펙에서 역산했다:
#   - GPS(M10): CEP50=1.5m (open sky, standalone) -> sigma = CEP/1.1774 ≈ 1.27m
#     -> R_horizontal 대각원소 = sigma^2 ≈ 1.6
#   - 기압계(MS5611): OSR4096 RMS noise=0.012mbar -> 고도환산식
#     44330*(1-(P/P0)^(1/5.255))을 P=P0에서 미분한 기울기 d(alt)/dP
#     ≈ -44330/(5.255*P0) ≈ -8.3 m/mbar(P0≈1013mbar 기준)
#     -> sigma_alt ≈ 8.3 * 0.012 ≈ 0.10m -> R_vertical ≈ sigma_alt^2 ≈ 0.01
#   두 값 다 "이상적 정적 조건" 스펙이라 실비행에서는 진동/멀티패스/바람이
#   정압구에 미치는 영향 등으로 실측이 더 나쁘게 나올 수 있다 - 실측 로그로
#   검증되면 이 값들을 다시 올려야 할 수 있음.
KF_OBSERVATION_NOISE_HORIZONTAL = np.diag([
    1.6, 1.6,
]).astype(float)
KF_OBSERVATION_NOISE_VERTICAL = np.diag([
    0.01,
]).astype(float)

# ---------------------------------------------------------------------------
# CSV 저장
# ---------------------------------------------------------------------------
CSV_OUTPUT_DIR = "./logs"
SIK_CSV_FILENAME_PATTERN = "sik_%Y%m%d.csv"
LORA_CSV_FILENAME_PATTERN = "lora_%Y%m%d.csv"
SIK_STATS_CSV_FILENAME_PATTERN = "sik_stats_%Y%m%d.csv"

# CSV flush 최소 간격(초) - 매 샘플(RAW_IMU 30Hz/LoRa 텔레메트리)마다 flush()를
# 부르면 그 순간 디스크/동기화 I/O가 수신 루프를 막아 뒤이어 도착하는
# 시리얼 바이트가 OS 버퍼에서 밀려 유실될 수 있다 (특히 CSV_OUTPUT_DIR이
# OneDrive 등 동기화 폴더 아래 있으면 그 필터 드라이버가 flush를 가로채
# 지연시킬 수 있음). 이 간격 이하로는 flush를 건너뛰고 다음 루프 반복으로
# 넘어가 수신을 우선한다 - 대신 크래시 시 최대 이 간격만큼의 최근 CSV
# 데이터가 디스크에 못 갈 수 있다 (실시간 GUI 표시는 큐 경유라 영향 없음).
CSV_FLUSH_INTERVAL_SEC = 0.5
LORA_STATS_CSV_FILENAME_PATTERN = "lora_stats_%Y%m%d.csv"
FUSION_COMPARE_STATS_CSV_FILENAME_PATTERN = "fusion_compare_stats_%Y%m%d.csv"

# ---------------------------------------------------------------------------
# 패킷 품질 / 위치·속도 안정성 정량 통계 (stats.py) - 터미널 로그 + CSV
# ---------------------------------------------------------------------------
STATS_REPORT_INTERVAL_SEC = 5.0   # 통계 콘솔 출력 + CSV 기록 주기
STATS_RATE_WINDOW = 50            # RateStats(갱신률/지터) 이동창 샘플 수
STATS_ROLLING_WINDOW = 100        # Vec3RollingStats(위치/속도 표준편차) 이동창 샘플 수

# LANDED 상태 감지 시 화면(궤적 포함)을 자동으로 이미지로 저장 - 저장 위치는
# main.py의 flight_log_dir(비행 폴더)를 그대로 따라간다(snapshots/ 하위폴더).
SNAPSHOT_FILENAME_PATTERN = "landed_%Y%m%d_%H%M%S.png"

# 여기 나열된 상태에서만 trajectory_queue push(3D 뷰/거리 플롯 표시)를 한다
# (칼만필터 자체는 상태와 무관하게 계속 돌아간다 - 발사 순간 바로 수렴된
# 추정치를 쓸 수 있도록). CSV 기록은 이 목록과 무관하게 sik.py/lora.py가
# 항상 무조건 남긴다 - 여기서 거르는 건 화면 표시(궤적 트레일)뿐이다.
# READY 포함 - 발사 전 지상 대기/테스트 궤적도 화면에 보이는 게 디버깅에
# 더 유용하다는 판단으로 포함시켰다. ERROR는 더 이상 별도 상태가 아니라
# fault_flags이므로(LORA_FAULT_FLAG_NAMES 참고) 여기 따로 넣지 않아도 된다.
RECORDING_ENABLED_STATES = ["READY", "ASCENT", "DEPLOY", "DESCENT", "LANDED"]

# STATUSTEXT에 아래 키워드 중 하나라도 포함되면(대소문자 무시, 부분 문자열
# 매칭) 해당 로켓 상태로 전환한다. 실제 FC/Lua 스크립트가 보내는 문구가
# 상태 이름 그대로("ASCENT")가 아닐 수 있어("Liftoff detected" 등) 상태
# 이름과 실제 문구를 분리했다. FC가 보내는 문구에 맞춰 이 목록만 고치면
# 되고 sik.py 코드는 손댈 필요 없다.
#
# LoRa 텔레메트리에 flight_state 필드가 이미 있어서(그리고 fusion.py가 LoRa를
# 우선시함) 이 STATUSTEXT 기반 추정은 LoRa가 죽었을 때만 쓰이는 폴백이다
# (SiK/mavlink는 LoRa와 물리적으로 다른 라디오라 LoRa만 끊겨도 살아있을 수
# 있음). 그래도 완전히 지우진 않고 유지 - 대신 값 순서에 의존하는 버그를
# 고쳤다: comet_flight_state.lua의 전이 로그가 "OLD -> NEW" 형식이라
# ("COMET: ASCENT -> DEPLOY" 등) 예전 순서(ASCENT를 먼저 검사)로는 새 상태
# 대신 옛 상태 이름이 먼저 매칭됐다. 비행 후반 상태부터 먼저 검사하도록
# 순서를 뒤집어서 항상 "->" 뒤의(새) 상태가 이기게 했다. ERROR는 더 이상
# 로켓 상태가 아니므로(RECORDING_ENABLED_STATES 주석 참고) 여기서 뺐다.
ROCKET_STATE_KEYWORDS = {
    "LANDED": ["LANDED", "LANDING DETECTED"],
    "DESCENT": ["DESCENT", "CHUTE", "PARACHUTE"],
    "DEPLOY": ["DEPLOY"],
    "ASCENT": ["LIFTOFF DETECTED", "ASCENT"],
    "READY": ["READY"],
}

# ---------------------------------------------------------------------------
# LoRa 패킷 스펙 - comet_main.lua(FC측 실제 구현)와 반드시 일치해야 한다.
# CRC는 poly=0x07, init=0x00 (반사/최종XOR 없음, 표준 "CRC-8") - "ATM"이 아님.
# ---------------------------------------------------------------------------
LORA_TELEMETRY_HEADER = 0xA5
LORA_COMMAND_HEADER = 0x5A
LORA_ACK_HEADER = 0xA6

LORA_TELEMETRY_LENGTH = 17
# comet_lora.lua의 send_telemetry()와 정확히 일치하는 바이트 레이아웃
# (little-endian, struct 포맷 "<4BI2hiB"):
#   header(1) + seq(1) + flight_state(1) + fault_flags(1)
#   + time_ms(4, uint32, millis())
#   + north, east(2x2, int16, HOME 기준 오프셋(m) x 100 -> cm 정밀도,
#     범위 ±327.67m - append_i16()이 clamp하므로 그 이상은 saturate됨.
#     절대 위경도가 아니라 HOME 기준 상대좌표라 GCS에서 origin 변환 없이
#     그대로 pos_n/pos_e(m)로 쓴다, lora.py 참고)
#   + altitude(4, int32, 실제값(m, HOME 기준 상대고도) x 100)
#   + crc8(1)
# GCS는 LoRa로는 위치값만 받는다 - 속도/자세는 SiK/mavlink(ATTITUDE 등)로만
# 받으며 폴백이 없다 (fusion.py 모듈 docstring 참고).
# fault_flags는 comet_flight_state.lua의 M.FAULT 비트필드 그대로 - 자세한
# 비트 의미는 아래 LORA_FAULT_FLAG_NAMES 참고.
LORA_COMMAND_LENGTH = 4      # header + sequence + command + crc8
LORA_ACK_LENGTH = 4          # header + sequence + result + crc8

LORA_CRC_POLYNOMIAL = 0x07   # CRC-8, init=0x00 (calculate_crc8()에 하드코딩되어 있음)

# comet_lora.lua의 lora.COMMAND 테이블과 반드시 값이 일치해야 한다.
# FORCE_*는 description_flight_state_machine.txt에 문서화된 "원격 강제전이"
# 명령 - 각각 정상 전이 그래프의 바로 이전 상태에서만 FC가 허용한다
# (is_command_allowed() 참고). GCS 버튼은 main.py의 FORCE_TRANSITIONS 참고.
LORA_COMMAND = {
    "SERVO1_LOCK": 1,
    "SERVO1_RELEASE": 2,
    "SERVO2_LOCK": 3,
    "SERVO2_RELEASE": 4,
    "LOCK_ALL": 5,
    "RELEASE_ALL": 6,
    "FORCE_ASCENT": 7,
    "FORCE_DEPLOY": 8,
    "FORCE_DESCENT": 9,
    "FORCE_LANDED": 10,
    "RESET": 11,
}

# FORCE_DEPLOY(낙하산 즉시 사출)만 안전 타이머를 건다 - ASCENT 진입 직후
# 실수로 눌러도 초저속/초저고도에서 바로 사출되지 않도록 하는 GCS측 UI
# 가드다. comet_config.lua의 ASCENT_TO_DEPLOY.MIN_TIME_AFTER_LAUNCH_MS
# (1000ms, 정상 DEPLOY 전이도 이 시간은 지나야 함)보다 여유를 더 둔 값 -
# FC 자체는 이 타이머를 모른다(FC는 ASCENT 상태이기만 하면 바로 허용,
# is_command_allowed() 참고) - 순전히 GCS 조작 실수 방지용이다.
FORCE_DEPLOY_SAFETY_DELAY_SEC = 5.0

# comet_main.lua의 lora.ACK_RESULT 테이블과 반드시 값이 일치해야 한다.
LORA_ACK_RESULT_NAMES = {
    0: "OK",
    1: "INVALID_COMMAND",
    2: "NOT_ALLOWED",
    3: "EXECUTION_FAILED",
    4: "RECEIVED",  # CRC/패킷 검증 통과, 아직 상태머신이 처리하기 전
}

# comet_lora.lua는 명령 하나당 ACK를 두 번 보낸다: CRC 검증 직후 즉시
# RECEIVED, 그리고 상태머신이 큐에서 꺼내 실제 처리한 뒤 최종 결과
# (OK/NOT_ALLOWED/EXECUTION_FAILED). lora.py의 _send_command_and_wait_ack는
# 같은 sequence로 두 번 다 받게 되므로, RECEIVED는 최종 응답이 아니라고
# 구분해서 계속 대기해야 한다 (안 그러면 RECEIVED만 보고 "성공"으로 착각해
# 다음 명령을 너무 일찍 보내고, 그 사이 FC가 마저 보내는 두 번째(진짜) ACK와
# 반이중 라디오에서 충돌해 뒤이은 명령이 유실될 수 있다).
LORA_ACK_RESULT_RECEIVED = 4

# ---------------------------------------------------------------------------
# SiK(MAVLink) 원격 명령 - LoRa와 동일한 comet_command_queue.lua 큐를 공유한다
# (comet_mavlink.lua 참고). 명령 코드 자체는 LORA_COMMAND와 완전히 동일한
# 값을 그대로 쓴다 - LoRa로 보내든 SiK로 보내든 FC 쪽 상태 검증/서보 제어/
# ACK 로직이 똑같이 동작해야 하기 때문이다.
#
# NAMED_VALUE_INT는 name(char[10]) + value(int32) + time_boot_ms만 있어서
# sequence(0~255)/command(0~255)/aux(0~255, 결과 코드 또는 미사용)를
# value 하나(3바이트)에 패킹한다 - comet_mavlink.lua의 pack_value()/
# unpack_value()와 반드시 동일한 비트 배치를 써야 한다.
# ---------------------------------------------------------------------------
MAVLINK_COMET_CMD_NAME = "COMET_CMD"
MAVLINK_COMET_ACK_NAME = "COMET_ACK"


def pack_comet_value(sequence: int, command: int, aux: int = 0) -> int:
    return ((sequence & 0xFF) << 16) | ((command & 0xFF) << 8) | (aux & 0xFF)


def unpack_comet_value(value: int):
    value = int(value)
    sequence = (value >> 16) & 0xFF
    command = (value >> 8) & 0xFF
    aux = value & 0xFF
    return sequence, command, aux

# comet_flight_state.lua의 M.FAULT 비트필드와 반드시 일치해야 한다. ERROR는
# 더 이상 별도 로켓 상태가 아니라(description_flight_state_machine.txt 참고)
# 이 비트필드로만 표현한다 - 텔레메트리의 fault_flags 바이트를 이 표로 풀어서
# "왜" 에러인지 이름을 보여준다. DEPLOY_COMMAND_FAILED는 낙하산 사출 명령을
# 보냈는데 서보 출력이 실패한 경우 디버깅용으로 켜진다.
LORA_FAULT_FLAG_NAMES = {
    0x01: "ACCEL_SENSOR_INVALID",
    0x02: "POSITION_INVALID",
    0x04: "VELOCITY_INVALID",
    0x08: "ASCENT_TIMEOUT",
    0x10: "DEPLOY_COMMAND_FAILED",
    0x20: "LORA_UNAVAILABLE",
}

LORA_FLIGHT_STATE_NAMES = ["READY", "ASCENT", "DEPLOY", "DESCENT", "LANDED"]

# COM8 = USB-SERIAL CH340 (장치관리자로 확인됨, SIK_CONNECTION_STRING의
# COM6과는 별개의 실제 물리 장치 - DX-LR03 LoRa 모듈의 지상측 USB-TTL 어댑터로 추정).
LORA_SERIAL_PORT = "COM18"

# DX-LR03 모듈 공장 기본값(9600/8N1). comet_config.lua의 CONFIG.LORA.BAUD와
# 반드시 같은 값이어야 한다 (예전에 57600으로 바꾸는 걸 검토하다가 lua 쪽만
# 바뀌고 여기가 안 맞았던 적이 있음 - 값 바꿀 땐 양쪽 다 확인할 것).
LORA_BAUDRATE = 9600

# LoRa 명령에 sequence(0~255, wraparound)를 붙여 재전송을 구분한다.
LORA_ACK_TIMEOUT_SEC = 2.0
LORA_COMMAND_RETRY_COUNT = 3

# 연결이 끊기면(라디오 전원, 케이블 등) 이 간격(초)마다 재연결을 시도한다.
# SIK_RECONNECT_DELAY_SEC와 동일한 패턴.
LORA_RECONNECT_DELAY_SEC = 2.0

# ---------------------------------------------------------------------------
# 로켓 상태 머신
# ---------------------------------------------------------------------------
# ERROR는 별도 상태가 아니라 fault_flags로 관리한다
# (description_flight_state_machine.txt, LORA_FAULT_FLAG_NAMES 참고) - 화면
# 강조는 상태 타임라인이 아니라 별도 fault 표시(main.py)가 담당한다.
ROCKET_STATES = ["READY", "ASCENT", "DEPLOY", "DESCENT", "LANDED"]

# ---------------------------------------------------------------------------
# 3D 궤적 뷰 (카메라 / 지면 그리드 / 로켓 와이어프레임 모델)
# ---------------------------------------------------------------------------
SCENE3D_FOV_DEG = 45.0
SCENE3D_NEAR = 0.5
SCENE3D_FAR = 5000.0

SCENE3D_INITIAL_DISTANCE_M = 50.0
SCENE3D_MIN_DISTANCE_M = 5.0
SCENE3D_MAX_DISTANCE_M = 2000.0
SCENE3D_INITIAL_AZIMUTH_DEG = 45.0
SCENE3D_INITIAL_ELEVATION_DEG = 25.0
SCENE3D_ORBIT_SPEED_DEG_PER_PX = 0.3   # 좌클릭 드래그 1px당 회전각
SCENE3D_ZOOM_STEP_RATIO = 0.1          # 휠 한 틱당 거리 변화 비율

SCENE3D_GRID_HALF_EXTENT_M = 100.0     # 지면 그리드 한 변의 절반 길이
SCENE3D_GRID_STEP_M = 10.0             # 그리드 칸 간격

# 로켓 와이어프레임 모델 크기 (실제 로켓과 다를 수 있는 표시용 단순화 모델)
ROCKET_MODEL_LENGTH_M = 2.0
ROCKET_MODEL_DIAMETER_M = 0.3
ROCKET_MODEL_FIN_SPAN_M = 0.6
ROCKET_MODEL_NOSE_RATIO = 0.25

# ---------------------------------------------------------------------------
# 메시지 로그 콘솔 (Mission Planner의 메시지창과 동일 - STATUSTEXT 표시)
# ---------------------------------------------------------------------------
MESSAGE_LOG_MAX_LINES = 200     # 화면에 유지할 최대 줄 수 (넘으면 오래된 것부터 삭제)
MESSAGE_LOG_HEIGHT_PX = 150

# ---------------------------------------------------------------------------
# 서보 제어 콘솔 - 여기 목록만 수정하면 GUI에 슬라이더/버튼이 그만큼 생긴다.
# channel: MAV_CMD_DO_SET_SERVO의 서보 번호(=출력 채널 번호)
# ---------------------------------------------------------------------------
SERVO_CHANNELS = [
    {"channel": 9, "label": "낙하산 서보", "pwm_min": 1000, "pwm_max": 2000, "pwm_default": 1500},
]
