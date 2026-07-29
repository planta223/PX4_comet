-- =====================================================================
-- Configuration
-- =====================================================================

local CONFIG = {}

CONFIG.UPDATE_PERIOD_MS = 50

CONFIG.SAFETY = {
    MAX_TILT_DEG = 70.0, -- 현재 미사용
    SENSOR_TIMEOUT_MS = 1500
}

-- 서보 1개(에비오닉스 서보)만 사용 - 원래 2개였다가 하드웨어가 1개로
-- 줄어들었다. SERVO_CANDIDATE 관련 로직(comet_servo.lua의
-- apply_to_enabled_servos, comet_flight_state.lua의 is_command_allowed)은
-- 이미 이 키가 없으면 자동으로 건너뛰도록 nil 체크가 돼 있어서 여기서
-- 항목만 지우면 됨 - 다른 파일은 손댈 필요 없다.
CONFIG.SERVO = {
    SERVO_MAIN = {
        ENABLED = true,    -- 현재 사용함
        FUNCTION_ID = 94,  -- 확인 완료 (Mission Planner에서 SERVO_FUNCTION 파라미터 설정)
        LOCK_PWM = 1000,   -- 각도 검증 필요
        RELEASE_PWM = 2000 -- 각도 검증 필요
    }
}

CONFIG.LORA = {
    SERIAL_INSTANCE = 0,     -- SERIALx_PROTOCOL이 28인 포트들을 찾는다. (Mission Planner에서 LORA 28, SiK 2 설정)
    BAUD = 9600,             -- DX-LR03 공장 기본값. gcs/config.py의 LORA_BAUDRATE와 반드시 같은 값이어야 한다.

    -- 빠른 전송 주기 (비행 중). 원래 140ms였으나, DX-LR03이 반이중이라 FC가
    -- 이 주기로 계속 텔레메트리를 쏘는 동안 GCS -> FC 명령(FORCE_DEPLOY/
    -- FORCE_LANDED 등)이 들어갈 틈이 거의 없어서 비행 중 원격 명령이
    -- 반복적으로 무응답 처리되는 문제가 실측 로그로 확인됐다(READY/LANDED -
    -- NORMAL_PERIOD_MS 상태에서는 명령이 항상 성공, ASCENT/DEPLOY/DESCENT -
    -- FAST_PERIOD_MS 상태에서는 반복 실패). 텔레메트리 갱신률을 낮추더라도
    -- 비행 중 명령 수신 신뢰성을 우선해 350ms로 늘렸다 - 필요하면 실측
    -- 재확인 후 다시 조정할 것.
    FAST_PERIOD_MS   = 350,
    NORMAL_PERIOD_MS = 1000, -- 일반 전송 주기 (예: 지상에서, 21B/s로 충분)

    TELEMETRY_HEADER  = 0xA5,
    COMMAND_HEADER    = 0x5A,
    ACK_HEADER        = 0xA6,

    TELEMETRY_LENGTH = 17,  -- header(1) + sequence(1) + flight_state(1) + fault_flags(1)
                            -- + time_ms(4)
                            -- + north(2) + east(2) + altitude(4)
                            -- + crc8(1)
                            -- GCS는 위치값만 필요하므로 속도/자세는 보내지 않는다
                            -- (SiK/mavlink ATTITUDE로 대체 수신). fault_flags는
                            -- comet_flight_state.lua의 M.FAULT 비트필드 그대로.
                            -- north/east는 절대 위경도 대신 HOME 기준 오프셋
                            -- (cm 정밀도, int16 -> ±327.67m)으로 보내 바이트를
                            -- 줄인다 - comet_sensor.lua/comet_lora.lua 주석 참고.
    COMMAND_LENGTH    = 4,   -- header(1) + sequence(1) + command(1) + crc8(1)
    ACK_LENGTH        = 4,   -- header(1) + sequence(1) + ack_result(1) + crc8(1)
}

-- SiK(MAVLink)를 통한 원격 명령 입력. LoRa와 동일한 command/ack 코드
-- 체계(comet_lora.lua의 M.COMMAND/M.ACK_RESULT)를 공유한다.
-- GCS -> FC(COMET_CMD)는 NAMED_VALUE_INT, FC -> GCS(COMET_ACK)는
-- NAMED_VALUE_FLOAT(ArduPilot 내장 gcs:send_named_float() 사용 - 이
-- 보드는 mavlink_msgs 모듈을 로드할 메모리 여유가 없어서 인코드는 이
-- 내장 함수로, 디코드는 원시 바이트 수동 파싱으로 대체했다. comet_mavlink.lua
-- 상단 주석 참고). 두 메시지 다 name(char[10]) + value(int32/float) +
-- time_boot_ms만 있어서 sequence/command/result를 value 하나에
-- 3바이트로 패킹한다 (comet_mavlink.lua의 pack_value/unpack_value 참고).
CONFIG.MAVLINK = {
    CMD_NAME = "COMET_CMD",  -- GCS -> 비행체 명령 (NAMED_VALUE_INT.name)
    ACK_NAME = "COMET_ACK",  -- 비행체 -> GCS 응답 (NAMED_VALUE_FLOAT.name)
    POLL_MSG_COUNT = 5        -- 한 루프 틱에서 처리할 최대 MAVLink 메시지 수
}

-- LoRa/SiK 공통 원격 명령 큐. 두 경로 모두 이 큐에 명령을 적재하고,
-- 비행 상태머신(comet_flight_state.lua)이 이 큐를 유일한 명령 소스로 사용한다.
CONFIG.COMMAND_QUEUE = {
    CAPACITY = 12  -- FIFO 링버퍼 크기 (8~16 권장)
}

CONFIG.TRANSITION = {
    READY_TO_ASCENT = {
        -- 정상 전이
        MIN_ACCEL_MPS2 = 25.0,
        ACCEL_CONFIRM_COUNT = 2,

        -- 센서 손실 fallback 전이
        ACCEL_PREVIOUS_STABLE_TIME_MS = 3000,
        ACCEL_LOST_TIME_MS = 500
    },

    ASCENT_TO_DEPLOY = {
        -- 지상 디버깅용
        DEPLOY_ENABLE = true,

        -- 정상 전이
        MIN_TIME_AFTER_LAUNCH_MS = 1000,
        MIN_DESCENT_SPEED_MPS = 1.0,
        DESCENT_SPEED_CONFIRM_COUNT = 3,
        MIN_ALTITUDE_M = 30.0,

        -- 최대시간 fallback 전이
        MAX_TIME_AFTER_LAUNCH_MS = 15000
    },

    DEPLOY_TO_DESCENT = {
        -- 정상 전이
        SERVO_OPERATION_TIME_MS = 1000,
        
        -- 재시도
        RETRY_INTERVAL_MS = 500,
        MAX_RETRY_COUNT = 2
    },

    DESCENT_TO_LANDED = {
        -- 정상 전이
        MAX_ALTITUDE_M = 5.0,
        MAX_ABS_VERTICAL_SPEED_MPS = 1.5,
        MIN_ACCEL_MPS2 = 8.0,
        MAX_ACCEL_MPS2 = 12.0,
        LANDED_CONFIRM_COUNT = 20
    }
}

return CONFIG