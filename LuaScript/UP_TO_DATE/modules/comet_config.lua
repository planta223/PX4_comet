-- =====================================================================
-- Configuration
-- =====================================================================

local CONFIG = {}


CONFIG.UPDATE_PERIOD_MS = 10
CONFIG.TELEMETRY_PERIOD_MS = 200


CONFIG.DEBUG = {
    -- 전체 디버그 기능 활성화 여부
    ENABLE = true,

    -- 비행 상태와 관계없이 LoRa 서보 LOCK/RELEASE 명령 허용
    SERVO_COMMAND_BYPASS_STATE_ENABLE = true,

    -- 센서 조건 및 timeout에 의한 자동 ASCENT → DEPLOY 전이 차단
    DEPLOY_AUTO_TRANSITION_DISABLE = true
}


CONFIG.SERVO = {
    SERVO_MAIN = {
        ENABLED = true,    -- 현재 사용함
        OUTPUT_CHANNEL = 9,-- Mission Planner 채널 9
        LOCK_PWM = 1000,   -- 각도 검증 필요
        RELEASE_PWM = 2000 -- 각도 검증 필요
    },

    SERVO_CANDIDATE = {
        ENABLED = false,   -- 현재 미사용
        OUTPUT_CHANNEL = 10, -- Mission Planner 채널 10
        LOCK_PWM = 1000,   -- 각도 검증 필요
        RELEASE_PWM = 2000 -- 각도 검증 필요
    }
}


CONFIG.LORA = {
    SERIAL_INSTANCE = 0,     -- SERIALx_PROTOCOL이 28인 포트들을 찾는다. (Mission Planner에서 LORA 28, SiK 2 설정)
    BAUD = 57600,

    TELEMETRY_HEADER  = 0xA5,
    COMMAND_HEADER    = 0x5A,

    TELEMETRY_LENGTH = 36,
    -- header(1)
    -- + telemetry_sequence(1)
    -- + flight_state(1)
    -- + status_flags(1)
    -- + fault_flags(1)
    -- + time_ms(4)
    -- + latitude(4)
    -- + longitude(4)
    -- + altitude(4)
    -- + velocity_n(2)
    -- + velocity_e(2)
    -- + velocity_d(2)
    -- + roll(2)
    -- + pitch(2)
    -- + yaw(2)
    -- + ack_sequence(1)
    -- + ack_result(1)
    -- + crc8(1)

    COMMAND_LENGTH    = 4
    -- header(1)
    -- + command_sequence(1)
    -- + command(1)
    -- + crc8(1)
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
        -- 정상 전이
        MIN_TIME_AFTER_LAUNCH_MS = 1000,
        MIN_DESCENT_SPEED_MPS = 1.0,
        DESCENT_SPEED_CONFIRM_COUNT = 3,
        MIN_ALTITUDE_M = 30.0,

        -- 최대시간 fallback 전이
        MAX_TIME_AFTER_LAUNCH_MS = 3000
    },

    DEPLOY_TO_DESCENT = {
        -- 정상 전이
        SERVO_OPERATION_TIME_MS = 1000,
        
        -- 재시도
        RETRY_INTERVAL_MS = 500,
        MAX_RETRY_COUNT = 2 -- 실제 시도 횟수는 MAX_RETRY_COUNT + 1
    },

    DESCENT_TO_LANDED = {
        -- 정상 전이
        MAX_ALTITUDE_M = 5.0,
        MAX_ABS_VERTICAL_SPEED_MPS = 1.5,
        MIN_ACCEL_MPS2 = 8.0,
        MAX_ACCEL_MPS2 = 12.0,
        LANDED_CONFIRM_COUNT = 200
    }
}

return CONFIG
