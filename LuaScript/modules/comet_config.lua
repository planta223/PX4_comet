-- =====================================================================
-- Configuration
-- =====================================================================

local CONFIG = {}

CONFIG.UPDATE_PERIOD_MS = 50

CONFIG.SAFETY = {
    MAX_TILT_DEG = 70.0,
    SENSOR_TIMEOUT_MS = 1500
}

CONFIG.SERVO = {
    SERVO1 = {
        FUNCTION_ID = 94,  -- 확인 필요 (Mission Planner에서 확인)
        LOCK_PWM = 1000,   -- 각도 검증 필요
        RELEASE_PWM = 2000 -- 각도 검증 필요
    },

    SERVO2 = {
        FUNCTION_ID = 95,  -- 확인 필요 (Mission Planner에서 확인)
        LOCK_PWM = 1000,   -- 각도 검증 필요
        RELEASE_PWM = 2000 -- 각도 검증 필요
    }
}

CONFIG.LORA = {
    SERIAL_INSTANCE = 0,     -- SERIALx_PROTOCOL이 28인 포트들을 찾는다. (Mission Planner에서 LORA 28, SiK 2 설정)
    BAUD = 57600,

    FAST_PERIOD_MS   = 250,  -- 빠른 전송 주기 (예: 비행 중)
    NORMAL_PERIOD_MS = 1000, -- 일반 전송 주기 (예: 지상에서)

    TELEMETRY_HEADER  = 0xA5,
    COMMAND_HEADER    = 0x5A,
    ACK_HEADER        = 0xA6,

    TELEMETRY_LENGTH = 33,  -- header(1) + sequence(1) + flight_state(1) + status_flags(1)
                            -- + time_ms(4)
                            -- + latitude(4) + longitude(4) + altitude(4)
                            -- + velocity_n(2) + velocity_e(2) + velocity_d(2)
                            -- + roll(2) + pitch(2) + yaw(2)
                            -- + crc8(1)
    COMMAND_LENGTH    = 4,   -- header(1) + sequence(1) + command(1) + crc8(1)
    ACK_LENGTH        = 4,   -- header(1) + sequence(1) + ack_result(1) + crc8(1)
}

CONFIG.TRANSITION = {
    READY_TO_ASCENT = {
        ACCEL_THRESHOLD_MPS2 = 25.0,
        CONFIRM_COUNT = 3
    },

    ASCENT_TO_APOGEE = {
        MIN_TIME_AFTER_LAUNCH_MS = 1000,
        MIN_DESCENT_SPEED_MPS = 1.0,
        ALTITUDE_DROP_MARGIN_M = 2.0,
        CONFIRM_COUNT = 3,
        MAX_ASCENT_TIME_MS = 15000
    },

    APOGEE_TO_DESCENT = {
        MIN_HOLD_TIME_MS = 1000
    },

    DESCENT_TO_LANDED = {
        MAX_RELATIVE_ALTITUDE_M = 3.0,
        MAX_ABS_VERTICAL_SPEED_MPS = 1.5,
        ACCEL_MIN_MPS2 = 8.0,
        ACCEL_MAX_MPS2 = 12.0,
        CONFIRM_COUNT = 20
    }
}

return CONFIG