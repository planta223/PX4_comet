-- =====================================================================
-- Configuration
-- =====================================================================

local CONFIG = {}

CONFIG.UPDATE_PERIOD_MS = 20

CONFIG.SAFETY = {
    MAX_TILT_DEG = 70.0, -- 현재 미사용
    SENSOR_TIMEOUT_MS = 1500
}

-- 이건 추후 삭제
CONFIG.DEBUG = {
    -- true:
    -- 비행 상태와 관계없이 LoRa 서보 LOCK/RELEASE 명령 허용
    --
    -- false:
    -- 정상 비행 상태 제한 적용
    SERVO_COMMAND_BYPASS_STATE = true
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
    NORMAL_PERIOD_MS = 1000, -- 일반 전송 주기 (예: 지상에서)

    TELEMETRY_HEADER  = 0xA5,
    COMMAND_HEADER    = 0x5A,
    ACK_HEADER        = 0xA6,

    TELEMETRY_LENGTH = 10,  -- header(1) + sequence(1) + state_fault(1)
                            -- + north(2) + east(2) + altitude(2)
                            -- + crc8(1)
                            -- (time_ms는 GCS가 안 써서 뺐다 - 아래 참고)
                            -- gcs/config.py의 LORA_TELEMETRY_LENGTH와 정확히
                            -- 일치해야 한다. north/east는 절대 위경도 대신
                            -- HOME 기준 오프셋을 dm 정밀도(int16, ×10 ->
                            -- ±3276.7m)로, altitude는 같은 오프셋을 cm
                            -- 정밀도 unsigned(×100, 0~655.35m, 음수 없음)로
                            -- 보내 바이트를 줄인다 - comet_sensor.lua/
                            -- comet_lora.lua 주석 참고.
                            --
                            -- state_fault(1바이트) = flight_state(상위 3비트,
                            -- 0~4 -> 0~7 여유) | fault_flags(하위 5비트,
                            -- 0x01~0x10만 - LORA_UNAVAILABLE(0x20)은 여기서
                            -- 뺐다: LoRa 자체가 끊기면 이 텔레메트리도 GCS에
                            -- 안 도착하니 "LoRa로 LoRa 끊김을 알린다"는 게
                            -- 애초에 모순이라 전송할 필요가 없다 - GCS는
                            -- 이미 자체 타임아웃(마지막 패킷 수신 후 경과
                            -- 시간)으로 LoRa 링크 생사를 판단한다
                            -- (gcs/main.py의 LORA_LINK_TIMEOUT_SEC).
                            -- comet_lora.lua의 pack_state_fault()/
                            -- unpack_state_fault() 참고.
    COMMAND_LENGTH    = 4,   -- header(1) + sequence(1) + command(1) + crc8(1)
    ACK_LENGTH        = 4,   -- header(1) + sequence(1) + ack_result(1) + crc8(1)
}

-- LoRa 원격 명령 큐. pending_command 단일 변수 대신 FIFO 링버퍼로 명령을
-- 적재해서, 연속 명령(FORCE_ASCENT -> FORCE_DEPLOY 등)이 상태머신 처리
-- 전에 겹쳐 들어와도 유실되지 않게 한다 (comet_command_queue.lua 참고).
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
        DEPLOY_ENABLE = false,

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
        LANDED_CONFIRM_COUNT = 40
    }
}

return CONFIG
