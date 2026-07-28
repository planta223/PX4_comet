-- =====================================================================
-- Comet Main (단일 파일 병합판)
-- =====================================================================
-- 기존에는 comet_config/sensor/servo/lora/flight_state.lua로 분리하고
-- require()로 서로 불러오는 구조였으나, 실제 보드에서 require()가
-- comet_config 등을 찾지 못하는 문제(module not found)가 반복 확인되어
-- require() 의존을 완전히 제거하고 이 파일 하나로 합쳤다.
--
-- 섹션 순서는 의존관계를 따른다: config -> sensor -> servo -> lora
-- -> flight_state -> main. 각 섹션은 원래 파일의 "local M = {}" 모듈
-- 테이블을 그 모듈의 원래 이름(sensor/servo/lora/flight)으로 바꿔서
-- 그대로 담고 있다. 각 섹션 끝의 "return M" 등은 전부 제거했다 -
-- 한 파일이 되면 return이 그 자리에서 스크립트 전체를 끝내버리기
-- 때문이다. 실제 재스케줄 return은 이 파일 맨 끝, 딱 한 번뿐이다.
--
-- 메인 루프 처리 순서:
-- 센서 갱신 → LoRa 명령 수신 → 비행 상태 갱신 → 텔레메트리 송신 → 리셋 스위치 확인
-- =====================================================================


-- =====================================================================
-- [SECTION] Configuration (원래 comet_config.lua)
-- =====================================================================

local CONFIG = {}

-- ==========================================================================
-- 테스트 모드 - 실내 책상 테스트용. 실제 비행 전에 반드시 false로 되돌릴 것.
-- GPS 3D fix가 없어도(실내) 상태머신이 ERROR로 안 떨어지고 ASCENT->APOGEE->
-- DESCENT->LANDED까지 흘러가도록, 아래 두 가지만 완화한다:
--   1. READY->ASCENT 가속도 문턱값을 낮춰서 가볍게 흔들기만 해도 발사로 인식
--   2. ASCENT/DESCENT 단계의 센서 유효성 검사에서 GPS position/velocity
--      valid 요구를 뺀다 (실내에선 GPS fix가 안 잡히니 계속 false라
--      SENSOR_TIMEOUT_MS를 넘겨 바로 ERROR로 떨어졌었음)
-- ==========================================================================
CONFIG.TEST_MODE = true

CONFIG.UPDATE_PERIOD_MS = 50

CONFIG.SAFETY = {
    MAX_TILT_DEG = 70.0,
    SENSOR_TIMEOUT_MS = 1500,

    -- 사출 검증 및 재시도
    DEPLOY_VERIFY_DELAY_MS = 1500, -- 사출 후 이 시간이 지나면 가속도로 전개 여부를 검증
    FREEFALL_ACCEL_MPS2 = 5.0,     -- 이 값 미만이면 항력이 느껴지지 않는 것으로 보고 미전개로 판단

    -- ERROR 상태 사출 재시도 최소 간격
    ERROR_RETRY_COOLDOWN_MS = 500
}

CONFIG.RESET = {
    RC_CHANNEL = 7,   -- 확인 필요 (Mission Planner 또는 조종기 채널 매핑 확인)
    RC_HIGH_PWM = 1700
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
    BAUD = 9600,             -- DX-LR03 모듈 공장 기본값(9600/8N1)에 맞춤 - 확인 후 57600으로 재조정 검토

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
        -- 테스트 모드: 가볍게 흔들기만 해도 ASCENT로 넘어가도록 문턱값을 낮춤
        -- (원래 25.0 - 실제 발사 가속도 기준, 실비행 전 반드시 복원)
        ACCEL_THRESHOLD_MPS2 = CONFIG.TEST_MODE and 3.0 or 25.0,
        CONFIRM_COUNT = 1
    },

    ASCENT_TO_APOGEE = {
        MIN_TIME_AFTER_LAUNCH_MS = 1000,
        MIN_DESCENT_SPEED_MPS = 1.0,
        ALTITUDE_DROP_MARGIN_M = 2.0,
        CONFIRM_COUNT = 3,
        MAX_ASCENT_TIME_MS = 150000000
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


-- =====================================================================
-- [SECTION] Sensor Interface (원래 comet_sensor.lua, M -> sensor)
-- =====================================================================

local sensor = {}

local RAD_TO_DEG = 180.0 / math.pi


local sensor_data = {
    -- Position
    latitude = 0.0,           -- deg
    longitude = 0.0,          -- deg
    altitude = 0.0,           -- relative altitude from HOME, m

    -- Velocity, NED frame
    velocity_n = 0.0,         -- m/s, north positive
    velocity_e = 0.0,         -- m/s, east positive
    velocity_d = 0.0,         -- m/s, down positive

    -- Attitude
    roll = 0.0,               -- deg
    pitch = 0.0,              -- deg
    yaw = 0.0,                -- deg, -180 ~ +180

    -- Acceleration
    total_acceleration = 0.0, -- m/s^2, 3-axis magnitude

    -- Validity
    position_valid = false,
    velocity_valid = false,
    attitude_valid = false,
    acceleration_valid = false
}


-- yaw를 -180 ~ +180 deg 범위로 정규화
local function normalize_yaw_deg(yaw)
    while yaw > 180.0 do
        yaw = yaw - 360.0
    end

    while yaw < -180.0 do
        yaw = yaw + 360.0
    end

    return yaw
end


-- 숫자가 NaN인지 검사
local function is_nan(value)
    return value ~= value
end


-- 일반 숫자 유효성 검사
local function is_valid_number(value)
    return value ~= nil
        and not is_nan(value)
        and value ~= math.huge
        and value ~= -math.huge
end


local function update_position()
    local position = ahrs:get_position()

    if position == nil then
        sensor_data.position_valid = false
        return false
    end

    -- ArduPilot Location:
    -- lat() : deg × 1e7
    -- lng() : deg × 1e7
    -- alt() : cm
    local latitude = position:lat() / 10000000.0
    local longitude = position:lng() / 10000000.0

    local altitude_m = 0.0
    local home = ahrs:get_home()

    if home ~= nil then
        -- 현재 위치와 HOME의 고도 차이를 발사점 기준 상대고도로 사용
        altitude_m = (position:alt() - home:alt()) / 100.0
    else
        -- HOME이 없으면 상대고도 계산 불가
        sensor_data.position_valid = false
        return false
    end

    if not is_valid_number(latitude)
        or not is_valid_number(longitude)
        or not is_valid_number(altitude_m) then

        sensor_data.position_valid = false
        return false
    end

    sensor_data.latitude = latitude
    sensor_data.longitude = longitude
    sensor_data.altitude = altitude_m
    sensor_data.position_valid = true

    return true
end


local function update_velocity()
    local velocity = ahrs:get_velocity_NED()

    if velocity == nil then
        sensor_data.velocity_valid = false
        return false
    end

    if velocity:is_nan() or velocity:is_inf() then
        sensor_data.velocity_valid = false
        return false
    end

    sensor_data.velocity_n = velocity:x()
    sensor_data.velocity_e = velocity:y()
    sensor_data.velocity_d = velocity:z()

    sensor_data.velocity_valid = true
    return true
end


local function update_attitude()
    local roll_rad = ahrs:get_roll()
    local pitch_rad = ahrs:get_pitch()
    local yaw_rad = ahrs:get_yaw()

    if not is_valid_number(roll_rad)
        or not is_valid_number(pitch_rad)
        or not is_valid_number(yaw_rad) then

        sensor_data.attitude_valid = false
        return false
    end

    sensor_data.roll = roll_rad * RAD_TO_DEG
    sensor_data.pitch = pitch_rad * RAD_TO_DEG
    sensor_data.yaw = normalize_yaw_deg(yaw_rad * RAD_TO_DEG)

    sensor_data.attitude_valid = true
    return true
end


local function update_acceleration()
    local acceleration = ahrs:get_accel()

    if acceleration == nil then
        sensor_data.acceleration_valid = false
        return false
    end

    if acceleration:is_nan() or acceleration:is_inf() then
        sensor_data.acceleration_valid = false
        return false
    end

    local total_acceleration = acceleration:length()

    if not is_valid_number(total_acceleration) then
        sensor_data.acceleration_valid = false
        return false
    end

    sensor_data.total_acceleration = total_acceleration
    sensor_data.acceleration_valid = true

    return true
end


function sensor.initialize()
    sensor_data.latitude = 0.0
    sensor_data.longitude = 0.0
    sensor_data.altitude = 0.0

    sensor_data.velocity_n = 0.0
    sensor_data.velocity_e = 0.0
    sensor_data.velocity_d = 0.0

    sensor_data.roll = 0.0
    sensor_data.pitch = 0.0
    sensor_data.yaw = 0.0

    sensor_data.total_acceleration = 0.0

    sensor_data.position_valid = false
    sensor_data.velocity_valid = false
    sensor_data.attitude_valid = false
    sensor_data.acceleration_valid = false

    gcs:send_text(6, "COMET: Sensor interface initialized")
    return true
end


function sensor.update()
    update_position()
    update_velocity()
    update_attitude()
    update_acceleration()

    return true
end


function sensor.get_data()
    return sensor_data
end


function sensor.get_status_flags()
    local flags = 0

    -- bit 0: position valid
    if sensor_data.position_valid then
        flags = flags | 0x01
    end

    -- bit 1: velocity valid
    if sensor_data.velocity_valid then
        flags = flags | 0x02
    end

    -- bit 2: attitude valid
    if sensor_data.attitude_valid then
        flags = flags | 0x04
    end

    -- bit 3: acceleration valid
    if sensor_data.acceleration_valid then
        flags = flags | 0x08
    end

    return flags
end


-- =====================================================================
-- [SECTION] Servo (Parachute) Control (원래 comet_servo.lua, M -> servo)
-- =====================================================================

local SERVO = CONFIG.SERVO

local servo = {}


-- 설정 테이블에서 서보 이름에 해당하는 설정을 반환한다.
local function get_servo_config(name)
    local config = SERVO[name]

    if config == nil then
        gcs:send_text(3, "COMET: Invalid servo name")
        return nil
    end

    return config
end


-- 지정된 서보 기능에 PWM 값을 출력한다.
local function set_pwm(name, config, pwm)
    if type(pwm) ~= "number" then
        gcs:send_text(3, "COMET: Invalid " .. name .. " PWM")
        return false
    end

    -- FUNCTION_ID가 실제 출력 채널에 할당되어 있는지 확인한다.
    local channel =
        SRV_Channels:find_channel(config.FUNCTION_ID)

    if channel == nil then
        gcs:send_text(
            3,
            "COMET: " .. name .. " function not assigned"
        )
        return false
    end

    SRV_Channels:set_output_pwm(
        config.FUNCTION_ID,
        pwm
    )

    return true
end


function servo.initialize()
    -- 부팅 시 모든 낙하산 서보를 잠금 상태로 초기화한다.
    local ok = servo.lock_all()

    if not ok then
        gcs:send_text(3, "COMET: Servo initialization failed")
        return false
    end

    gcs:send_text(6, "COMET: Servos initialized")
    return true
end


function servo.lock(name)
    local config = get_servo_config(name)

    if config == nil then
        return false
    end

    return set_pwm(
        name,
        config,
        config.LOCK_PWM
    )
end


function servo.release(name)
    local config = get_servo_config(name)

    if config == nil then
        return false
    end

    return set_pwm(
        name,
        config,
        config.RELEASE_PWM
    )
end


function servo.lock_all()
    -- 한 서보가 실패하더라도 다른 서보 제어는 시도한다.
    local servo1_ok = servo.lock("SERVO1")
    local servo2_ok = servo.lock("SERVO2")

    return servo1_ok and servo2_ok
end


function servo.release_all()
    -- 한 서보가 실패하더라도 다른 서보 제어는 시도한다.
    local servo1_ok = servo.release("SERVO1")
    local servo2_ok = servo.release("SERVO2")

    return servo1_ok and servo2_ok
end


-- =====================================================================
-- [SECTION] LoRa Communication (원래 comet_lora.lua, M -> lora)
-- =====================================================================

local LORA = CONFIG.LORA

local lora = {}


local port = nil

local rx_buffer = {}
local rx_index = 1
local pending_command = nil

local last_ack_sequence = nil
local last_ack_command = nil
local last_ack_result = nil

local telemetry_sequence = 0
local last_send_ms = 0


lora.COMMAND = {
    SERVO1_LOCK = 1,
    SERVO1_RELEASE = 2,
    SERVO2_LOCK = 3,
    SERVO2_RELEASE = 4,
    LOCK_ALL = 5,
    RELEASE_ALL = 6,
    RESET = 7
}

lora.ACK_RESULT = {
    OK = 0,
    INVALID_COMMAND = 1,
    NOT_ALLOWED = 2,
    EXECUTION_FAILED = 3
}


-- CRC-8 (poly=0x07, init=0x00) 계산.
local function calculate_crc8(bytes, length)
    local crc = 0x00

    for i = 1, length do
        crc = crc ~ bytes[i]

        for _ = 1, 8 do
            if (crc & 0x80) ~= 0 then
                crc = ((crc << 1) ~ 0x07) & 0xFF
            else
                crc = (crc << 1) & 0xFF
            end
        end
    end

    return crc
end


-- 바이트 배열을 UART로 전송한다.
local function send_packet(packet)
    if port == nil then
        return false
    end

    for i = 1, #packet do
        port:write(packet[i])
    end

    return true
end


-- 값을 지정된 범위로 제한한다.
local function clamp(value, min_value, max_value)
    if value < min_value then
        return min_value
    end

    if value > max_value then
        return max_value
    end

    return value
end


-- 실수를 가장 가까운 정수로 반올림한다.
local function round(value)
    if value >= 0 then
        return math.floor(value + 0.5)
    end

    return math.ceil(value - 0.5)
end


-- yaw를 -180 ~ +180 deg 범위로 정규화한다.
-- (sensor 섹션에도 동명 함수가 있으나, 각 섹션에서 그 시점에 보이는
--  local이 알아서 잡히므로 - 즉 여기서부터 아래 lora 코드는 이 버전을
--  쓴다 - 문제 없다. 두 구현은 완전히 동일하다.)
local function normalize_yaw_deg(yaw)
    while yaw > 180.0 do
        yaw = yaw - 360.0
    end

    while yaw < -180.0 do
        yaw = yaw + 360.0
    end

    return yaw
end


local function append_u8(packet, value)
    packet[#packet + 1] = value % 256
end


local function append_i16(packet, value)
    value = clamp(round(value), -32768, 32767)
    value = value % 65536

    packet[#packet + 1] = value % 256
    packet[#packet + 1] = math.floor(value / 256) % 256
end


local function append_u32(packet, value)
    value = math.floor(value) % 4294967296

    packet[#packet + 1] = value % 256
    packet[#packet + 1] = math.floor(value / 256) % 256
    packet[#packet + 1] = math.floor(value / 65536) % 256
    packet[#packet + 1] = math.floor(value / 16777216) % 256
end


local function append_i32(packet, value)
    value = clamp(round(value), -2147483648, 2147483647)
    value = value % 4294967296

    packet[#packet + 1] = value % 256
    packet[#packet + 1] = math.floor(value / 256) % 256
    packet[#packet + 1] = math.floor(value / 65536) % 256
    packet[#packet + 1] = math.floor(value / 16777216) % 256
end


local function process_command_packet()
    local header = rx_buffer[1]
    local sequence = rx_buffer[2]
    local command = rx_buffer[3]
    local received_crc = rx_buffer[4]

    if header ~= LORA.COMMAND_HEADER then
        return
    end

    local expected_crc =
        calculate_crc8(
            rx_buffer,
            LORA.COMMAND_LENGTH - 1
        )

    if received_crc ~= expected_crc then
        gcs:send_text(4, "COMET: LoRa command CRC error")
        return
    end

    if command < lora.COMMAND.SERVO1_LOCK
        or command > lora.COMMAND.RESET then

        lora.send_ack(
            sequence,
            command,
            lora.ACK_RESULT.INVALID_COMMAND
        )
        return
    end

    -- 이미 처리한 동일 명령이 재전송되면 기존 ACK만 다시 전송한다.
    if last_ack_sequence == sequence
        and last_ack_command == command then

        lora.send_ack(
            sequence,
            command,
            last_ack_result
        )
        return
    end

    -- 상태머신이 아직 이전 명령을 처리하지 않았다면 새 명령을 보류한다.
    if pending_command ~= nil then
        return
    end

    pending_command = {
        sequence = sequence,
        command = command
    }
end


local function process_received_byte(byte)
    if rx_index == 1
        and byte ~= LORA.COMMAND_HEADER then

        return
    end

    rx_buffer[rx_index] = byte
    rx_index = rx_index + 1

    if rx_index > LORA.COMMAND_LENGTH then
        process_command_packet()

        rx_buffer = {}
        rx_index = 1
    end
end


local function receive_commands()
    if port == nil then
        return false
    end

    while port:available() > 0 do
        local byte = port:read()

        if byte == nil then
            break
        end

        process_received_byte(byte)
    end

    return true
end


local function send_telemetry(
    now,
    telemetry_sensor_data,
    flight_state,
    status_flags
)
    if port == nil or telemetry_sensor_data == nil then
        return false
    end

    local packet = {}

    append_u8(packet, LORA.TELEMETRY_HEADER)
    append_u8(packet, telemetry_sequence)
    append_u8(packet, flight_state or 0)
    append_u8(packet, status_flags or 0)

    append_u32(packet, now)

    -- latitude, longitude: deg × 1e7
    -- altitude: relative m × 100
    append_i32(
        packet,
        (telemetry_sensor_data.latitude or 0) * 10000000
    )

    append_i32(
        packet,
        (telemetry_sensor_data.longitude or 0) * 10000000
    )

    append_i32(
        packet,
        (telemetry_sensor_data.altitude or 0) * 100
    )

    -- NED velocity: m/s × 10
    append_i16(
        packet,
        (telemetry_sensor_data.velocity_n or 0) * 10
    )

    append_i16(
        packet,
        (telemetry_sensor_data.velocity_e or 0) * 10
    )

    append_i16(
        packet,
        (telemetry_sensor_data.velocity_d or 0) * 10
    )

    -- attitude: deg × 100
    append_i16(
        packet,
        (telemetry_sensor_data.roll or 0) * 100
    )

    append_i16(
        packet,
        (telemetry_sensor_data.pitch or 0) * 100
    )

    local yaw =
        normalize_yaw_deg(telemetry_sensor_data.yaw or 0)

    append_i16(packet, yaw * 100)

    if #packet ~= LORA.TELEMETRY_LENGTH - 1 then
        gcs:send_text(
            3,
            "COMET: Invalid telemetry packet length"
        )
        return false
    end

    packet[#packet + 1] =
        calculate_crc8(
            packet,
            LORA.TELEMETRY_LENGTH - 1
        )

    return send_packet(packet)
end


local function get_telemetry_period(flight_state)
    -- 비행 중에는 빠른 주기를 사용한다.
    if flight_state == 1
        or flight_state == 2
        or flight_state == 3 then

        return LORA.FAST_PERIOD_MS
    end

    return LORA.NORMAL_PERIOD_MS
end


function lora.initialize()
    port = serial:find_serial(LORA.SERIAL_INSTANCE)

    if port == nil then
        gcs:send_text(3, "COMET: LoRa initialization failed")
        return false
    end

    port:begin(LORA.BAUD)
    port:set_flow_control(0)

    rx_buffer = {}
    rx_index = 1
    pending_command = nil

    last_ack_sequence = nil
    last_ack_command = nil
    last_ack_result = nil

    telemetry_sequence = 0
    last_send_ms = 0

    gcs:send_text(6, "COMET: LoRa initialized")
    return true
end


function lora.receive()
    return receive_commands()
end


function lora.send_ack(sequence, command, result)
    if port == nil then
        return false
    end

    local packet = {
        LORA.ACK_HEADER,
        sequence,
        result
    }

    packet[LORA.ACK_LENGTH] =
        calculate_crc8(
            packet,
            LORA.ACK_LENGTH - 1
        )

    local sent = send_packet(packet)

    if sent then
        last_ack_sequence = sequence
        last_ack_command = command
        last_ack_result = result
    end

    return sent
end


function lora.send_telemetry_if_due(
    now,
    telemetry_sensor_data,
    flight_state,
    status_flags
)
    if port == nil then
        return false
    end

    local period =
        get_telemetry_period(flight_state)

    if now - last_send_ms < period then
        return true
    end

    local sent = send_telemetry(
        now,
        telemetry_sensor_data,
        flight_state,
        status_flags
    )

    if sent then
        last_send_ms = now
        telemetry_sequence =
            (telemetry_sequence + 1) % 256
    end

    return sent
end


function lora.get_command()
    if pending_command == nil then
        return nil
    end

    local command = pending_command
    pending_command = nil

    return command
end


-- =====================================================================
-- [SECTION] Flight State Machine (원래 comet_flight_state.lua, M -> flight)
-- =====================================================================

local SAFETY = CONFIG.SAFETY
local TRANSITION = CONFIG.TRANSITION

local READY_TO_ASCENT = TRANSITION.READY_TO_ASCENT
local ASCENT_TO_APOGEE = TRANSITION.ASCENT_TO_APOGEE
local APOGEE_TO_DESCENT = TRANSITION.APOGEE_TO_DESCENT
local DESCENT_TO_LANDED = TRANSITION.DESCENT_TO_LANDED

local flight = {}


flight.STATE = {
    READY = 0,
    ASCENT = 1,
    APOGEE = 2,
    DESCENT = 3,
    LANDED = 4,
    ERROR = 5
}


local current_state = flight.STATE.READY

local launch_time_ms = 0
local state_enter_time_ms = 0
local last_sensor_time_ms = 0

local maximum_altitude_m = 0.0

local launch_confirm_count = 0
local apogee_confirm_count = 0
local landing_confirm_count = 0

local parachute_deployed = false

-- 사출 검증 및 재시도
local deploy_time_ms = 0
local deploy_verified = false
local retry_issued = false

-- ERROR 상태 사출 재시도 쿨다운
local last_error_retry_ms = 0


local function change_state(new_state, now)
    if current_state == new_state then
        return
    end

    -- 유효성 검사
    if new_state < flight.STATE.READY
        or new_state > flight.STATE.ERROR then

        gcs:send_text(3, "COMET: Invalid flight state")
        return
    end

    current_state = new_state
    state_enter_time_ms = now

    if new_state == flight.STATE.READY then
        gcs:send_text(6, "COMET: State READY")

    elseif new_state == flight.STATE.ASCENT then
        gcs:send_text(6, "COMET: State ASCENT")

    elseif new_state == flight.STATE.APOGEE then
        gcs:send_text(6, "COMET: State APOGEE")

    elseif new_state == flight.STATE.DESCENT then
        gcs:send_text(6, "COMET: State DESCENT")

    elseif new_state == flight.STATE.LANDED then
        gcs:send_text(6, "COMET: State LANDED")

    elseif new_state == flight.STATE.ERROR then
        gcs:send_text(3, "COMET: State ERROR")
    end
end


local function reset_state(now)
    current_state = flight.STATE.READY

    launch_time_ms = 0
    state_enter_time_ms = now
    last_sensor_time_ms = now

    maximum_altitude_m = 0.0

    launch_confirm_count = 0
    apogee_confirm_count = 0
    landing_confirm_count = 0

    parachute_deployed = false

    deploy_time_ms = 0
    deploy_verified = false
    retry_issued = false

    last_error_retry_ms = 0
end


-- 낙하산 전개 상태를 기록하고, 사출 검증 타이머를 시작한다.
local function mark_parachute_deployed(now)
    parachute_deployed = true

    deploy_time_ms = now
    deploy_verified = false
    retry_issued = false
end


local function deploy_parachute(now)
    if parachute_deployed then
        return true
    end

    local ok = servo.release_all()

    if not ok then
        if CONFIG.TEST_MODE then
            -- 테스트 모드: 서보가 안 붙어있어도(FUNCTION_ID 미할당 등)
            -- 상태머신이 ERROR로 안 떨어지고 계속 진행되도록, 경고만
            -- 남기고 사출된 것으로 간주한다.
            gcs:send_text(4, "COMET: [TEST] 서보 사출 실패 - ERROR 대신 사출된 것으로 간주")
            mark_parachute_deployed(now)
            return true
        end
        gcs:send_text(3, "COMET: Parachute deployment failed")
        return false
    end

    mark_parachute_deployed(now)
    gcs:send_text(6, "COMET: Parachute deployed")

    return true
end


-- 사출 후 일정 시간이 지나면 가속도로 실제 전개 여부를 검증하고,
-- 미전개로 판단되면 1회에 한해 재사출을 시도한다.
local function verify_deploy(now, verify_sensor_data)
    if not parachute_deployed
        or deploy_verified
        or retry_issued then

        return
    end

    if now - deploy_time_ms < SAFETY.DEPLOY_VERIFY_DELAY_MS then
        return
    end

    if verify_sensor_data == nil or not verify_sensor_data.acceleration_valid then
        -- 가속도 데이터가 무효하면 검증을 건너뛰고 성공으로 간주한다.
        deploy_verified = true
        return
    end

    if verify_sensor_data.total_acceleration < SAFETY.FREEFALL_ACCEL_MPS2 then
        gcs:send_text(4, "COMET: Parachute deploy unconfirmed, retrying")

        retry_issued = true
        local ok = servo.release_all()

        -- 테스트 모드에서는 서보 재시도 실패도 ERROR로 보내지 않는다
        -- (deploy_parachute와 동일한 이유 - 서보 미연결 상태로 계속 테스트).
        if not ok and not CONFIG.TEST_MODE then
            change_state(flight.STATE.ERROR, now)
        end
    else
        deploy_verified = true
        gcs:send_text(6, "COMET: Parachute deploy confirmed")
    end
end


local function execute_command(command, now)
    if command == lora.COMMAND.SERVO1_LOCK then
        return servo.lock("SERVO1")

    elseif command == lora.COMMAND.SERVO1_RELEASE then
        return servo.release("SERVO1")

    elseif command == lora.COMMAND.SERVO2_LOCK then
        return servo.lock("SERVO2")

    elseif command == lora.COMMAND.SERVO2_RELEASE then
        return servo.release("SERVO2")

    elseif command == lora.COMMAND.LOCK_ALL then
        return servo.lock_all()

    elseif command == lora.COMMAND.RELEASE_ALL then
        return servo.release_all()

    elseif command == lora.COMMAND.RESET then
        return flight.reset(now)
    end

    return false
end


local function is_command_allowed(command)
    -- 비행 중 서보 잠금은 금지
    if command == lora.COMMAND.SERVO1_LOCK
        or command == lora.COMMAND.SERVO2_LOCK
        or command == lora.COMMAND.LOCK_ALL then

        return current_state == flight.STATE.READY
            or current_state == flight.STATE.LANDED
    end

    -- READY 상태에서는 오작동 방지를 위해 원격 사출 금지
    if command == lora.COMMAND.SERVO1_RELEASE
        or command == lora.COMMAND.SERVO2_RELEASE
        or command == lora.COMMAND.RELEASE_ALL then

        return current_state == flight.STATE.APOGEE
            or current_state == flight.STATE.DESCENT
            or current_state == flight.STATE.ERROR
    end

    -- RESET은 flight.reset() 자체도 READY/LANDED에서만 동작하도록
    -- 이중으로 가드되어 있지만, 여기서도 동일하게 게이팅해서
    -- 허용되지 않는 상태에서는 NOT_ALLOWED ACK로 명확히 응답한다.
    if command == lora.COMMAND.RESET then
        return current_state == flight.STATE.READY
            or current_state == flight.STATE.LANDED
    end

    return false
end


local function process_lora_command(now)
    local received = lora.get_command()

    if received == nil then
        return
    end

    local sequence = received.sequence
    local command = received.command

    if not is_command_allowed(command) then
        lora.send_ack(
            sequence,
            command,
            lora.ACK_RESULT.NOT_ALLOWED
        )
        return
    end

    local ok = execute_command(command, now)

    if ok then
        -- 서보 1, 2 모두 RELEASE 상태일 때 낙하산 전개 판단
        if command == lora.COMMAND.RELEASE_ALL then
            mark_parachute_deployed(now)
        end

        lora.send_ack(
            sequence,
            command,
            lora.ACK_RESULT.OK
        )
    else
        lora.send_ack(
            sequence,
            command,
            lora.ACK_RESULT.EXECUTION_FAILED
        )
    end
end


-- READY 상태에서는 가속도값 이용
local function ready_sensor_is_valid(check_sensor_data)
    return check_sensor_data ~= nil
        and check_sensor_data.acceleration_valid
end

-- ASCENT 상태에서는 위치·속도값 이용.
-- 테스트 모드에서는 GPS position/velocity valid 요구를 뺀다 - 실내에선
-- GPS 3D fix가 안 잡혀 계속 false가 되고, 그러면 SENSOR_TIMEOUT_MS를
-- 넘겨 바로 ERROR로 떨어져서 ASCENT 이후 단계를 테스트할 수가 없었다.
local function ascent_sensor_is_valid(check_sensor_data)
    if check_sensor_data == nil then
        return false
    end
    if CONFIG.TEST_MODE then
        return true
    end
    return check_sensor_data.position_valid
        and check_sensor_data.velocity_valid
end

-- DESCENT 상태에서도 동일하게, 테스트 모드에서는 가속도 유효성만 요구한다
-- (가속도계는 실내에서도 항상 유효하므로 착지 판정 자체는 계속 동작함).
local function descent_sensor_is_valid(check_sensor_data)
    if check_sensor_data == nil then
        return false
    end
    if CONFIG.TEST_MODE then
        return check_sensor_data.acceleration_valid
    end
    return check_sensor_data.position_valid
        and check_sensor_data.velocity_valid
        and check_sensor_data.acceleration_valid
end

-- 상세한 사항은 flight_state_description.txt 참고

local function update_ready(now, ready_sensor_data)
    if not ready_sensor_is_valid(ready_sensor_data) then
        launch_confirm_count = 0
        return
    end

    if ready_sensor_data.total_acceleration
        >= READY_TO_ASCENT.ACCEL_THRESHOLD_MPS2 then

        launch_confirm_count = launch_confirm_count + 1
    else
        launch_confirm_count = 0
    end

    if launch_confirm_count >= READY_TO_ASCENT.CONFIRM_COUNT then
        launch_time_ms = now
        last_sensor_time_ms = now

        if ready_sensor_data.position_valid then
            maximum_altitude_m = ready_sensor_data.altitude
        else
            maximum_altitude_m = 0.0
        end

        launch_confirm_count = 0
        apogee_confirm_count = 0
        landing_confirm_count = 0

        change_state(flight.STATE.ASCENT, now)
    end
end

local function update_ascent(now, ascent_sensor_data)
    if not ascent_sensor_is_valid(ascent_sensor_data) then
        return
    end

    if ascent_sensor_data.altitude > maximum_altitude_m then
        maximum_altitude_m = ascent_sensor_data.altitude
        apogee_confirm_count = 0
    end

    local minimum_time_passed =
        now - launch_time_ms
            >= ASCENT_TO_APOGEE.MIN_TIME_AFTER_LAUNCH_MS

    local descending =
        ascent_sensor_data.velocity_d
            >= ASCENT_TO_APOGEE.MIN_DESCENT_SPEED_MPS

    local altitude_dropped =
        ascent_sensor_data.altitude
            <= maximum_altitude_m
                - ASCENT_TO_APOGEE.ALTITUDE_DROP_MARGIN_M

    if minimum_time_passed
        and descending
        and altitude_dropped then

        apogee_confirm_count = apogee_confirm_count + 1
    else
        apogee_confirm_count = 0
    end

    local ascent_timeout =
        now - launch_time_ms
            >= ASCENT_TO_APOGEE.MAX_ASCENT_TIME_MS

    if apogee_confirm_count
        >= ASCENT_TO_APOGEE.CONFIRM_COUNT
        or ascent_timeout then

        change_state(flight.STATE.APOGEE, now)
    end
end


local function update_apogee(now)
    if not parachute_deployed then
        local ok = deploy_parachute(now)

        if not ok then
            change_state(flight.STATE.ERROR, now)
            return
        end
    end

    if now - state_enter_time_ms
        >= APOGEE_TO_DESCENT.MIN_HOLD_TIME_MS then

        change_state(flight.STATE.DESCENT, now)
    end
end


local function update_descent(now, descent_sensor_data)
    if not descent_sensor_is_valid(descent_sensor_data) then
        landing_confirm_count = 0
        return
    end

    local altitude_low =
        descent_sensor_data.altitude
            <= DESCENT_TO_LANDED.MAX_RELATIVE_ALTITUDE_M

    local vertical_speed_low =
        math.abs(descent_sensor_data.velocity_d)
            <= DESCENT_TO_LANDED.MAX_ABS_VERTICAL_SPEED_MPS

    local acceleration_stable =
        descent_sensor_data.total_acceleration
            >= DESCENT_TO_LANDED.ACCEL_MIN_MPS2
        and descent_sensor_data.total_acceleration
            <= DESCENT_TO_LANDED.ACCEL_MAX_MPS2

    if altitude_low
        and vertical_speed_low
        and acceleration_stable then

        landing_confirm_count = landing_confirm_count + 1
    else
        landing_confirm_count = 0
    end

    if landing_confirm_count
        >= DESCENT_TO_LANDED.CONFIRM_COUNT then

        change_state(flight.STATE.LANDED, now)
    end
end


local function update_error(now)
    if parachute_deployed then
        return
    end

    -- 매 루프 재시도로 인한 서보 신호 남발을 막기 위한 최소 재시도 간격
    if now - last_error_retry_ms < SAFETY.ERROR_RETRY_COOLDOWN_MS then
        return
    end

    last_error_retry_ms = now
    deploy_parachute(now)
end


function flight.initialize(now)
    reset_state(now or 0)

    gcs:send_text(6, "COMET: Flight state initialized")
    return true
end


function flight.update(now, update_sensor_data)
    -- LoRa 명령은 센서 상태와 관계없이 우선 처리
    process_lora_command(now)

    -- 사출 검증/재시도는 현재 상태와 무관하게 매 루프 확인한다.
    verify_deploy(now, update_sensor_data)

    if current_state == flight.STATE.READY then
        update_ready(now, update_sensor_data)

    elseif current_state == flight.STATE.ASCENT then
        if ascent_sensor_is_valid(update_sensor_data) then
            last_sensor_time_ms = now
            update_ascent(now, update_sensor_data)

        elseif now - last_sensor_time_ms
            >= SAFETY.SENSOR_TIMEOUT_MS then

            change_state(flight.STATE.ERROR, now)
        end

    elseif current_state == flight.STATE.APOGEE then
        update_apogee(now)

    elseif current_state == flight.STATE.DESCENT then
        update_descent(now, update_sensor_data)

    elseif current_state == flight.STATE.ERROR then
        update_error(now)
    end

    return true
end


function flight.get_state()
    return current_state
end


function flight.get_maximum_altitude()
    return maximum_altitude_m
end


function flight.is_parachute_deployed()
    return parachute_deployed
end


function flight.reset(now)
    if current_state ~= flight.STATE.READY
        and current_state ~= flight.STATE.LANDED then

        return false
    end

    local ok = servo.lock_all()

    if not ok then
        gcs:send_text(3, "COMET: Flight state reset failed")
        return false
    end

    reset_state(now or 0)

    gcs:send_text(6, "COMET: Flight state reset")
    return true
end


-- =====================================================================
-- [SECTION] Main (원래 comet_main.lua 본체)
-- =====================================================================

local initialized = false
local reset_switch_was_high = false


local function startup()
    local now = millis():toint()

    local sensor_ok = sensor.initialize()
    local servo_ok = servo.initialize()
    local lora_ok = lora.initialize()
    local flight_ok = flight.initialize(now)

    if not sensor_ok then
        gcs:send_text(3, "COMET: Sensor initialization failed")
    end

    if not servo_ok then
        gcs:send_text(3, "COMET: Servo initialization failed")
    end

    if not lora_ok then
        gcs:send_text(4, "COMET: LoRa unavailable")
    end

    if not flight_ok then
        gcs:send_text(3, "COMET: Flight initialization failed")
    end

    -- 센서, 서보 및 상태머신 초기화 실패는 치명적 오류로 처리한다.
    -- 테스트 모드에서는 서보 초기화 실패(서보 미연결)만은 치명적으로 안
    -- 보고 넘어간다 - 안 그러면 initialized=false가 되어 flight.update()
    -- 자체가 안 불려서 상태머신이 READY에서 완전히 멈춰버린다.
    local servo_required_ok = servo_ok or CONFIG.TEST_MODE
    local critical_ok =
        sensor_ok
        and servo_required_ok
        and flight_ok

    if not servo_ok and CONFIG.TEST_MODE then
        gcs:send_text(4, "COMET: [TEST] 서보 초기화 실패했지만 테스트 모드라 계속 진행")
    end

    if not critical_ok then
        gcs:send_text(2, "COMET: Critical initialization failed")
        -- 비행 상태머신/서보 제어는 비활성화되지만, 센서 갱신과 LoRa
        -- 송수신은 update()에서 계속 동작한다 (통신/센서 검증 목적).
        gcs:send_text(4, "COMET: Comms-only mode (flight logic disabled)")
        return false
    end

    -- LoRa는 백업 통신이므로 초기화 실패 시에도 비행 로직은 계속 실행한다.
    if not lora_ok then
        gcs:send_text(4, "COMET: Running without LoRa")
    end

    gcs:send_text(6, "COMET: System initialized")
    return true
end


-- RESET 채널이 켜지는 순간(rising edge)에만 1회 리셋을 실행한다.
-- flight.reset()은 READY/LANDED 상태에서만 동작하므로 스위치를 계속
-- 켜둬도 반복 실행되지 않지만, 불필요한 재시도와 로그 반복을 막기
-- 위해 엣지 감지를 사용한다.
local function check_reset_switch(now)
    local pwm = rc:get_pwm(CONFIG.RESET.RC_CHANNEL)

    if pwm == nil then
        return
    end

    local is_high = pwm > CONFIG.RESET.RC_HIGH_PWM

    if is_high and not reset_switch_was_high then
        flight.reset(now)
    end

    reset_switch_was_high = is_high
end


local function update()
    local now = millis():toint()

    -- 센서 갱신과 LoRa 송수신은 치명적 초기화(특히 서보) 성공 여부와
    -- 무관하게 항상 동작한다 - 통신/센서 쪽만 먼저 검증하고 싶을 때
    -- 서보 하드웨어 이슈에 발이 묶이지 않게 하기 위함이다.
    --
    -- 서보를 실제로 움직이는 비행 상태머신(및 LoRa로 수신한 서보
    -- 잠금/해제/리셋 명령의 실행)만 initialized(=센서+서보+상태머신
    -- 전부 성공)에 게이팅한다.

    -- 1. ArduPilot AHRS/EKF 상태 갱신
    sensor.update()

    local current_sensor_data = sensor.get_data()
    local status_flags = sensor.get_status_flags()

    -- 2. LoRa 명령 수신 (버퍼링만 - 실제 실행은 flight.update() 안에서)
    lora.receive()

    -- 3. 비행 상태 및 낙하산 제어 갱신 (서보 제어 포함 - initialized일 때만)
    if initialized then
        flight.update(
            now,
            current_sensor_data
        )
    end

    -- 4. 갱신된 비행 상태와 센서 정보를 LoRa로 송신
    lora.send_telemetry_if_due(
        now,
        current_sensor_data,
        flight.get_state(),
        status_flags
    )

    -- 5. 재비행 준비를 위한 리셋 스위치 확인
    check_reset_switch(now)

    return update, CONFIG.UPDATE_PERIOD_MS
end


-- ---------------------------------------------------------------------
-- Script entry point (이 파일 전체에서 유일한 실행 시점의 최상위 코드)
-- ---------------------------------------------------------------------

initialized = startup()

return update, CONFIG.UPDATE_PERIOD_MS
