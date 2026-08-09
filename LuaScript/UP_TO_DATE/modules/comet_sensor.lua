-- =====================================================================
-- Comet Sensor Interface
-- =====================================================================
-- Pixhawk의 개별 원시 센서를 직접 제어하거나 센서융합을 수행하지 않고,
-- ArduPilot AHRS/EKF가 제공하는 위치·속도·자세·가속도 상태와
-- Barometer의 압력값을 읽어 단위 변환과 유효성 검사를 수행한 뒤,
-- 비행 상태머신과 LoRa/SiK 모듈에 공통 데이터 형식으로 제공한다.
--
-- 고도는 baro:get_altitude()를 사용하지 않고 baro:get_pressure()를
-- 직접 읽는다. 부팅 후 초기 3초 동안 평균 압력을 기준압으로 저장하고,
-- 이후 현재 압력과 기준압의 비율로 상대 기압고도를 계산한다.
-- 이 방식은 Rover disarmed 상태의 ground-pressure 재보정 영향과
-- EKF HOME 설정에 독립적이다.
-- =====================================================================

local CONFIG = require("comet_config")
local DEBUG_CONFIG = CONFIG.DEBUG or {}

local M = {}

local RAD_TO_DEG = 180.0 / math.pi

-- 국제표준대기 근사식 상수
local BARO_ALTITUDE_SCALE_M = 44330.0
local BARO_ALTITUDE_EXPONENT = 0.190294957


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local data = {
    -- Position
    latitude = 0.0,           -- deg
    longitude = 0.0,          -- deg
    altitude = 0.0,           -- startup pressure reference-relative, m

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
    altitude_valid = false,
    velocity_valid = false,
    attitude_valid = false,
    acceleration_valid = false
}


local debug_state = {
    last_message_ms = 0
}


-- ---------------------------------------------------------------------
-- Barometer pressure reference
-- ---------------------------------------------------------------------

local BARO_INSTANCE = 0
local BARO_REFERENCE_DURATION_MS = 3000

local baro_reference_state = {
    start_ms = nil,
    pressure_sum_pa = 0.0,
    sample_count = 0,
    reference_pressure_pa = nil,
    message_sent = false
}


-- ---------------------------------------------------------------------
-- Utility functions
-- ---------------------------------------------------------------------

-- yaw를 -180 ~ +180 deg 범위로 정규화한다.
local function normalize_yaw_deg(yaw)
    while yaw > 180.0 do
        yaw = yaw - 360.0
    end

    while yaw < -180.0 do
        yaw = yaw + 360.0
    end

    return yaw
end


-- 숫자가 NaN인지 검사한다.
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


-- boolean 값을 메시지 출력용 숫자로 변환한다.
local function bool_to_int(value)
    if value then
        return 1
    end

    return 0
end


-- ---------------------------------------------------------------------
-- Horizontal position
-- ---------------------------------------------------------------------

local function update_position()
    local position = ahrs:get_position()

    if position == nil then
        data.position_valid = false
        return false
    end

    -- ArduPilot Location:
    -- lat() : deg × 1e7
    -- lng() : deg × 1e7
    local latitude = position:lat() / 10000000.0
    local longitude = position:lng() / 10000000.0

    if not is_valid_number(latitude)
        or not is_valid_number(longitude) then

        data.position_valid = false
        return false
    end

    data.latitude = latitude
    data.longitude = longitude
    data.position_valid = true

    return true
end


-- ---------------------------------------------------------------------
-- Barometric altitude
-- ---------------------------------------------------------------------

local function reset_baro_reference(now)
    baro_reference_state.start_ms = now
    baro_reference_state.pressure_sum_pa = 0.0
    baro_reference_state.sample_count = 0
    baro_reference_state.reference_pressure_pa = nil
    baro_reference_state.message_sent = false

    data.altitude = 0.0
    data.altitude_valid = false
end


local function update_altitude(now)
    if baro == nil then
        data.altitude_valid = false
        return false
    end

    -- 첫 번째 기압계 상태 확인
    if not baro:healthy(BARO_INSTANCE) then
        data.altitude_valid = false
        return false
    end

    local current_pressure_pa = baro:get_pressure()

    if not is_valid_number(current_pressure_pa)
        or current_pressure_pa <= 0.0 then

        data.altitude_valid = false
        return false
    end

    -- 초기 기준압 수집 시작
    if baro_reference_state.start_ms == nil then
        reset_baro_reference(now)
    end

    -- 초기 3초 동안 정상 압력 샘플 평균 계산
    if baro_reference_state.reference_pressure_pa == nil then
        baro_reference_state.pressure_sum_pa =
            baro_reference_state.pressure_sum_pa
            + current_pressure_pa

        baro_reference_state.sample_count =
            baro_reference_state.sample_count + 1

        data.altitude = 0.0
        data.altitude_valid = false

        local elapsed_ms =
            now - baro_reference_state.start_ms

        if elapsed_ms < BARO_REFERENCE_DURATION_MS then
            return false
        end

        if baro_reference_state.sample_count <= 0 then
            reset_baro_reference(now)
            return false
        end

        baro_reference_state.reference_pressure_pa =
            baro_reference_state.pressure_sum_pa
            / baro_reference_state.sample_count

        data.altitude = 0.0
        data.altitude_valid = true

        if not baro_reference_state.message_sent then
            gcs:send_text(
                6,
                string.format(
                    "COMET: [SENSOR] Baro pressure reference %.1f Pa, N=%d",
                    baro_reference_state.reference_pressure_pa,
                    baro_reference_state.sample_count
                )
            )

            baro_reference_state.message_sent = true
        end

        return true
    end

    -- 상대 기압고도 계산
    -- 상승하면 압력이 낮아지므로 결과는 양수가 된다.
    local pressure_ratio =
        current_pressure_pa
        / baro_reference_state.reference_pressure_pa

    if not is_valid_number(pressure_ratio)
        or pressure_ratio <= 0.0 then

        data.altitude_valid = false
        return false
    end

    local altitude_m =
        BARO_ALTITUDE_SCALE_M
        * (
            1.0
            - pressure_ratio ^ BARO_ALTITUDE_EXPONENT
        )

    if not is_valid_number(altitude_m) then
        data.altitude_valid = false
        return false
    end

    data.altitude = altitude_m
    data.altitude_valid = true

    return true
end


-- ---------------------------------------------------------------------
-- Velocity
-- ---------------------------------------------------------------------

local function update_velocity()
    local velocity = ahrs:get_velocity_NED()

    if velocity == nil then
        data.velocity_valid = false
        return false
    end

    if velocity:is_nan() or velocity:is_inf() then
        data.velocity_valid = false
        return false
    end

    data.velocity_n = velocity:x()
    data.velocity_e = velocity:y()
    data.velocity_d = velocity:z()

    data.velocity_valid = true
    return true
end


-- ---------------------------------------------------------------------
-- Attitude
-- ---------------------------------------------------------------------

local function update_attitude()
    local roll_rad = ahrs:get_roll()
    local pitch_rad = ahrs:get_pitch()
    local yaw_rad = ahrs:get_yaw()

    if not is_valid_number(roll_rad)
        or not is_valid_number(pitch_rad)
        or not is_valid_number(yaw_rad) then

        data.attitude_valid = false
        return false
    end

    data.roll = roll_rad * RAD_TO_DEG
    data.pitch = pitch_rad * RAD_TO_DEG
    data.yaw = normalize_yaw_deg(yaw_rad * RAD_TO_DEG)

    data.attitude_valid = true
    return true
end


-- ---------------------------------------------------------------------
-- Acceleration
-- ---------------------------------------------------------------------

local function update_acceleration()
    local acceleration = ahrs:get_accel()

    if acceleration == nil then
        data.acceleration_valid = false
        return false
    end

    if acceleration:is_nan() or acceleration:is_inf() then
        data.acceleration_valid = false
        return false
    end

    local total_acceleration = acceleration:length()

    if not is_valid_number(total_acceleration) then
        data.acceleration_valid = false
        return false
    end

    data.total_acceleration = total_acceleration
    data.acceleration_valid = true

    return true
end


-- ---------------------------------------------------------------------
-- Debug messages
-- ---------------------------------------------------------------------

local function send_debug_messages()
    gcs:send_text(
        6,
        string.format(
            "COMET: [SDBG] ALT=%.2f VN=%.2f VE=%.2f VD=%.2f",
            data.altitude,
            data.velocity_n,
            data.velocity_e,
            data.velocity_d
        )
    )

    gcs:send_text(
        6,
        string.format(
            "COMET: [SDBG] R=%.1f P=%.1f Y=%.1f ACC=%.2f",
            data.roll,
            data.pitch,
            data.yaw,
            data.total_acceleration
        )
    )

    gcs:send_text(
        6,
        string.format(
            "COMET: [SDBG] VALID POS=%d ALT=%d VEL=%d ATT=%d ACC=%d",
            bool_to_int(data.position_valid),
            bool_to_int(data.altitude_valid),
            bool_to_int(data.velocity_valid),
            bool_to_int(data.attitude_valid),
            bool_to_int(data.acceleration_valid)
        )
    )
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize()
    data.latitude = 0.0
    data.longitude = 0.0
    data.altitude = 0.0

    data.velocity_n = 0.0
    data.velocity_e = 0.0
    data.velocity_d = 0.0

    data.roll = 0.0
    data.pitch = 0.0
    data.yaw = 0.0

    data.total_acceleration = 0.0

    data.position_valid = false
    data.altitude_valid = false
    data.velocity_valid = false
    data.attitude_valid = false
    data.acceleration_valid = false

    baro_reference_state.start_ms = nil
    baro_reference_state.pressure_sum_pa = 0.0
    baro_reference_state.sample_count = 0
    baro_reference_state.reference_pressure_pa = nil
    baro_reference_state.message_sent = false

    debug_state.last_message_ms = 0

    gcs:send_text(
        6,
        "COMET: [SENSOR] Sensor interface initialized"
    )

    return true
end


function M.update(now)
    if now == nil then
        now = millis():toint()
    end

    update_position()
    update_altitude(now)
    update_velocity()
    update_attitude()
    update_acceleration()

    return true
end


function M.send_debug_if_due(now)
    if DEBUG_CONFIG.ENABLE ~= true then
        return true
    end

    if DEBUG_CONFIG.SENSOR_MESSAGE_ENABLE ~= true then
        return true
    end

    local period_ms =
        DEBUG_CONFIG.SENSOR_MESSAGE_PERIOD_MS or 500

    if now - debug_state.last_message_ms < period_ms then
        return true
    end

    debug_state.last_message_ms = now

    send_debug_messages()

    return true
end


function M.get_data()
    return data
end


function M.get_status_flags()
    local flags = 0

    -- bit 0: horizontal position valid
    if data.position_valid then
        flags = flags | 0x01
    end

    -- bit 1: velocity valid
    if data.velocity_valid then
        flags = flags | 0x02
    end

    -- bit 2: attitude valid
    if data.attitude_valid then
        flags = flags | 0x04
    end

    -- bit 3: acceleration valid
    if data.acceleration_valid then
        flags = flags | 0x08
    end

    -- bit 4: barometric altitude valid
    if data.altitude_valid then
        flags = flags | 0x10
    end

    return flags
end


function M.is_position_valid()
    return data.position_valid
end


function M.is_altitude_valid()
    return data.altitude_valid
end


function M.is_velocity_valid()
    return data.velocity_valid
end


function M.is_attitude_valid()
    return data.attitude_valid
end


function M.is_acceleration_valid()
    return data.acceleration_valid
end


function M.is_all_valid()
    return data.position_valid
        and data.altitude_valid
        and data.velocity_valid
        and data.attitude_valid
        and data.acceleration_valid
end


-- 지상 재시험 시 기압 기준을 다시 설정하고 싶을 때 사용한다.
-- 비행 중에는 호출하지 않는다.
function M.reset_altitude_reference(now)
    if now == nil then
        now = millis():toint()
    end

    reset_baro_reference(now)

    gcs:send_text(
        6,
        "COMET: [SENSOR] Baro pressure reference reset"
    )

    return true
end


return M
