-- =====================================================================
-- Comet Sensor Interface
-- =====================================================================
-- Pixhawk의 개별 원시 센서를 직접 제어하거나 센서융합을 수행하지 않고,
-- ArduPilot AHRS/EKF가 제공하는 위치·속도·자세·가속도 상태를 읽어
-- 단위 변환과 유효성 검사를 수행한 뒤 비행 상태머신과 LoRa 모듈에
-- 공통 데이터 형식으로 제공한다.
-- =====================================================================

local M = {}

local RAD_TO_DEG = 180.0 / math.pi


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local data = {
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


-- ---------------------------------------------------------------------
-- Utility functions
-- ---------------------------------------------------------------------

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


-- ---------------------------------------------------------------------
-- Position
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
        data.position_valid = false
        return false
    end

    if not is_valid_number(latitude)
        or not is_valid_number(longitude)
        or not is_valid_number(altitude_m) then

        data.position_valid = false
        return false
    end

    data.latitude = latitude
    data.longitude = longitude
    data.altitude = altitude_m
    data.position_valid = true

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
    data.velocity_valid = false
    data.attitude_valid = false
    data.acceleration_valid = false

    gcs:send_text(6, "COMET: [SENSOR] Sensor interface initialized")
    return true
end


function M.update()
    update_position()
    update_velocity()
    update_attitude()
    update_acceleration()

    return true
end


function M.get_data()
    return data
end


function M.get_status_flags()
    local flags = 0

    -- bit 0: position valid
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

    return flags
end


function M.is_position_valid()
    return data.position_valid
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
        and data.velocity_valid
        and data.attitude_valid
        and data.acceleration_valid
end


return M