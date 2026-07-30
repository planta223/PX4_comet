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

-- gcs/shared_state.py의 SharedOrigin._EARTH_METERS_PER_DEG와 동일한 근사값 -
-- HOME 기준 north/east 오프셋 계산에 쓴다(equirectangular 근사, 로켓 회수
-- 반경 정도의 거리에서는 정밀도 손실이 무시할 수준).
local EARTH_METERS_PER_DEG = 111320.0


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local data = {
    -- Position (HOME 기준 평면 근사 오프셋 - 절대 위경도 대신 이걸 쓰는
    -- 이유는 comet_lora.lua 상단 주석 참고)
    north = 0.0,              -- HOME 기준 북쪽 오프셋, m
    east = 0.0,               -- HOME 기준 동쪽 오프셋, m
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

    local home = ahrs:get_home()

    if home == nil then
        -- HOME이 없으면 상대고도/상대위치(north/east) 둘 다 계산 불가
        data.position_valid = false
        return false
    end

    -- ArduPilot Location:
    -- lat() : deg × 1e7
    -- lng() : deg × 1e7
    -- alt() : cm
    local latitude = position:lat() / 10000000.0
    local longitude = position:lng() / 10000000.0
    local home_latitude = home:lat() / 10000000.0
    local home_longitude = home:lng() / 10000000.0

    -- 현재 위치와 HOME의 고도 차이를 발사점 기준 상대고도로 사용
    local altitude_m = (position:alt() - home:alt()) / 100.0

    -- HOME 기준 평면(equirectangular) 근사로 north/east 오프셋(m) 계산
    local north_m = (latitude - home_latitude) * EARTH_METERS_PER_DEG
    local east_m =
        (longitude - home_longitude)
            * EARTH_METERS_PER_DEG
            * math.cos(math.rad(home_latitude))

    if not is_valid_number(north_m)
        or not is_valid_number(east_m)
        or not is_valid_number(altitude_m) then

        data.position_valid = false
        return false
    end

    data.north = north_m
    data.east = east_m
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
    data.north = 0.0
    data.east = 0.0
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

    gcs:send_text(6, "COMET: Sensor interface initialized")
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
