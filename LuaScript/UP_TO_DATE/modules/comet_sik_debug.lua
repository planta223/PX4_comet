-- =====================================================================
-- SiK MAVLink Debug Telemetry
-- =====================================================================
-- Lua 상태머신이 실제로 사용하는 센서값과 상태를
-- MAVLink NAMED_VALUE_FLOAT 메시지로 GCS/SiK 링크에 전송한다.
--
-- C_ALT:
-- 부팅 후 초기 3초 평균 기압을 기준으로 계산한 상대 기압고도이며,
-- 위쪽이 양수이다.
-- =====================================================================

local CONFIG = require("comet_config")

local DEBUG_CONFIG = CONFIG.DEBUG or {}

local M = {}


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local last_send_ms = 0


-- ---------------------------------------------------------------------
-- Utility
-- ---------------------------------------------------------------------

local function bool_to_number(value)
    if value then
        return 1.0
    end

    return 0.0
end


local function get_safe_value(value)
    if value == nil
        or value ~= value
        or value == math.huge
        or value == -math.huge then

        return 0.0
    end

    return value
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize()
    last_send_ms = 0

    if DEBUG_CONFIG.ENABLE == true
        and DEBUG_CONFIG.SIK_DEBUG_ENABLE == true then

        gcs:send_text(
            6,
            "COMET: [SIK DEBUG] Enabled"
        )
    else
        gcs:send_text(
            6,
            "COMET: [SIK DEBUG] Disabled"
        )
    end

    return true
end


function M.update(
    now,
    sensor_data,
    flight_state,
    status_flags,
    fault_flags,
    maximum_altitude,
    parachute_deployed
)
    if DEBUG_CONFIG.ENABLE ~= true then
        return true
    end

    if DEBUG_CONFIG.SIK_DEBUG_ENABLE ~= true then
        return true
    end

    local period_ms =
        DEBUG_CONFIG.SIK_DEBUG_PERIOD_MS or 200

    if now - last_send_ms < period_ms then
        return true
    end

    last_send_ms = now

    if sensor_data == nil then
        return false
    end

    -- 상태 전이에 직접 사용되는 센서값
    gcs:send_named_float(
        "C_ACC",
        get_safe_value(sensor_data.total_acceleration)
    )

    -- 초기 평균 기압 기준 상대고도, 위쪽 양수
    gcs:send_named_float(
        "C_ALT",
        get_safe_value(sensor_data.altitude)
    )

    -- NED 수직속도, 아래쪽 양수
    gcs:send_named_float(
        "C_VD",
        get_safe_value(sensor_data.velocity_d)
    )

    -- 비행 상태 및 센서 유효성
    gcs:send_named_float(
        "C_STATE",
        get_safe_value(flight_state)
    )

    gcs:send_named_float(
        "C_VALID",
        get_safe_value(status_flags)
    )

    gcs:send_named_float(
        "C_FAULT",
        get_safe_value(fault_flags)
    )

    -- 보조 상태
    gcs:send_named_float(
        "C_MAXALT",
        get_safe_value(maximum_altitude)
    )

    gcs:send_named_float(
        "C_DEPLOY",
        bool_to_number(parachute_deployed)
    )

    return true
end


return M
