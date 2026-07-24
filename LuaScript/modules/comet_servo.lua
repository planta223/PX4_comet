-- =====================================================================
-- Servo (Parachute) Control
-- =====================================================================
-- comet_config.lua에 정의된 서보별 FUNCTION_ID와 PWM 설정을 이용하여
-- 낙하산 잠금 및 해제 동작을 수행한다.
--
-- 실제 PWM 출력은 ArduPilot의 SRV_Channels 인터페이스를 사용하며,
-- 개별 서보 제어와 전체 서보 동시 제어 기능을 제공한다.
-- =====================================================================

local CONFIG = require("comet_config")
local SERVO = CONFIG.SERVO

local M = {}


-- ---------------------------------------------------------------------
-- Utility functions
-- ---------------------------------------------------------------------

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


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize()
    -- 부팅 시 모든 낙하산 서보를 잠금 상태로 초기화한다.
    local ok = M.lock_all()

    if not ok then
        gcs:send_text(3, "COMET: Servo initialization failed")
        return false
    end

    gcs:send_text(6, "COMET: Servos initialized")
    return true
end


function M.lock(name)
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


function M.release(name)
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


function M.lock_all()
    -- 한 서보가 실패하더라도 다른 서보 제어는 시도한다.
    local servo1_ok = M.lock("SERVO1")
    local servo2_ok = M.lock("SERVO2")

    return servo1_ok and servo2_ok
end


function M.release_all()
    -- 한 서보가 실패하더라도 다른 서보 제어는 시도한다.
    local servo1_ok = M.release("SERVO1")
    local servo2_ok = M.release("SERVO2")

    return servo1_ok and servo2_ok
end


return M