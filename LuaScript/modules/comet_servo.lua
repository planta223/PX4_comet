-- =====================================================================
-- Servo (Parachute) Control
-- =====================================================================
-- comet_config.lua에 정의된 서보별 FUNCTION_ID와 PWM 설정을 이용하여
-- 낙하산 잠금 및 해제 동작을 수행한다.
--
-- 실제 PWM 출력은 ArduPilot의 SRV_Channels 인터페이스를 사용하며,
-- ENABLED=true인 서보만 초기화 및 제어한다.
-- lock_all/release_all은 설정된 모든 서보가 아니라 활성화된 서보만 처리한다.
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
        gcs:send_text(3, "COMET: [SERVO] Invalid servo name")
        return nil
    end

    return config
end


-- 지정된 서보가 활성화되어 있는지 확인한다.
local function is_servo_enabled(name, config)
    if config.ENABLED ~= true then
        gcs:send_text(4, "COMET: [SERVO] " .. name .. " is disabled")
        return false
    end

    return true
end


-- 지정된 서보 기능에 PWM 값을 출력한다.
local function set_pwm(name, config, pwm)
    if type(pwm) ~= "number" then
        gcs:send_text(3, "COMET: [SERVO] Invalid " .. name .. " PWM")
        return false
    end

    -- FUNCTION_ID가 실제 출력 채널에 할당되어 있는지 확인한다.
    local channel =
        SRV_Channels:find_channel(config.FUNCTION_ID)

    if channel == nil then
        gcs:send_text(
            3,
            "COMET: [SERVO] " .. name .. " function not assigned"
        )
        return false
    end

    SRV_Channels:set_output_pwm(
        config.FUNCTION_ID,
        pwm
    )

    return true
end


-- 활성화된 서보에 동일한 동작을 수행한다.
local function apply_to_enabled_servos(action)
    local enabled_count = 0
    local all_ok = true

    -- MAIN 설정이 존재하고 활성화된 경우에만 처리한다.
    if SERVO.SERVO_MAIN ~= nil
        and SERVO.SERVO_MAIN.ENABLED == true then

        enabled_count = enabled_count + 1

        if not action("SERVO_MAIN") then
            all_ok = false
        end
    end

    -- CANDIDATE 설정이 존재하고 활성화된 경우에만 처리한다.
    if SERVO.SERVO_CANDIDATE ~= nil
        and SERVO.SERVO_CANDIDATE.ENABLED == true then

        enabled_count = enabled_count + 1

        if not action("SERVO_CANDIDATE") then
            all_ok = false
        end
    end

    if enabled_count == 0 then
        gcs:send_text(3, "COMET: [SERVO] No enabled servo")
        return false
    end

    return all_ok
end

-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize()
    -- 부팅 시 활성화된 모든 낙하산 서보를 잠금 상태로 초기화한다.
    local ok = M.lock_all()

    if not ok then
        gcs:send_text(3, "COMET: [SERVO] Servo initialization failed")
        return false
    end

    gcs:send_text(6, "COMET: [SERVO] Servos initialized")
    return true
end


function M.lock(name)
    local config = get_servo_config(name)

    if config == nil then
        return false
    end

    if not is_servo_enabled(name, config) then
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

    if not is_servo_enabled(name, config) then
        return false
    end

    return set_pwm(
        name,
        config,
        config.RELEASE_PWM
    )
end


function M.lock_all()
    return apply_to_enabled_servos(M.lock)
end


function M.release_all()
    return apply_to_enabled_servos(M.release)
end


return M