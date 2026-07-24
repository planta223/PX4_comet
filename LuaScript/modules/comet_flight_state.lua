-- =====================================================================
-- Comet Flight State Machine
-- =====================================================================
-- 센서 모듈이 제공하는 위치·속도·자세·가속도 정보를 이용하여
-- READY, ASCENT, APOGEE, DESCENT, LANDED, ERROR 상태를 관리한다.
--
-- 또한 상태 전이 조건을 판단하고, 정점 또는 비상 상황에서 낙하산
-- 사출을 실행하며, LoRa로 수신한 서보 명령의 허용 여부를 결정한다.
-- =====================================================================

local CONFIG = require("comet_config")
local servo = require("comet_servo")
local lora = require("comet_lora")

local SAFETY = CONFIG.SAFETY
local TRANSITION = CONFIG.TRANSITION

local READY_TO_ASCENT = TRANSITION.READY_TO_ASCENT
local ASCENT_TO_APOGEE = TRANSITION.ASCENT_TO_APOGEE
local APOGEE_TO_DESCENT = TRANSITION.APOGEE_TO_DESCENT
local DESCENT_TO_LANDED = TRANSITION.DESCENT_TO_LANDED

local M = {}


-- ---------------------------------------------------------------------
-- Flight states
-- ---------------------------------------------------------------------

M.STATE = {
    READY = 0,
    ASCENT = 1,
    APOGEE = 2,
    DESCENT = 3,
    LANDED = 4,
    ERROR = 5
}


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local current_state = M.STATE.READY

local launch_time_ms = 0
local state_enter_time_ms = 0
local last_sensor_time_ms = 0

local maximum_altitude_m = 0.0

local launch_confirm_count = 0
local apogee_confirm_count = 0
local landing_confirm_count = 0

local parachute_deployed = false

-- ---------------------------------------------------------------------
-- Utility
-- ---------------------------------------------------------------------

local function change_state(new_state, now)
    if current_state == new_state then
        return
    end

    -- 유효성 검사
    if new_state < M.STATE.READY
        or new_state > M.STATE.ERROR then

        gcs:send_text(3, "COMET: Invalid flight state")
        return
    end

    current_state = new_state
    state_enter_time_ms = now

    if new_state == M.STATE.READY then
        gcs:send_text(6, "COMET: State READY")

    elseif new_state == M.STATE.ASCENT then
        gcs:send_text(6, "COMET: State ASCENT")

    elseif new_state == M.STATE.APOGEE then
        gcs:send_text(6, "COMET: State APOGEE")

    elseif new_state == M.STATE.DESCENT then
        gcs:send_text(6, "COMET: State DESCENT")

    elseif new_state == M.STATE.LANDED then
        gcs:send_text(6, "COMET: State LANDED")

    elseif new_state == M.STATE.ERROR then
        gcs:send_text(3, "COMET: State ERROR")
    end
end


local function reset_state(now)
    current_state = M.STATE.READY

    launch_time_ms = 0
    state_enter_time_ms = now
    last_sensor_time_ms = now

    maximum_altitude_m = 0.0

    launch_confirm_count = 0
    apogee_confirm_count = 0
    landing_confirm_count = 0

    parachute_deployed = false
end


-- ---------------------------------------------------------------------
-- Parachute deployment
-- ---------------------------------------------------------------------

local function deploy_parachute()
    if parachute_deployed then
        return true
    end

    local ok = servo.release_all()

    if not ok then
        gcs:send_text(3, "COMET: Parachute deployment failed")
        return false
    end

    parachute_deployed = true
    gcs:send_text(6, "COMET: Parachute deployed")

    return true
end


-- ---------------------------------------------------------------------
-- LoRa command processing
-- ---------------------------------------------------------------------

local function execute_command(command)
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
    end

    return false
end


local function is_command_allowed(command)
    -- 비행 중 서보 잠금은 금지
    if command == lora.COMMAND.SERVO1_LOCK
        or command == lora.COMMAND.SERVO2_LOCK
        or command == lora.COMMAND.LOCK_ALL then

        return current_state == M.STATE.READY
            or current_state == M.STATE.LANDED
    end

    -- READY 상태에서는 오작동 방지를 위해 원격 사출 금지
    if command == lora.COMMAND.SERVO1_RELEASE
        or command == lora.COMMAND.SERVO2_RELEASE
        or command == lora.COMMAND.RELEASE_ALL then

        return current_state == M.STATE.APOGEE
            or current_state == M.STATE.DESCENT
            or current_state == M.STATE.ERROR
    end

    return false
end


local function process_lora_command()
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

    local ok = execute_command(command)

    if ok then
        -- 서보 1, 2 모두 RELEASE 상태일 때 낙하산 전개 판단
        if command == lora.COMMAND.RELEASE_ALL then
            parachute_deployed = true
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


-- ---------------------------------------------------------------------
-- Sensor validity
-- ---------------------------------------------------------------------

-- READY 상태에서는 가속도값 이용
local function ready_sensor_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.acceleration_valid
end

-- ASCENT 상태에서는 위치·속도값 이용
local function ascent_sensor_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.position_valid
        and sensor_data.velocity_valid
end

-- DESCENT 상태에서는 위치·속도, 가속도 값 이용
local function descent_sensor_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.position_valid
        and sensor_data.velocity_valid
        and sensor_data.acceleration_valid
end

-- ---------------------------------------------------------------------
-- State handlers
-- ---------------------------------------------------------------------
-- 상세한 사항은 flight_state_description.txt 참고

local function update_ready(now, sensor_data)
    if not ready_sensor_is_valid(sensor_data) then
        launch_confirm_count = 0
        return
    end

    if sensor_data.total_acceleration
        >= READY_TO_ASCENT.ACCEL_THRESHOLD_MPS2 then

        launch_confirm_count = launch_confirm_count + 1
    else
        launch_confirm_count = 0
    end

    if launch_confirm_count >= READY_TO_ASCENT.CONFIRM_COUNT then
        launch_time_ms = now
        last_sensor_time_ms = now

        if sensor_data.position_valid then
            maximum_altitude_m = sensor_data.altitude
        else
            maximum_altitude_m = 0.0
        end

        launch_confirm_count = 0
        apogee_confirm_count = 0
        landing_confirm_count = 0

        change_state(M.STATE.ASCENT, now)
    end
end

local function update_ascent(now, sensor_data)
    if not ascent_sensor_is_valid(sensor_data) then
        return
    end

    if sensor_data.altitude > maximum_altitude_m then
        maximum_altitude_m = sensor_data.altitude
        apogee_confirm_count = 0
    end

    local minimum_time_passed =
        now - launch_time_ms
            >= ASCENT_TO_APOGEE.MIN_TIME_AFTER_LAUNCH_MS

    local descending =
        sensor_data.velocity_d
            >= ASCENT_TO_APOGEE.MIN_DESCENT_SPEED_MPS

    local altitude_dropped =
        sensor_data.altitude
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

        change_state(M.STATE.APOGEE, now)
    end
end


local function update_apogee(now)
    if not parachute_deployed then
        local ok = deploy_parachute()

        if not ok then
            change_state(M.STATE.ERROR, now)
            return
        end
    end

    if now - state_enter_time_ms
        >= APOGEE_TO_DESCENT.MIN_HOLD_TIME_MS then

        change_state(M.STATE.DESCENT, now)
    end
end


local function update_descent(now, sensor_data)
    if not descent_sensor_is_valid(sensor_data) then
        landing_confirm_count = 0
        return
    end

    local altitude_low =
        sensor_data.altitude
            <= DESCENT_TO_LANDED.MAX_RELATIVE_ALTITUDE_M

    local vertical_speed_low =
        math.abs(sensor_data.velocity_d)
            <= DESCENT_TO_LANDED.MAX_ABS_VERTICAL_SPEED_MPS

    local acceleration_stable =
        sensor_data.total_acceleration
            >= DESCENT_TO_LANDED.ACCEL_MIN_MPS2
        and sensor_data.total_acceleration
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

        change_state(M.STATE.LANDED, now)
    end
end


local function update_error()
    if not parachute_deployed then
        deploy_parachute()
    end
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize(now)
    reset_state(now or 0)

    gcs:send_text(6, "COMET: Flight state initialized")
    return true
end


function M.update(now, sensor_data)
    -- LoRa 명령은 센서 상태와 관계없이 우선 처리
    process_lora_command()

    if current_state == M.STATE.READY then
        update_ready(now, sensor_data)

    elseif current_state == M.STATE.ASCENT then
        if ascent_sensor_is_valid(sensor_data) then
            last_sensor_time_ms = now
            update_ascent(now, sensor_data)

        elseif now - last_sensor_time_ms
            >= SAFETY.SENSOR_TIMEOUT_MS then

            change_state(M.STATE.ERROR, now)
        end

    elseif current_state == M.STATE.APOGEE then
        update_apogee(now)

    elseif current_state == M.STATE.DESCENT then
        update_descent(now, sensor_data)

    elseif current_state == M.STATE.ERROR then
        update_error()
    end

    return true
end


function M.get_state()
    return current_state
end


function M.get_maximum_altitude()
    return maximum_altitude_m
end


function M.is_parachute_deployed()
    return parachute_deployed
end


function M.reset(now)
    if current_state ~= M.STATE.READY
        and current_state ~= M.STATE.LANDED then

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


return M