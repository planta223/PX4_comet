-- =====================================================================
-- Comet Flight State Machine
-- =====================================================================
-- READY -> ASCENT -> DEPLOY -> DESCENT -> LANDED
--
-- 비행 상태와 시스템 이상을 분리한다.
-- 상태는 실제 비행 단계를 나타내며, 이상은 fault_flags로 관리한다.
--
-- 원격 명령은 LoRa/MAVLink 어느 경로로 들어왔든 comet_command_queue.lua의
-- 공통 FIFO 큐를 거치며, 여기서는 그 큐만 소비한다 (process_command()).
-- =====================================================================

local CONFIG = require("comet_config")
local servo = require("comet_servo")
local lora = require("comet_lora")
local mavlink = require("comet_mavlink")
local queue = require("comet_command_queue")

local SERVO_CONFIG = CONFIG.SERVO
local TRANSITION = CONFIG.TRANSITION

local READY_TO_ASCENT = TRANSITION.READY_TO_ASCENT
local ASCENT_TO_DEPLOY = TRANSITION.ASCENT_TO_DEPLOY
local DEPLOY_TO_DESCENT = TRANSITION.DEPLOY_TO_DESCENT
local DESCENT_TO_LANDED = TRANSITION.DESCENT_TO_LANDED

local M = {}


-- ---------------------------------------------------------------------
-- Flight states / Fault flags
-- ---------------------------------------------------------------------

M.STATE = {
    READY = 0,
    ASCENT = 1,
    DEPLOY = 2,
    DESCENT = 3,
    LANDED = 4
}

M.FAULT = {
    NONE                  = 0x00,
    ACCEL_SENSOR_INVALID  = 0x01,
    POSITION_INVALID      = 0x02,
    VELOCITY_INVALID      = 0x04,
    ASCENT_TIMEOUT        = 0x08,
    DEPLOY_COMMAND_FAILED = 0x10,
    LORA_UNAVAILABLE      = 0x20
}


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local current_state = M.STATE.READY
local fault_flags = M.FAULT.NONE

local state_enter_time_ms = 0
local launch_time_ms = 0
local maximum_altitude_m = 0.0

local launch_confirm_count = 0
local accel_valid_start_ms = nil
local accel_lost_start_ms = nil
local accel_was_stable = false
local launch_detection_message_sent = false

local descent_speed_confirm_count = 0
local descent_detection_message_sent = false

local deploy_attempted = false
local deploy_command_succeeded = false
local deploy_command_time_ms = 0
local parachute_deployed = false
local deploy_retry_count = 0
local last_deploy_attempt_ms = 0

local landed_confirm_count = 0
local landing_detection_message_sent = false


-- ---------------------------------------------------------------------
-- Utility
-- ---------------------------------------------------------------------

local function state_name(state)
    if state == M.STATE.READY then
        return "READY"
    elseif state == M.STATE.ASCENT then
        return "ASCENT"
    elseif state == M.STATE.DEPLOY then
        return "DEPLOY"
    elseif state == M.STATE.DESCENT then
        return "DESCENT"
    elseif state == M.STATE.LANDED then
        return "LANDED"
    end

    return "UNKNOWN"
end


local function set_fault(flag, message)
    if (fault_flags & flag) ~= 0 then
        return
    end

    fault_flags = fault_flags | flag

    if message ~= nil then
        gcs:send_text(3, "COMET: Fault - " .. message)
    end
end


local function clear_fault(flag, message)
    if (fault_flags & flag) == 0 then
        return
    end

    fault_flags = fault_flags & (~flag)

    if message ~= nil then
        gcs:send_text(
            6,
            "COMET: Fault cleared - " .. message
        )
    end
end


local function change_state(new_state, now, reason)
    if current_state == new_state then
        return true
    end

    if new_state < M.STATE.READY
        or new_state > M.STATE.LANDED then

        gcs:send_text(3, "COMET: Invalid flight state")
        return false
    end

    local old_state = current_state

    current_state = new_state
    state_enter_time_ms = now

    gcs:send_text(
        6,
        "COMET: "
            .. state_name(old_state)
            .. " -> "
            .. state_name(new_state)
    )

    if reason ~= nil then
        gcs:send_text(6, "COMET: Reason - " .. reason)
    end

    return true
end


local function enter_ascent(now, sensor_data, reason)
    launch_time_ms = now

    launch_confirm_count = 0
    descent_speed_confirm_count = 0
    landed_confirm_count = 0

    launch_detection_message_sent = false
    descent_detection_message_sent = false
    landing_detection_message_sent = false

    if sensor_data ~= nil
        and sensor_data.position_valid then

        maximum_altitude_m = sensor_data.altitude
    else
        maximum_altitude_m = 0.0
    end

    return change_state(M.STATE.ASCENT, now, reason)
end


local function reset_state(now)
    current_state = M.STATE.READY
    fault_flags = M.FAULT.NONE

    state_enter_time_ms = now
    launch_time_ms = 0
    maximum_altitude_m = 0.0

    launch_confirm_count = 0
    accel_valid_start_ms = nil
    accel_lost_start_ms = nil
    accel_was_stable = false
    launch_detection_message_sent = false

    descent_speed_confirm_count = 0
    descent_detection_message_sent = false

    deploy_attempted = false
    deploy_command_succeeded = false
    deploy_command_time_ms = 0
    parachute_deployed = false
    deploy_retry_count = 0
    last_deploy_attempt_ms = 0

    landed_confirm_count = 0
    landing_detection_message_sent = false
end


-- ---------------------------------------------------------------------
-- Parachute deployment
-- ---------------------------------------------------------------------

local function deploy_parachute(now, reason)
    if deploy_command_succeeded then
        return true
    end

    last_deploy_attempt_ms = now

    gcs:send_text(
        6,
        "COMET: Deploy attempt "
            .. tostring(deploy_retry_count + 1)
    )

    local ok = servo.release_all()

    if not ok then
        set_fault(
            M.FAULT.DEPLOY_COMMAND_FAILED,
            "deploy command failed"
        )

        gcs:send_text(3, "COMET: Deploy attempt failed")
        return false
    end

    deploy_command_succeeded = true
    deploy_command_time_ms = now
    parachute_deployed = true

    clear_fault(
        M.FAULT.DEPLOY_COMMAND_FAILED,
        "deploy command recovered"
    )

    gcs:send_text(6, "COMET: Deploy command succeeded")

    if reason ~= nil then
        gcs:send_text(6, "COMET: Deploy - " .. reason)
    end

    return true
end


-- ---------------------------------------------------------------------
-- Remote command processing (LoRa/MAVLink 공통 큐 소비)
-- ---------------------------------------------------------------------

local function send_result(source, sequence, command, result)
    if source == "MAVLINK" then
        return mavlink.send_result(sequence, command, result)
    end

    return lora.send_result(sequence, command, result)
end

local function execute_command(now, sensor_data, command)
    if command == lora.COMMAND.SERVO_MAIN_LOCK then
        return servo.lock("SERVO_MAIN")

    elseif command == lora.COMMAND.SERVO_MAIN_RELEASE then
        return servo.release("SERVO_MAIN")

    elseif command == lora.COMMAND.SERVO_CANDIDATE_LOCK then
        return servo.lock("SERVO_CANDIDATE")

    elseif command == lora.COMMAND.SERVO_CANDIDATE_RELEASE then
        return servo.release("SERVO_CANDIDATE")

    elseif command == lora.COMMAND.LOCK_ALL then
        return servo.lock_all()

    elseif command == lora.COMMAND.RELEASE_ALL then
        return servo.release_all()

    -- 원격 강제 상태전이 - 정상 전이 조건을 건너뛰고 다음 상태로 바로
    -- 진입한다. 실제 동작(서보 사출 등)은 각 상태의 update_* 핸들러가
    -- 다음 루프 틱에서 정상 전이와 동일하게 수행한다.
    elseif command == lora.COMMAND.FORCE_ASCENT then
        return enter_ascent(now, sensor_data, "remote forced transition")

    elseif command == lora.COMMAND.FORCE_DEPLOY then
        return change_state(M.STATE.DEPLOY, now, "remote forced transition")

    elseif command == lora.COMMAND.FORCE_DESCENT then
        return change_state(M.STATE.DESCENT, now, "remote forced transition")

    elseif command == lora.COMMAND.FORCE_LANDED then
        return change_state(M.STATE.LANDED, now, "remote forced transition")

    elseif command == lora.COMMAND.RESET then
        return M.reset(now)
    end

    return false
end


local function is_release_command(command)
    return command == lora.COMMAND.SERVO_MAIN_RELEASE
        or command == lora.COMMAND.SERVO_CANDIDATE_RELEASE
        or command == lora.COMMAND.RELEASE_ALL
end


local function is_main_release_command(command)
    return command == lora.COMMAND.SERVO_MAIN_RELEASE
        or command == lora.COMMAND.RELEASE_ALL
end


local function is_lock_command(command)
    return command == lora.COMMAND.SERVO_MAIN_LOCK
        or command == lora.COMMAND.SERVO_CANDIDATE_LOCK
        or command == lora.COMMAND.LOCK_ALL
end


local function is_command_allowed(command)
    if command == lora.COMMAND.SERVO_MAIN_LOCK
        or command == lora.COMMAND.SERVO_MAIN_RELEASE then

        if SERVO_CONFIG.SERVO_MAIN == nil
            or SERVO_CONFIG.SERVO_MAIN.ENABLED ~= true then

            return false
        end
    end

    if command == lora.COMMAND.SERVO_CANDIDATE_LOCK
        or command == lora.COMMAND.SERVO_CANDIDATE_RELEASE then

        if SERVO_CONFIG.SERVO_CANDIDATE == nil
            or SERVO_CONFIG.SERVO_CANDIDATE.ENABLED ~= true then

            return false
        end
    end

    if is_lock_command(command) then
        return current_state == M.STATE.READY
            or current_state == M.STATE.LANDED
    end

    if is_release_command(command) then
        return current_state == M.STATE.ASCENT
            or current_state == M.STATE.DEPLOY
            or current_state == M.STATE.DESCENT
    end

    -- 각 강제전이는 정상 전이 그래프의 바로 이전 상태에서만 허용한다
    -- (예: FORCE_DEPLOY는 ASCENT에서만 - DEPLOY/DESCENT/LANDED에서는 거부).
    if command == lora.COMMAND.FORCE_ASCENT then
        return current_state == M.STATE.READY
    end

    if command == lora.COMMAND.FORCE_DEPLOY then
        return current_state == M.STATE.ASCENT
    end

    if command == lora.COMMAND.FORCE_DESCENT then
        return current_state == M.STATE.DEPLOY
    end

    if command == lora.COMMAND.FORCE_LANDED then
        return current_state == M.STATE.DESCENT
    end

    if command == lora.COMMAND.RESET then
        return current_state == M.STATE.READY
            or current_state == M.STATE.LANDED
    end

    return false
end


-- 큐에서 한 번에 하나씩 꺼내 처리한다. 연속 명령(FORCE_ASCENT ->
-- FORCE_DEPLOY -> FORCE_LANDED 등)이 같은 틱에 여러 개 쌓여 있어도 유실
-- 없이 순서대로 처리되도록, 매 M.update() 호출마다 큐를 비울 때까지
-- 반복한다 (한 틱에 몰려 들어온 정상적 다단계 전이 명령을 다음 틱까지
-- 미루지 않기 위함).
local function process_command(now, sensor_data)
    while not queue.is_empty() do
        local received = queue.pop()

        local source = received.source
        local sequence = received.sequence
        local command = received.command

        gcs:send_text(
            6,
            "COMET: Command executing ("
                .. tostring(source) .. " seq="
                .. tostring(sequence) .. " cmd="
                .. tostring(command) .. ")"
        )

        if not is_command_allowed(command) then
            gcs:send_text(4, "COMET: Remote command rejected")

            send_result(
                source,
                sequence,
                command,
                lora.ACK_RESULT.NOT_ALLOWED
            )

            gcs:send_text(
                6,
                "COMET: Command completed (result=NOT_ALLOWED)"
            )

            goto continue
        end

        local ok = execute_command(now, sensor_data, command)

        if not ok then
            gcs:send_text(3, "COMET: Remote command failed")

            if is_main_release_command(command) then
                set_fault(
                    M.FAULT.DEPLOY_COMMAND_FAILED,
                    "remote deploy failed"
                )
            end

            send_result(
                source,
                sequence,
                command,
                lora.ACK_RESULT.EXECUTION_FAILED
            )

            gcs:send_text(
                6,
                "COMET: Command completed (result=EXECUTION_FAILED)"
            )

            goto continue
        end

        gcs:send_text(6, "COMET: Remote command succeeded")

        if is_main_release_command(command) then
            parachute_deployed = true
            deploy_attempted = true
            deploy_command_succeeded = true
            deploy_command_time_ms = now

            clear_fault(M.FAULT.DEPLOY_COMMAND_FAILED)

            if current_state == M.STATE.ASCENT then
                change_state(
                    M.STATE.DEPLOY,
                    now,
                    "remote deploy command"
                )
            end
        end

        if command == lora.COMMAND.SERVO_MAIN_LOCK
            or command == lora.COMMAND.LOCK_ALL then

            parachute_deployed = false
        end

        send_result(
            source,
            sequence,
            command,
            lora.ACK_RESULT.OK
        )

        gcs:send_text(6, "COMET: Command completed (result=OK)")

        ::continue::
    end
end


-- ---------------------------------------------------------------------
-- Sensor validity
-- ---------------------------------------------------------------------

local function acceleration_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.acceleration_valid
end


local function position_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.position_valid
end


local function velocity_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.velocity_valid
end


-- ---------------------------------------------------------------------
-- Fault management
-- ---------------------------------------------------------------------
-- 상태(current_state)와 무관하게 매 틱 독립적으로 판단한다 - 상태 전이
-- 로직(state handlers)은 "다음 상태로 넘어갈지"만 결정하고, fault_flags가
-- "지금 뭐가 이상한지"를 결정하는 건 여기 한 곳으로 모은다(모듈 상단
-- 설명 참고). 상태 전이 핸들러 쪽에서 개별로 set_fault/clear_fault를
-- 부르지 않는다.

local function update_faults(now, sensor_data)
    if acceleration_is_valid(sensor_data) then
        clear_fault(M.FAULT.ACCEL_SENSOR_INVALID, "accel recovered")
    else
        set_fault(M.FAULT.ACCEL_SENSOR_INVALID, "accel invalid")
    end

    if position_is_valid(sensor_data) then
        clear_fault(M.FAULT.POSITION_INVALID, "position recovered")
    else
        set_fault(M.FAULT.POSITION_INVALID, "position invalid")
    end

    if velocity_is_valid(sensor_data) then
        clear_fault(M.FAULT.VELOCITY_INVALID, "velocity recovered")
    else
        set_fault(M.FAULT.VELOCITY_INVALID, "velocity invalid")
    end

    -- ASCENT_TIMEOUT은 ASCENT 상태에서만 의미가 있지만(그 상태에서만
    -- launch_time_ms가 유효), "왜 fault인지" 판단 자체는 여기서만 한다 -
    -- 실제 DEPLOY 강제전이 액션은 update_ascent()가 그대로 수행한다.
    -- 한 번 세팅되면 이 비행 사이클 동안 clear하지 않는다(기존 동작 유지).
    if current_state == M.STATE.ASCENT
        and now - launch_time_ms
            >= ASCENT_TO_DEPLOY.MAX_TIME_AFTER_LAUNCH_MS then

        set_fault(M.FAULT.ASCENT_TIMEOUT, "maximum ascent time reached")
    end
end


-- ---------------------------------------------------------------------
-- State handlers
-- ---------------------------------------------------------------------

local function update_ready(now, sensor_data)
    local accel_valid = acceleration_is_valid(sensor_data)

    if accel_valid then
        accel_lost_start_ms = nil

        if accel_valid_start_ms == nil then
            accel_valid_start_ms = now
        end

        if now - accel_valid_start_ms
            >= READY_TO_ASCENT.ACCEL_PREVIOUS_STABLE_TIME_MS then

            if not accel_was_stable then
                accel_was_stable = true
                gcs:send_text(6, "COMET: Accel sensor stable")
            end
        end
    else
        accel_valid_start_ms = nil
    end

    if accel_valid
        and sensor_data.total_acceleration
            >= READY_TO_ASCENT.MIN_ACCEL_MPS2 then

        if launch_confirm_count == 0
            and not launch_detection_message_sent then

            launch_detection_message_sent = true
            gcs:send_text(6, "COMET: Launch detection started")
        end

        launch_confirm_count = launch_confirm_count + 1
    else
        launch_confirm_count = 0
        launch_detection_message_sent = false
    end

    if launch_confirm_count
        >= READY_TO_ASCENT.ACCEL_CONFIRM_COUNT then

        gcs:send_text(6, "COMET: Launch condition confirmed")
        enter_ascent(now, sensor_data, "acceleration confirmed")
        return
    end

    if accel_was_stable and not accel_valid then
        if accel_lost_start_ms == nil then
            accel_lost_start_ms = now
            gcs:send_text(4, "COMET: Accel sensor lost")
        end

        if now - accel_lost_start_ms
            >= READY_TO_ASCENT.ACCEL_LOST_TIME_MS then

            gcs:send_text(
                4,
                "COMET: Launch fallback by accel loss"
            )

            enter_ascent(now, sensor_data, "accel loss fallback")
        end
    end
end


local function update_ascent(now, sensor_data)
    local elapsed_ms = now - launch_time_ms

    if elapsed_ms
        >= ASCENT_TO_DEPLOY.MAX_TIME_AFTER_LAUNCH_MS then

        gcs:send_text(4, "COMET: Deploy fallback by timeout")

        change_state(
            M.STATE.DEPLOY,
            now,
            "maximum ascent time"
        )
        return
    end

    local position_valid = position_is_valid(sensor_data)
    local velocity_valid = velocity_is_valid(sensor_data)

    if not position_valid or not velocity_valid then
        descent_speed_confirm_count = 0
        descent_detection_message_sent = false
        return
    end

    if sensor_data.altitude > maximum_altitude_m then
        maximum_altitude_m = sensor_data.altitude
    end

    local minimum_time_passed =
        elapsed_ms
            >= ASCENT_TO_DEPLOY.MIN_TIME_AFTER_LAUNCH_MS

    local descending =
        sensor_data.velocity_d
            >= ASCENT_TO_DEPLOY.MIN_DESCENT_SPEED_MPS

    local altitude_high_enough =
        sensor_data.altitude
            >= ASCENT_TO_DEPLOY.MIN_ALTITUDE_M

    if minimum_time_passed
        and descending
        and altitude_high_enough then

        if descent_speed_confirm_count == 0
            and not descent_detection_message_sent then

            descent_detection_message_sent = true
            gcs:send_text(6, "COMET: Descent condition detected")
        end

        descent_speed_confirm_count =
            descent_speed_confirm_count + 1
    else
        descent_speed_confirm_count = 0
        descent_detection_message_sent = false
    end

    if descent_speed_confirm_count
        >= ASCENT_TO_DEPLOY.DESCENT_SPEED_CONFIRM_COUNT then

        gcs:send_text(6, "COMET: Descent condition confirmed")

        change_state(
            M.STATE.DEPLOY,
            now,
            "descent speed and altitude"
        )
    end
end


local function update_deploy(now)
    -- simulation mode: 실제 서보 출력 생략
    if not ASCENT_TO_DEPLOY.DEPLOY_ENABLE then
        if not deploy_attempted then
            deploy_attempted = true
            deploy_command_succeeded = true
            deploy_command_time_ms = now

            gcs:send_text(
                4,
                "COMET: Auto deploy disabled - simulation"
            )
        end

    -- 실제 자동 사출 모드
    elseif not deploy_command_succeeded then
        if not deploy_attempted then
            deploy_attempted = true
            deploy_parachute(now, "automatic")

        elseif deploy_retry_count
            < DEPLOY_TO_DESCENT.MAX_RETRY_COUNT
            and now - last_deploy_attempt_ms
                >= DEPLOY_TO_DESCENT.RETRY_INTERVAL_MS then

            deploy_retry_count = deploy_retry_count + 1

            gcs:send_text(
                4,
                "COMET: Retrying deploy command"
            )

            deploy_parachute(now, "automatic retry")
        end
    end

    if not deploy_command_succeeded then
        return
    end

    if now - deploy_command_time_ms
        >= DEPLOY_TO_DESCENT.SERVO_OPERATION_TIME_MS then

        gcs:send_text(6, "COMET: Servo operation time complete")

        change_state(
            M.STATE.DESCENT,
            now,
            "deploy operation completed"
        )
    end
end


local function update_descent(now, sensor_data)
    local position_valid = position_is_valid(sensor_data)
    local velocity_valid = velocity_is_valid(sensor_data)
    local accel_valid = acceleration_is_valid(sensor_data)

    if not position_valid
        or not velocity_valid
        or not accel_valid then

        landed_confirm_count = 0
        landing_detection_message_sent = false
        return
    end

    local altitude_low =
        sensor_data.altitude
            <= DESCENT_TO_LANDED.MAX_ALTITUDE_M

    local vertical_speed_low =
        math.abs(sensor_data.velocity_d)
            <= DESCENT_TO_LANDED.MAX_ABS_VERTICAL_SPEED_MPS

    local acceleration_stable =
        sensor_data.total_acceleration
            >= DESCENT_TO_LANDED.MIN_ACCEL_MPS2
        and sensor_data.total_acceleration
            <= DESCENT_TO_LANDED.MAX_ACCEL_MPS2

    if altitude_low
        and vertical_speed_low
        and acceleration_stable then

        if landed_confirm_count == 0
            and not landing_detection_message_sent then

            landing_detection_message_sent = true
            gcs:send_text(6, "COMET: Landing condition detected")
        end

        landed_confirm_count = landed_confirm_count + 1
    else
        landed_confirm_count = 0
        landing_detection_message_sent = false
    end

    if landed_confirm_count
        >= DESCENT_TO_LANDED.LANDED_CONFIRM_COUNT then

        gcs:send_text(6, "COMET: Landing condition confirmed")

        change_state(
            M.STATE.LANDED,
            now,
            "landing conditions confirmed"
        )
    end
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize(now)
    reset_state(now or 0)

    gcs:send_text(6, "COMET: Flight state initialized")

    if ASCENT_TO_DEPLOY.DEPLOY_ENABLE then
        gcs:send_text(6, "COMET: Deploy output enabled")
    else
        gcs:send_text(4, "COMET: Deploy output disabled")
    end

    return true
end


function M.update(now, sensor_data)
    process_command(now, sensor_data)
    update_faults(now, sensor_data)

    if current_state == M.STATE.READY then
        update_ready(now, sensor_data)

    elseif current_state == M.STATE.ASCENT then
        update_ascent(now, sensor_data)

    elseif current_state == M.STATE.DEPLOY then
        update_deploy(now)

    elseif current_state == M.STATE.DESCENT then
        update_descent(now, sensor_data)

    elseif current_state == M.STATE.LANDED then
        -- LANDED 상태 유지
    end

    return true
end


function M.get_state()
    return current_state
end


function M.get_fault_flags()
    return fault_flags
end


function M.has_fault()
    return fault_flags ~= M.FAULT.NONE
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

        gcs:send_text(4, "COMET: Reset rejected in flight")
        return false
    end

    local ok = servo.lock_all()

    if not ok then
        set_fault(
            M.FAULT.DEPLOY_COMMAND_FAILED,
            "servo lock failed during reset"
        )

        gcs:send_text(3, "COMET: Flight reset failed")
        return false
    end

    reset_state(now or 0)

    gcs:send_text(6, "COMET: Flight state reset")
    return true
end


return M
