-- =====================================================================
-- Comet Flight State Machine
-- =====================================================================
-- READY -> ASCENT -> DEPLOY -> DESCENT -> LANDED
--
-- 비행 상태와 시스템 이상을 분리한다.
-- 상태는 실제 비행 단계를 나타내며, 이상은 fault_flags로 관리한다.
-- =====================================================================

local CONFIG = require("comet_config")
local servo = require("comet_servo")
local lora = require("comet_lora")

local SERVO_CONFIG = CONFIG.SERVO
local TRANSITION = CONFIG.TRANSITION

local READY_TO_ASCENT = TRANSITION.READY_TO_ASCENT
local ASCENT_TO_DEPLOY = TRANSITION.ASCENT_TO_DEPLOY
local DEPLOY_TO_DESCENT = TRANSITION.DEPLOY_TO_DESCENT
local DESCENT_TO_LANDED = TRANSITION.DESCENT_TO_LANDED

local DEBUG_CONFIG = CONFIG.DEBUG or {}

-- DEBUG override가 활성화된 경우 디버그용 ASCENT → DEPLOY
-- 전이조건을 반환하고, 그렇지 않으면 실제 비행조건을 반환한다.
local function get_ascent_to_deploy_config()
    if DEBUG_CONFIG.ENABLE == true
        and DEBUG_CONFIG.ASCENT_TO_DEPLOY_OVERRIDE_ENABLE == true
        and DEBUG_CONFIG.ASCENT_TO_DEPLOY_OVERRIDE ~= nil then

        return DEBUG_CONFIG.ASCENT_TO_DEPLOY_OVERRIDE
    end

    return ASCENT_TO_DEPLOY
end

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
    ALTITUDE_INVALID      = 0x02,
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
local deploy_disabled_message_sent = false

local landed_confirm_count = 0
local landing_detection_message_sent = false

local sd_log_state = {
    last_log_ms = 0
}


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
        gcs:send_text(
            3,
            "COMET: [FLIGHT] Fault - " .. message
        )
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
            "COMET: [FLIGHT] Fault cleared - " .. message
        )
    end
end


local function change_state(new_state, now, reason)
    if current_state == new_state then
        return true
    end

    if new_state < M.STATE.READY
        or new_state > M.STATE.LANDED then

        gcs:send_text(
            3,
            "COMET: [FLIGHT] Invalid flight state"
        )

        return false
    end

    local old_state = current_state

    current_state = new_state

    gcs:send_text(
        6,
        "COMET: [FLIGHT] "
            .. state_name(old_state)
            .. " -> "
            .. state_name(new_state)
    )

    if reason ~= nil then
        gcs:send_text(
            6,
            "COMET: [FLIGHT] Reason - " .. reason
        )
    end

    return true
end


local function altitude_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.altitude_valid
end


local function enter_ascent(now, sensor_data, reason)
    launch_time_ms = now

    launch_confirm_count = 0
    descent_speed_confirm_count = 0
    landed_confirm_count = 0

    launch_detection_message_sent = false
    descent_detection_message_sent = false
    landing_detection_message_sent = false

    if altitude_is_valid(sensor_data) then
        maximum_altitude_m = sensor_data.altitude
    else
        maximum_altitude_m = 0.0
    end

    return change_state(
        M.STATE.ASCENT,
        now,
        reason
    )
end


local function enter_deploy(now, reason)
    deploy_attempted = false
    deploy_command_succeeded = false
    deploy_command_time_ms = 0
    parachute_deployed = false
    deploy_retry_count = 0
    last_deploy_attempt_ms = 0

    return change_state(
        M.STATE.DEPLOY,
        now,
        reason
    )
end


local function enter_descent(now, reason)
    landed_confirm_count = 0
    landing_detection_message_sent = false

    return change_state(
        M.STATE.DESCENT,
        now,
        reason
    )
end


local function request_deploy(now, reason)
    local auto_transition_disabled =
        DEBUG_CONFIG.ENABLE == true
        and DEBUG_CONFIG.DEPLOY_AUTO_TRANSITION_DISABLE == true

    if auto_transition_disabled then
        if not deploy_disabled_message_sent then
            deploy_disabled_message_sent = true

            gcs:send_text(
                4,
                "COMET: [FLIGHT] Automatic deploy request blocked"
            )
        end

        return false
    end

    return enter_deploy(
        now,
        reason
    )
end


local function enter_landed(now, reason)
    landed_confirm_count = 0
    landing_detection_message_sent = false

    return change_state(
        M.STATE.LANDED,
        now,
        reason
    )
end


local function reset_state()
    current_state = M.STATE.READY
    fault_flags = M.FAULT.NONE

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
    deploy_disabled_message_sent = false

    landed_confirm_count = 0
    landing_detection_message_sent = false

    sd_log_state.last_log_ms = 0
end


local function get_sensor_validity_flags(sensor_data)
    local flags = 0

    if sensor_data == nil then
        return flags
    end

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

    -- bit 4: barometric altitude valid
    if sensor_data.altitude_valid then
        flags = flags | 0x10
    end

    return flags
end


-- ---------------------------------------------------------------------
-- SD DataFlash logging
-- ---------------------------------------------------------------------

local function write_sd_log(now, sensor_data)
    if DEBUG_CONFIG.ENABLE ~= true then
        return
    end

    if DEBUG_CONFIG.SD_LOG_ENABLE ~= true then
        return
    end

    local period_ms =
        DEBUG_CONFIG.SD_LOG_PERIOD_MS or 20

    if now - sd_log_state.last_log_ms < period_ms then
        return
    end

    sd_log_state.last_log_ms = now

    local validity_flags =
        get_sensor_validity_flags(sensor_data)

    local acceleration = 0.0
    local altitude = 0.0
    local velocity_d = 0.0

    if sensor_data ~= nil then
        acceleration =
            sensor_data.total_acceleration or 0.0

        altitude =
            sensor_data.altitude or 0.0

        velocity_d =
            sensor_data.velocity_d or 0.0
    end

    local launch_elapsed_ms = 0

    if launch_time_ms > 0 then
        launch_elapsed_ms =
            now - launch_time_ms
    end

    local deploy_command_flag = 0
    local parachute_flag = 0

    if deploy_command_succeeded then
        deploy_command_flag = 1
    end

    if parachute_deployed then
        parachute_flag = 1
    end

    -- Lua가 상태머신에 실제로 전달한 센서값
    logger:write(
        "CMSN",
        "State,Fault,Valid,Acc,Alt,VD,MaxAlt",
        "BBBffff",
        current_state,
        fault_flags,
        validity_flags,
        acceleration,
        altitude,
        velocity_d,
        maximum_altitude_m
    )

    -- 상태머신 내부 카운터 및 사출 상태
    logger:write(
        "CMST",
        "LCnt,DCnt,NCnt,TLaunch,DpTry,DpCmd,DpOut",
        "BBHIBBB",
        launch_confirm_count,
        descent_speed_confirm_count,
        landed_confirm_count,
        launch_elapsed_ms,
        deploy_retry_count,
        deploy_command_flag,
        parachute_flag
    )
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
        "COMET: [FLIGHT] Deploy attempt "
            .. tostring(deploy_retry_count + 1)
    )

    local ok =
        servo.release_all()

    if not ok then
        set_fault(
            M.FAULT.DEPLOY_COMMAND_FAILED,
            "deploy command failed"
        )

        gcs:send_text(
            3,
            "COMET: [FLIGHT] Deploy attempt failed"
        )

        return false
    end

    deploy_command_succeeded = true
    deploy_command_time_ms = now
    parachute_deployed = true

    clear_fault(
        M.FAULT.DEPLOY_COMMAND_FAILED,
        "deploy command recovered"
    )

    gcs:send_text(
        6,
        "COMET: [FLIGHT] Deploy command succeeded"
    )

    if reason ~= nil then
        gcs:send_text(
            6,
            "COMET: [FLIGHT] Deploy - " .. reason
        )
    end

    return true
end


-- ---------------------------------------------------------------------
-- LoRa command processing
-- ---------------------------------------------------------------------

local function execute_servo_command(command)
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


local function is_servo_command_allowed(command)
    -- 개별 메인 서보 명령은 메인 서보가 활성화되어 있어야 한다.
    if command == lora.COMMAND.SERVO_MAIN_LOCK
        or command == lora.COMMAND.SERVO_MAIN_RELEASE then

        if SERVO_CONFIG.SERVO_MAIN == nil
            or SERVO_CONFIG.SERVO_MAIN.ENABLED ~= true then

            return false
        end
    end

    -- 개별 후보 서보 명령은 후보 서보가 활성화되어 있어야 한다.
    if command == lora.COMMAND.SERVO_CANDIDATE_LOCK
        or command == lora.COMMAND.SERVO_CANDIDATE_RELEASE then

        if SERVO_CONFIG.SERVO_CANDIDATE == nil
            or SERVO_CONFIG.SERVO_CANDIDATE.ENABLED ~= true then

            return false
        end
    end

    -- 디버그 모드에서는 비행 상태 제한만 우회한다.
    -- 존재하지 않거나 비활성화된 서보에 대한 명령은 여전히 거부한다.
    if DEBUG_CONFIG.ENABLE == true
        and DEBUG_CONFIG.SERVO_COMMAND_BYPASS_STATE_ENABLE == true then

        return true
    end

    -- 정상 운용: LOCK은 지상 상태에서만 허용한다.
    if is_lock_command(command) then
        return current_state == M.STATE.READY
            or current_state == M.STATE.LANDED
    end

    -- 정상 운용: RELEASE는 비행 중에만 허용한다.
    if is_release_command(command) then
        return current_state == M.STATE.ASCENT
            or current_state == M.STATE.DEPLOY
            or current_state == M.STATE.DESCENT
    end

    return false
end


local function is_force_state_command(command)
    return command == lora.COMMAND.FORCE_ASCENT
        or command == lora.COMMAND.FORCE_DEPLOY
        or command == lora.COMMAND.FORCE_DESCENT
        or command == lora.COMMAND.FORCE_LANDED
end


local function is_force_state_command_allowed(command)
    if command == lora.COMMAND.FORCE_ASCENT then
        return current_state == M.STATE.READY

    elseif command == lora.COMMAND.FORCE_DEPLOY then
        return current_state == M.STATE.ASCENT

    elseif command == lora.COMMAND.FORCE_DESCENT then
        return current_state == M.STATE.ASCENT
            or current_state == M.STATE.DEPLOY

    elseif command == lora.COMMAND.FORCE_LANDED then
        return current_state == M.STATE.DESCENT
    end

    return false
end


local function execute_force_state_command(
    command,
    now,
    sensor_data
)
    if command == lora.COMMAND.FORCE_ASCENT then
        return enter_ascent(
            now,
            sensor_data,
            "remote force command"
        )

    elseif command == lora.COMMAND.FORCE_DEPLOY then
        return enter_deploy(
            now,
            "remote force command"
        )

    elseif command == lora.COMMAND.FORCE_DESCENT then
        return enter_descent(
            now,
            "remote force command"
        )

    elseif command == lora.COMMAND.FORCE_LANDED then
        return enter_landed(
            now,
            "remote force command"
        )
    end

    return false
end


local function process_lora_command(now, sensor_data)
    local received =
        lora.get_command()

    if received == nil then
        return
    end

    local sequence = received.sequence
    local command = received.command

    gcs:send_text(
        6,
        "COMET: [FLIGHT] Remote command received"
    )

    local force_state_command =
        is_force_state_command(command)

    local allowed

    if force_state_command then
        allowed =
            is_force_state_command_allowed(command)
    else
        allowed =
            is_servo_command_allowed(command)
    end

    if not allowed then
        gcs:send_text(
            4,
            "COMET: [FLIGHT] Remote command rejected"
        )

        lora.schedule_ack(
            sequence,
            command,
            lora.ACK_RESULT.NOT_ALLOWED
        )

        return
    end

    local ok

    if force_state_command then
        ok =
            execute_force_state_command(
                command,
                now,
                sensor_data
            )
    else
        ok =
            execute_servo_command(command)
    end

    if not ok then
        gcs:send_text(
            3,
            "COMET: [FLIGHT] Remote command failed"
        )

        if not force_state_command
            and is_main_release_command(command) then

            set_fault(
                M.FAULT.DEPLOY_COMMAND_FAILED,
                "remote deploy failed"
            )
        end

        lora.schedule_ack(
            sequence,
            command,
            lora.ACK_RESULT.EXECUTION_FAILED
        )

        return
    end

    gcs:send_text(
        6,
        "COMET: [FLIGHT] Remote command succeeded"
    )

    if not force_state_command
        and is_main_release_command(command) then

        if current_state == M.STATE.ASCENT then
            enter_deploy(
                now,
                "remote deploy command"
            )
        end

        parachute_deployed = true
        deploy_attempted = true
        deploy_command_succeeded = true
        deploy_command_time_ms = now

        clear_fault(
            M.FAULT.DEPLOY_COMMAND_FAILED,
            "remote deploy recovered"
        )
    end

    if not force_state_command
        and (
            command == lora.COMMAND.SERVO_MAIN_LOCK
            or command == lora.COMMAND.LOCK_ALL
        ) then

        parachute_deployed = false
    end

    lora.schedule_ack(
        sequence,
        command,
        lora.ACK_RESULT.OK
    )
end


-- ---------------------------------------------------------------------
-- Sensor validity
-- ---------------------------------------------------------------------

local function acceleration_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.acceleration_valid
end


local function velocity_is_valid(sensor_data)
    return sensor_data ~= nil
        and sensor_data.velocity_valid
end


-- ---------------------------------------------------------------------
-- State handlers
-- ---------------------------------------------------------------------

local function update_ready(now, sensor_data)
    local accel_valid =
        acceleration_is_valid(sensor_data)

    -- -------------------------------------------------------------
    -- 가속도 센서 안정 상태 및 fault 관리
    -- -------------------------------------------------------------
    if accel_valid then
        accel_lost_start_ms = nil

        clear_fault(
            M.FAULT.ACCEL_SENSOR_INVALID,
            "accel recovered during READY"
        )

        if accel_valid_start_ms == nil then
            accel_valid_start_ms = now
        end

        if now - accel_valid_start_ms
            >= READY_TO_ASCENT.ACCEL_PREVIOUS_STABLE_TIME_MS then

            if not accel_was_stable then
                accel_was_stable = true

                gcs:send_text(
                    6,
                    "COMET: [FLIGHT] Accel sensor stable"
                )
            end
        end

    else
        accel_valid_start_ms = nil

        if accel_was_stable then
            if accel_lost_start_ms == nil then
                accel_lost_start_ms = now

                gcs:send_text(
                    4,
                    "COMET: [FLIGHT] Accel sensor lost"
                )
            end

            if now - accel_lost_start_ms
                >= READY_TO_ASCENT.ACCEL_LOST_TIME_MS then

                set_fault(
                    M.FAULT.ACCEL_SENSOR_INVALID,
                    "accel lost during READY"
                )
            end
        end
    end

    -- -------------------------------------------------------------
    -- 정상 발사 감지
    -- -------------------------------------------------------------
    if accel_valid
        and sensor_data.total_acceleration
            >= READY_TO_ASCENT.MIN_ACCEL_MPS2 then

        if launch_confirm_count == 0
            and not launch_detection_message_sent then

            launch_detection_message_sent = true

            gcs:send_text(
                6,
                "COMET: [FLIGHT] Launch detection started"
            )
        end

        launch_confirm_count =
            launch_confirm_count + 1

    else
        launch_confirm_count = 0
        launch_detection_message_sent = false
    end

    if launch_confirm_count
        >= READY_TO_ASCENT.ACCEL_CONFIRM_COUNT then

        gcs:send_text(
            6,
            "COMET: [FLIGHT] Launch condition confirmed"
        )

        enter_ascent(
            now,
            sensor_data,
            "acceleration confirmed"
        )

        return
    end
end


local function update_ascent(now, sensor_data)
    local elapsed_ms =
        now - launch_time_ms

    local transition_config =
        get_ascent_to_deploy_config()

    if acceleration_is_valid(sensor_data) then
        clear_fault(
            M.FAULT.ACCEL_SENSOR_INVALID,
            "accel recovered"
        )
    else
        set_fault(
            M.FAULT.ACCEL_SENSOR_INVALID,
            "accel invalid during ASCENT"
        )
    end

    -- -------------------------------------------------------------
    -- 최대 상승시간 fallback
    -- -------------------------------------------------------------
    local maximum_time_ms =
        transition_config.MAX_TIME_AFTER_LAUNCH_MS

    if maximum_time_ms ~= nil
        and elapsed_ms >= maximum_time_ms then

        set_fault(
            M.FAULT.ASCENT_TIMEOUT,
            "maximum ascent time reached"
        )

        local transitioned =
            request_deploy(
                now,
                "maximum ascent time"
            )

        if transitioned then
            gcs:send_text(
                4,
                "COMET: [FLIGHT] Deploy fallback by timeout"
            )
        end

        return
    end

    local altitude_valid =
        altitude_is_valid(sensor_data)

    local velocity_valid =
        velocity_is_valid(sensor_data)

    -- -------------------------------------------------------------
    -- 센서 fault 관리
    -- -------------------------------------------------------------
    if not altitude_valid then
        set_fault(
            M.FAULT.ALTITUDE_INVALID,
            "altitude invalid during ASCENT"
        )
    else
        clear_fault(
            M.FAULT.ALTITUDE_INVALID,
            "altitude recovered"
        )
    end

    if not velocity_valid then
        set_fault(
            M.FAULT.VELOCITY_INVALID,
            "velocity invalid during ASCENT"
        )
    else
        clear_fault(
            M.FAULT.VELOCITY_INVALID,
            "velocity recovered"
        )
    end

    -- -------------------------------------------------------------
    -- 최대고도 갱신
    --
    -- 속도 데이터가 invalid여도 고도가 유효하면 갱신한다.
    -- -------------------------------------------------------------
    if altitude_valid
        and sensor_data.altitude > maximum_altitude_m then

        maximum_altitude_m =
            sensor_data.altitude
    end

    -- 하강속도 판정에는 velocity가 반드시 필요하다.
    if not velocity_valid then
        descent_speed_confirm_count = 0
        descent_detection_message_sent = false
        return
    end

    -- -------------------------------------------------------------
    -- 전이조건 계산
    -- -------------------------------------------------------------
    local minimum_time_passed =
        elapsed_ms
            >= transition_config.MIN_TIME_AFTER_LAUNCH_MS

    local descending =
        sensor_data.velocity_d
            >= transition_config.MIN_DESCENT_SPEED_MPS

    -- 현재 고도 제한
    local altitude_condition_passed = true

    if transition_config.MIN_ALTITUDE_M ~= nil then
        altitude_condition_passed =
            altitude_valid
            and sensor_data.altitude
                >= transition_config.MIN_ALTITUDE_M
    end

    -- 최고고도 대비 하락량
    local altitude_drop_m = 0.0

    if altitude_valid then
        altitude_drop_m =
            maximum_altitude_m - sensor_data.altitude
    end

    -- 최고고도 대비 최소 하락량 조건
    local altitude_drop_condition_passed = true

    if transition_config.MIN_ALTITUDE_DROP_M ~= nil then
        altitude_drop_condition_passed =
            altitude_valid
            and altitude_drop_m
                >= transition_config.MIN_ALTITUDE_DROP_M
    end

    -- -------------------------------------------------------------
    -- 하강조건 연속 확인
    -- -------------------------------------------------------------
    if minimum_time_passed
        and descending
        and altitude_condition_passed
        and altitude_drop_condition_passed then

        if descent_speed_confirm_count == 0
            and not descent_detection_message_sent then

            descent_detection_message_sent = true

            gcs:send_text(
                6,
                "COMET: [FLIGHT] Descent condition detected"
            )
        end

        descent_speed_confirm_count =
            descent_speed_confirm_count + 1

    else
        descent_speed_confirm_count = 0
        descent_detection_message_sent = false
    end

    -- -------------------------------------------------------------
    -- DEPLOY 전이
    -- -------------------------------------------------------------
    if descent_speed_confirm_count
        >= transition_config.DESCENT_SPEED_CONFIRM_COUNT then

        local transitioned =
            request_deploy(
                now,
                "descent speed and altitude drop condition"
            )

        if transitioned then
            gcs:send_text(
                6,
                "COMET: [FLIGHT] Descent condition confirmed"
            )
        end
    end
end

local function update_deploy(now)
    if not deploy_command_succeeded then
        if not deploy_attempted then
            deploy_attempted = true
            deploy_parachute(
                now,
                "automatic"
            )

        elseif deploy_retry_count
            < DEPLOY_TO_DESCENT.MAX_RETRY_COUNT
            and now - last_deploy_attempt_ms
                >= DEPLOY_TO_DESCENT.RETRY_INTERVAL_MS then

            deploy_retry_count =
                deploy_retry_count + 1

            gcs:send_text(
                4,
                "COMET: [FLIGHT] Retrying deploy command"
            )

            deploy_parachute(
                now,
                "automatic retry"
            )
        end
    end

    local retries_exhausted =
        deploy_attempted
        and not deploy_command_succeeded
        and deploy_retry_count
            >= DEPLOY_TO_DESCENT.MAX_RETRY_COUNT

    if retries_exhausted then
        set_fault(
            M.FAULT.DEPLOY_COMMAND_FAILED,
            "deploy retries exhausted"
        )

        gcs:send_text(
            3,
            "COMET: [FLIGHT] Deploy retries exhausted"
        )

        enter_descent(
            now,
            "deploy failed after retries"
        )

        return
    end

    if not deploy_command_succeeded then
        return
    end

    if now - deploy_command_time_ms
        >= DEPLOY_TO_DESCENT.SERVO_OPERATION_TIME_MS then

        gcs:send_text(
            6,
            "COMET: [FLIGHT] Servo operation time complete"
        )

        enter_descent(
            now,
            "deploy operation completed"
        )
    end
end


local function update_descent(now, sensor_data)
    local altitude_valid =
        altitude_is_valid(sensor_data)

    local velocity_valid =
        velocity_is_valid(sensor_data)

    local accel_valid =
        acceleration_is_valid(sensor_data)

    if not altitude_valid then
        set_fault(
            M.FAULT.ALTITUDE_INVALID,
            "altitude invalid during DESCENT"
        )
    else
        clear_fault(
            M.FAULT.ALTITUDE_INVALID,
            "altitude recovered"
        )
    end

    if not velocity_valid then
        set_fault(
            M.FAULT.VELOCITY_INVALID,
            "velocity invalid during DESCENT"
        )
    else
        clear_fault(
            M.FAULT.VELOCITY_INVALID,
            "velocity recovered"
        )
    end

    if not accel_valid then
        set_fault(
            M.FAULT.ACCEL_SENSOR_INVALID,
            "accel invalid during DESCENT"
        )
    else
        clear_fault(
            M.FAULT.ACCEL_SENSOR_INVALID,
            "accel recovered"
        )
    end

    if not altitude_valid
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

            gcs:send_text(
                6,
                "COMET: [FLIGHT] Landing condition detected"
            )
        end

        landed_confirm_count =
            landed_confirm_count + 1

    else
        landed_confirm_count = 0
        landing_detection_message_sent = false
    end

    if landed_confirm_count
        >= DESCENT_TO_LANDED.LANDED_CONFIRM_COUNT then

        gcs:send_text(
            6,
            "COMET: [FLIGHT] Landing condition confirmed"
        )

        enter_landed(
            now,
            "landing conditions confirmed"
        )
    end
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize(now)
    reset_state()

    gcs:send_text(
        6,
        "COMET: [FLIGHT] Flight state initialized"
    )

    if DEBUG_CONFIG.ENABLE == true then
        gcs:send_text(
            4,
            "COMET: [FLIGHT] Debug mode enabled"
        )

        if DEBUG_CONFIG.SERVO_COMMAND_BYPASS_STATE_ENABLE == true then
            gcs:send_text(
                4,
                "COMET: [FLIGHT] Servo command state bypass enabled"
            )
        else
            gcs:send_text(
                6,
                "COMET: [FLIGHT] Servo command state bypass disabled"
            )
        end

        if DEBUG_CONFIG.DEPLOY_AUTO_TRANSITION_DISABLE == true then
            gcs:send_text(
                4,
                "COMET: [FLIGHT] Automatic deploy transition disabled"
            )
        else
            gcs:send_text(
                6,
                "COMET: [FLIGHT] Automatic deploy transition enabled"
            )
        end

    else
        gcs:send_text(
            6,
            "COMET: [FLIGHT] Debug mode disabled"
        )

        gcs:send_text(
            6,
            "COMET: [FLIGHT] Servo command state bypass disabled"
        )

        gcs:send_text(
            6,
            "COMET: [FLIGHT] Automatic deploy transition enabled"
        )
    end

    return true
end


function M.update(now, sensor_data)
    process_lora_command(
        now,
        sensor_data
    )

    if current_state == M.STATE.READY then
        update_ready(
            now,
            sensor_data
        )

    elseif current_state == M.STATE.ASCENT then
        update_ascent(
            now,
            sensor_data
        )

    elseif current_state == M.STATE.DEPLOY then
        update_deploy(now)

    elseif current_state == M.STATE.DESCENT then
        update_descent(
            now,
            sensor_data
        )

    elseif current_state == M.STATE.LANDED then
        -- LANDED 상태 유지
    end

    -- 현재 루프의 상태 전이 판단이 끝난 후 기록
    write_sd_log(
        now,
        sensor_data
    )

    return true
end


function M.get_state()
    return current_state
end


function M.get_fault_flags()
    return fault_flags
end


function M.set_lora_available(available)
    if available then
        clear_fault(
            M.FAULT.LORA_UNAVAILABLE,
            "LoRa recovered"
        )
    else
        set_fault(
            M.FAULT.LORA_UNAVAILABLE,
            "LoRa unavailable"
        )
    end
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

        gcs:send_text(
            4,
            "COMET: [FLIGHT] Reset rejected in flight"
        )

        return false
    end

    local ok =
        servo.lock_all()

    if not ok then
        gcs:send_text(
            3,
            "COMET: [FLIGHT] Flight reset failed - servo lock failed"
        )

        return false
    end

    reset_state()

    gcs:send_text(
        6,
        "COMET: [FLIGHT] Flight state reset"
    )

    return true
end


return M