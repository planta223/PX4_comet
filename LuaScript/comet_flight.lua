-- =====================================================================
-- Comet Flight Controller
-- =====================================================================
--
-- 역할:
--   1. ArduPilot AHRS/EKF 데이터 읽기
--   2. 발사, 정점, 착륙 상태 판단
--   3. 낙하산 사출 서보 제어
--   4. LoRa 텔레메트리 송신
--
-- 파일 구조:
--   APM/scripts/comet_flight.lua
--   APM/scripts/modules/comet_config.lua
--
-- 좌표계:
--   ArduPilot NED
--   velocity.z < 0 : 상승
--   velocity.z > 0 : 하강
-- =====================================================================


-- =====================================================================
-- 1. Configuration
-- =====================================================================

local CONFIG = require("comet_config")

local LAUNCH  = CONFIG.LAUNCH
local APOGEE  = CONFIG.APOGEE
local SAFETY  = CONFIG.SAFETY
local LANDING = CONFIG.LANDING
local SERVO   = CONFIG.SERVO
local LORA    = CONFIG.LORA


-- =====================================================================
-- 2. State definitions
-- =====================================================================

local STATE = {
    READY   = 0,
    ASCENT  = 1,
    APOGEE  = 2,
    DESCENT = 3,
    LANDED  = 4,
    ERROR   = 5
}

local STATE_NAME = {
    [STATE.READY]   = "READY",
    [STATE.ASCENT]  = "ASCENT",
    [STATE.APOGEE]  = "APOGEE",
    [STATE.DESCENT] = "DESCENT",
    [STATE.LANDED]  = "LANDED",
    [STATE.ERROR]   = "ERROR"
}

local ERROR = {
    NONE           = 0,
    SENSOR_TIMEOUT = 1,
    ASCENT_TIMEOUT = 2,
    EXCESSIVE_TILT = 3
}


-- =====================================================================
-- 3. Navigation data
-- =====================================================================
--
-- 내부 상태판단과 LoRa 송신에서 공통으로 사용할 최신 데이터입니다.
--
-- acceleration:
--   발사·착륙 판단에만 사용
--
-- altitude / velocity / attitude:
--   AHRS/EKF 결과 사용
-- =====================================================================

local nav = {
    acceleration_mps2 = 0.0,

    relative_altitude_m = 0.0,

    velocity_north_mps = 0.0,
    velocity_east_mps  = 0.0,
    velocity_down_mps  = 0.0,

    latitude_deg  = 0.0,
    longitude_deg = 0.0,

    roll_deg  = 0.0,
    pitch_deg = 0.0,

    acceleration_valid = false,
    position_valid     = false,
    velocity_valid     = false,
    attitude_valid     = false,

    last_required_update_ms = 0
}


-- =====================================================================
-- 4. Flight context
-- =====================================================================

local flight = {
    state = STATE.READY,
    previous_state = STATE.READY,
    error_code = ERROR.NONE,

    state_enter_ms = 0,
    launch_ms = 0,

    launch_confirm_count = 0,
    apogee_confirm_count = 0,
    landing_confirm_count = 0,

    max_altitude_m = 0.0,

    parachute_deployed = false
}


-- =====================================================================
-- 5. Reference altitude
-- =====================================================================
--
-- 첫 번째 유효 EKF 위치 고도를 기준고도로 저장합니다.
--
-- Location:alt() 단위는 cm이므로 100으로 나눕니다.
-- =====================================================================

local reference_altitude_m = nil


-- =====================================================================
-- 6. LoRa initialization
-- =====================================================================

local lora = serial:find_serial(LORA.SERIAL_INSTANCE)

local lora_sequence = 0
local last_lora_ms = 0
local lora_tx_failure_count = 0

if lora then
    lora:begin(LORA.BAUD)
    lora:set_flow_control(0)

    gcs:send_text(
        6,
        "COMET: LoRa ready, baud=" .. tostring(LORA.BAUD)
    )
else
    gcs:send_text(
        4,
        "COMET: LoRa unavailable"
    )
end


-- =====================================================================
-- 7. Utility functions
-- =====================================================================

local function state_name(state)
    return STATE_NAME[state] or "UNKNOWN"
end


local function enter_state(new_state, now)
    if flight.state == new_state then
        return
    end

    flight.previous_state = flight.state
    flight.state = new_state
    flight.state_enter_ms = now

    gcs:send_text(
        6,
        "COMET: "
            .. state_name(flight.previous_state)
            .. " -> "
            .. state_name(new_state)
    )
end


local function reset_confirmation_counts()
    flight.launch_confirm_count = 0
    flight.apogee_confirm_count = 0
    flight.landing_confirm_count = 0
end


-- =====================================================================
-- 8. Parachute servo
-- =====================================================================

local function lock_parachute()
    servo.set_output_pwm(
        SERVO.SCRIPT_FUNCTION,
        SERVO.LOCK_PWM
    )
end


local function deploy_parachute()
    if flight.parachute_deployed then
        return
    end

    servo.set_output_pwm(
        SERVO.SCRIPT_FUNCTION,
        SERVO.RELEASE_PWM
    )

    flight.parachute_deployed = true

    gcs:send_text(
        2,
        "COMET: Parachute deployed"
    )
end


-- =====================================================================
-- 9. Sensor and EKF update
-- =====================================================================

local function update_acceleration()
    -- 현재 기존 코드와 동일하게 AHRS 가속도 벡터를 사용합니다.
    --
    -- 주의:
    -- 실제 펌웨어에서 ahrs:get_accel() 바인딩 지원 여부를 확인해야 합니다.
    -- 지원되지 않는 경우 INS API 또는 다른 가속도 바인딩으로 변경해야 합니다.

    local acceleration = ahrs:get_accel()

    if acceleration then
        nav.acceleration_mps2 = acceleration:length()
        nav.acceleration_valid = true
    else
        nav.acceleration_valid = false
    end
end


local function update_position()
    local position = ahrs:get_position()

    if not position then
        nav.position_valid = false
        return
    end

    local altitude_m = position:alt() / 100.0

    if reference_altitude_m == nil then
        reference_altitude_m = altitude_m
    end

    nav.relative_altitude_m =
        altitude_m - reference_altitude_m

    nav.latitude_deg =
        position:lat() / 1.0e7

    nav.longitude_deg =
        position:lng() / 1.0e7

    nav.position_valid = true
end


local function update_velocity()
    local velocity = ahrs:get_velocity_NED()

    if not velocity then
        nav.velocity_valid = false
        return
    end

    nav.velocity_north_mps = velocity:x()
    nav.velocity_east_mps  = velocity:y()
    nav.velocity_down_mps  = velocity:z()

    nav.velocity_valid = true
end


local function update_attitude()
    local roll = ahrs:get_roll()
    local pitch = ahrs:get_pitch()

    if roll == nil or pitch == nil then
        nav.attitude_valid = false
        return
    end

    nav.roll_deg = math.deg(roll)
    nav.pitch_deg = math.deg(pitch)

    nav.attitude_valid = true
end


local function update_navigation(now)
    update_acceleration()
    update_position()
    update_velocity()
    update_attitude()

    -- 비행 상태판단에 필요한 핵심 데이터가 모두 유효한 경우에만
    -- 워치독 기준시간을 갱신합니다.
    if nav.acceleration_valid
        and nav.position_valid
        and nav.velocity_valid
        and nav.attitude_valid then

        nav.last_required_update_ms = now
    end
end


-- =====================================================================
-- 10. READY state
-- =====================================================================

local function update_ready(now)
    if not nav.acceleration_valid then
        flight.launch_confirm_count = 0
        return
    end

    if nav.acceleration_mps2
        >= LAUNCH.ACCEL_THRESHOLD_MPS2 then

        flight.launch_confirm_count =
            flight.launch_confirm_count + 1
    else
        flight.launch_confirm_count = 0
    end

    if flight.launch_confirm_count
        < LAUNCH.CONFIRM_COUNT then

        return
    end

    flight.launch_ms = now
    flight.max_altitude_m = nav.relative_altitude_m
    flight.error_code = ERROR.NONE

    reset_confirmation_counts()
    enter_state(STATE.ASCENT, now)

    gcs:send_text(
        6,
        "COMET: Liftoff detected"
    )
end


-- =====================================================================
-- 11. ASCENT state
-- =====================================================================

local function update_ascent(now)
    local elapsed_ms = now - flight.launch_ms

    -- ---------------------------------------------------------------
    -- Maximum altitude update
    -- ---------------------------------------------------------------

    if nav.position_valid
        and nav.relative_altitude_m
            > flight.max_altitude_m then

        flight.max_altitude_m =
            nav.relative_altitude_m
    end

    -- ---------------------------------------------------------------
    -- Safety: ascent timeout
    -- ---------------------------------------------------------------

    if elapsed_ms >= APOGEE.MAX_ASCENT_TIME_MS then
        flight.error_code = ERROR.ASCENT_TIMEOUT
        enter_state(STATE.ERROR, now)
        return
    end

    -- ---------------------------------------------------------------
    -- Safety: excessive tilt
    -- ---------------------------------------------------------------
    --
    -- 발사 직후에는 자세 변화와 EKF 과도응답이 있을 수 있으므로
    -- MIN_TIME_AFTER_LAUNCH_MS 이후부터 평가합니다.
    -- ---------------------------------------------------------------

    if elapsed_ms >= APOGEE.MIN_TIME_AFTER_LAUNCH_MS
        and nav.attitude_valid then

        local excessive_tilt =
            math.abs(nav.roll_deg) >= SAFETY.MAX_TILT_DEG
            or math.abs(nav.pitch_deg) >= SAFETY.MAX_TILT_DEG

        if excessive_tilt then
            flight.error_code = ERROR.EXCESSIVE_TILT
            enter_state(STATE.ERROR, now)
            return
        end
    end

    -- ---------------------------------------------------------------
    -- Minimum apogee detection time
    -- ---------------------------------------------------------------

    if elapsed_ms
        < APOGEE.MIN_TIME_AFTER_LAUNCH_MS then

        flight.apogee_confirm_count = 0
        return
    end

    -- ---------------------------------------------------------------
    -- Apogee detection
    -- ---------------------------------------------------------------
    --
    -- NED 좌표계:
    --   velocity_down < 0 : 상승
    --   velocity_down > 0 : 하강
    --
    -- 조건 A:
    --   EKF 수직속도가 하강 방향 임계값을 초과
    --
    -- 조건 B:
    --   현재 고도가 최고고도보다 지정값 이상 감소
    --
    -- 둘 중 하나가 연속으로 CONFIRM_COUNT회 검출되면 정점으로 판단합니다.
    -- ---------------------------------------------------------------

    local descending_by_velocity =
        nav.velocity_valid
        and nav.velocity_down_mps
            >= APOGEE.DOWN_SPEED_THRESHOLD_MPS

    local descending_by_altitude =
        nav.position_valid
        and nav.relative_altitude_m
            <= flight.max_altitude_m
                - APOGEE.ALTITUDE_DROP_MARGIN_M

    if descending_by_velocity
        or descending_by_altitude then

        flight.apogee_confirm_count =
            flight.apogee_confirm_count + 1
    else
        flight.apogee_confirm_count = 0
    end

    if flight.apogee_confirm_count
        >= APOGEE.CONFIRM_COUNT then

        enter_state(STATE.APOGEE, now)

        gcs:send_text(
            6,
            "COMET: Apogee detected"
        )
    end
end


-- =====================================================================
-- 12. APOGEE state
-- =====================================================================

local function update_apogee(now)
    deploy_parachute()

    flight.landing_confirm_count = 0

    enter_state(STATE.DESCENT, now)
end


-- =====================================================================
-- 13. DESCENT state
-- =====================================================================

local function update_descent(now)
    local altitude_ok =
        nav.position_valid
        and nav.relative_altitude_m
            <= LANDING.MAX_LANDING_ALTITUDE_M

    local vertical_speed_ok =
        nav.velocity_valid
        and math.abs(nav.velocity_down_mps)
            <= LANDING.MAX_VERTICAL_SPEED_MPS

    local acceleration_ok =
        nav.acceleration_valid
        and nav.acceleration_mps2
            >= LANDING.ACCEL_MIN_MPS2
        and nav.acceleration_mps2
            <= LANDING.ACCEL_MAX_MPS2

    if altitude_ok
        and vertical_speed_ok
        and acceleration_ok then

        flight.landing_confirm_count =
            flight.landing_confirm_count + 1
    else
        flight.landing_confirm_count = 0
    end

    if flight.landing_confirm_count
        >= LANDING.CONFIRM_COUNT then

        enter_state(STATE.LANDED, now)

        gcs:send_text(
            6,
            "COMET: Landing confirmed"
        )
    end
end


-- =====================================================================
-- 14. ERROR state
-- =====================================================================

local function update_error(now)
    -- 최소 정책:
    -- 비행 중 치명적 오류가 발생하면 낙하산을 즉시 사출하고
    -- DESCENT 상태로 전환합니다.

    deploy_parachute()
    enter_state(STATE.DESCENT, now)
end


-- =====================================================================
-- 15. Sensor watchdog
-- =====================================================================

local function update_sensor_watchdog(now)
    local monitored =
        flight.state == STATE.ASCENT
        or flight.state == STATE.APOGEE

    if not monitored then
        return
    end

    if now - nav.last_required_update_ms
        < SAFETY.SENSOR_TIMEOUT_MS then

        return
    end

    flight.error_code = ERROR.SENSOR_TIMEOUT
    enter_state(STATE.ERROR, now)
end


-- =====================================================================
-- 16. Flight state machine
-- =====================================================================

local function update_flight_state(now)
    update_sensor_watchdog(now)

    if flight.state == STATE.READY then
        update_ready(now)

    elseif flight.state == STATE.ASCENT then
        update_ascent(now)

    elseif flight.state == STATE.APOGEE then
        update_apogee(now)

    elseif flight.state == STATE.DESCENT then
        update_descent(now)

    elseif flight.state == STATE.ERROR then
        update_error(now)

    elseif flight.state == STATE.LANDED then
        -- 착륙 후 상태 유지
    end
end


-- =====================================================================
-- 17. LoRa health flags
-- =====================================================================
--
-- bit 0: position valid
-- bit 1: velocity valid
-- bit 2: attitude valid
-- bit 3: acceleration valid
-- =====================================================================

local function make_health_flags()
    local flags = 0

    if nav.position_valid then
        flags = flags | 0x01
    end

    if nav.velocity_valid then
        flags = flags | 0x02
    end

    if nav.attitude_valid then
        flags = flags | 0x04
    end

    if nav.acceleration_valid then
        flags = flags | 0x08
    end

    return flags
end


-- =====================================================================
-- 18. LoRa checksum
-- =====================================================================

local function xor_checksum(text)
    local checksum = 0

    for i = 1, #text do
        checksum =
            checksum ~ string.byte(text, i)
    end

    return checksum & 0xFF
end


-- =====================================================================
-- 19. LoRa packet
-- =====================================================================
--
-- ASCII packet:
--
-- $COMET,
-- version,
-- sequence,
-- time_ms,
-- state,
-- error,
-- deployed,
-- health,
-- altitude,
-- velocity_down,
-- latitude,
-- longitude,
-- roll,
-- pitch
-- *checksum
--
-- 실제 한 줄:
--
-- $COMET,1,15,5230,1,0,0,15,42.3,-18.6,
-- 37.1234567,127.1234567,2.1,-1.7*5A
--
-- raw IMU와 sensor bias는 송신하지 않습니다.
-- =====================================================================

local function make_lora_packet(now)
    local body = string.format(
        "COMET,%d,%d,%d,%d,%d,%d,%d,"
            .. "%.1f,%.1f,%.7f,%.7f,%.1f,%.1f",

        LORA.PROTOCOL_VERSION,
        lora_sequence,
        now,

        flight.state,
        flight.error_code,
        flight.parachute_deployed and 1 or 0,
        make_health_flags(),

        nav.relative_altitude_m,
        nav.velocity_down_mps,

        nav.latitude_deg,
        nav.longitude_deg,

        nav.roll_deg,
        nav.pitch_deg
    )

    local checksum = xor_checksum(body)

    return string.format(
        "$%s*%02X\r\n",
        body,
        checksum
    )
end


-- =====================================================================
-- 20. LoRa transmission
-- =====================================================================

local function send_lora(now)
    if not lora then
        return false
    end

    local packet = make_lora_packet(now)
    local written = lora:writestring(packet)

    if written ~= #packet then
        lora_tx_failure_count =
            lora_tx_failure_count + 1

        if lora_tx_failure_count == 1
            or lora_tx_failure_count % 10 == 0 then

            gcs:send_text(
                4,
                "COMET: LoRa TX failure="
                    .. tostring(lora_tx_failure_count)
            )
        end

        return false
    end

    lora_sequence =
        (lora_sequence + 1) % 256

    return true
end


local function update_lora(now)
    local period_ms = LORA.NORMAL_PERIOD_MS

    if flight.state == STATE.ASCENT
        or flight.state == STATE.APOGEE then

        period_ms = LORA.FAST_PERIOD_MS
    end

    if now - last_lora_ms < period_ms then
        return
    end

    send_lora(now)
    last_lora_ms = now
end


-- =====================================================================
-- 21. Main loop
-- =====================================================================

function update()
    local now = millis():toint()

    update_navigation(now)
    update_flight_state(now)
    update_lora(now)

    return update, CONFIG.LOOP_PERIOD_MS
end


-- =====================================================================
-- 22. Startup
-- =====================================================================

lock_parachute()

gcs:send_text(
    6,
    "COMET flight script ready"
)

return update, CONFIG.LOOP_PERIOD_MS