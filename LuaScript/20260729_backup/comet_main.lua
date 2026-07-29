-- =====================================================================
-- Comet Main
-- =====================================================================
-- 센서, 서보, LoRa/MAVLink 통신 및 비행 상태머신 모듈을 초기화하고,
-- 설정된 주기에 따라 전체 시스템 업데이트 순서를 관리한다.
--
-- LoRa와 SiK(MAVLink) 명령은 comet_command_queue.lua의 공통 FIFO 큐로
-- 모이며, 비행 상태머신이 그 큐 하나만 소비한다 - 두 통신 경로 모두
-- 동일한 상태 전이/서보 제어/Fault 처리/ACK 경로를 거친다.
--
-- 메인 루프 처리 순서:
-- 센서 갱신 → LoRa/MAVLink 명령 수신(큐 적재) → 비행 상태 갱신(큐 소비)
-- → 텔레메트리 송신
-- =====================================================================

local CONFIG = require("comet_config")
local sensor = require("comet_sensor")
local servo = require("comet_servo")
local lora = require("comet_lora")
local flight = require("comet_flight_state")

-- comet_mavlink.lua는 require("MAVLink/mavlink_msgs")에 의존하는데, 이
-- 모듈이 보드에 없으면 require() 자체가 에러를 던진다. 톱레벨에서
-- 무보호로 require하면 그 에러가 여기까지 전파되어 스크립트 전체
-- (LoRa 포함)가 로드 시점에 죽으므로 pcall로 감싼다. comet_mavlink.lua
-- 내부에서도 자체적으로 pcall 처리를 하지만(M.initialize), 그건 이미
-- require가 성공해서 모듈 자체는 로드된 뒤의 얘기라 이중 방어다.
local mavlink_ok, mavlink = pcall(require, "comet_mavlink")

if not mavlink_ok then
    gcs:send_text(
        4,
        "COMET: comet_mavlink module load failed - " .. tostring(mavlink)
    )
    mavlink = {
        initialize = function() return false end,
        receive = function() return false end
    }
end


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local initialized = false


-- ---------------------------------------------------------------------
-- Initialization
-- ---------------------------------------------------------------------

local function startup()
    local now = millis():toint()

    local sensor_ok = sensor.initialize()
    local servo_ok = servo.initialize()
    local lora_ok = lora.initialize()
    local mavlink_ok = mavlink.initialize()
    local flight_ok = flight.initialize(now)

    if not sensor_ok then
        gcs:send_text(3, "COMET: Sensor initialization failed")
    end

    if not servo_ok then
        gcs:send_text(3, "COMET: Servo initialization failed")
    end

    if not lora_ok then
        gcs:send_text(4, "COMET: LoRa unavailable")
    end

    if not mavlink_ok then
        gcs:send_text(4, "COMET: MAVLink command interface unavailable")
    end

    if not flight_ok then
        gcs:send_text(3, "COMET: Flight initialization failed")
    end

    -- 센서, 서보 및 상태머신 초기화 실패는 치명적 오류로 처리한다.
    local critical_ok =
        sensor_ok
        and servo_ok
        and flight_ok

    if not critical_ok then
        gcs:send_text(2, "COMET: Critical initialization failed")
        return false
    end

    -- LoRa는 백업 통신이므로 초기화 실패 시에도 비행 로직은 계속 실행한다.
    if not lora_ok then
        gcs:send_text(4, "COMET: Running without LoRa")
    end

    gcs:send_text(6, "COMET: System initialized")
    return true
end


-- ---------------------------------------------------------------------
-- Main update loop
-- ---------------------------------------------------------------------

local function update()
    local now = millis():toint()

    if not initialized then
        return update, CONFIG.UPDATE_PERIOD_MS
    end

    -- 1. ArduPilot AHRS/EKF 상태 갱신
    sensor.update()

    local sensor_data = sensor.get_data()

    -- 2. LoRa/MAVLink 명령 수신 (둘 다 comet_command_queue.lua 공통 큐로 적재)
    lora.receive()
    mavlink.receive()

    -- 3. 비행 상태 및 낙하산 제어 갱신 (공통 큐 소비)
    flight.update(
        now,
        sensor_data
    )

    -- 4. 갱신된 비행 상태와 위치를 LoRa로 송신 (에러 사유는 fault_flags로)
    lora.send_telemetry_if_due(
        now,
        sensor_data,
        flight.get_state(),
        flight.get_fault_flags()
    )

    return update, CONFIG.UPDATE_PERIOD_MS
end


-- ---------------------------------------------------------------------
-- Script entry point
-- ---------------------------------------------------------------------

initialized = startup()

return update, CONFIG.UPDATE_PERIOD_MS