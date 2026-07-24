-- =====================================================================
-- Comet Main
-- =====================================================================
-- 센서, 서보, LoRa 및 비행 상태머신 모듈을 초기화하고,
-- 설정된 주기에 따라 전체 시스템 업데이트 순서를 관리한다.
--
-- 메인 루프 처리 순서:
-- 센서 갱신 → LoRa 명령 수신 → 비행 상태 갱신 → 텔레메트리 송신
-- =====================================================================

local CONFIG = require("comet_config")
local sensor = require("comet_sensor")
local servo = require("comet_servo")
local lora = require("comet_lora")
local flight = require("comet_flight_state")


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
    local status_flags = sensor.get_status_flags()

    -- 2. LoRa 명령 수신
    lora.receive()

    -- 3. 비행 상태 및 낙하산 제어 갱신
    flight.update(
        now,
        sensor_data
    )

    -- 4. 갱신된 비행 상태와 센서 정보를 LoRa로 송신
    lora.send_telemetry_if_due(
        now,
        sensor_data,
        flight.get_state(),
        status_flags
    )

    return update, CONFIG.UPDATE_PERIOD_MS
end


-- ---------------------------------------------------------------------
-- Script entry point
-- ---------------------------------------------------------------------

initialized = startup()

return update, CONFIG.UPDATE_PERIOD_MS