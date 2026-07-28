-- =====================================================================
-- LoRa Communication
-- =====================================================================
-- LoRa UART를 통해 지상국 명령을 수신하고 ACK를 반환하며,
-- 센서 및 비행 상태 정보를 바이너리 텔레메트리 패킷으로 송신한다.
--
-- 명령 수신과 텔레메트리 송신을 분리하여 메인 루프에서 다음 순서로
-- 처리할 수 있도록 한다.
--
-- 센서 갱신 → 명령 수신 → 상태머신 처리 → 텔레메트리 송신
-- =====================================================================

local CONFIG = require("comet_config")
local LORA = CONFIG.LORA

local M = {}


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local port = nil

local rx_buffer = {}
local rx_index = 1
local pending_command = nil

local last_ack_sequence = nil
local last_ack_command = nil
local last_ack_result = nil

local telemetry_sequence = 0
local last_send_ms = 0


-- ---------------------------------------------------------------------
-- Protocol definitions
-- ---------------------------------------------------------------------

M.COMMAND = {
    SERVO_MAIN_LOCK = 1,
    SERVO_MAIN_RELEASE = 2,
    SERVO_CANDIDATE_LOCK = 3,
    SERVO_CANDIDATE_RELEASE = 4,
    LOCK_ALL = 5,
    RELEASE_ALL = 6,

    FORCE_ASCENT = 7,
    FORCE_DEPLOY = 8,
    FORCE_DESCENT = 9,
    FORCE_LANDED = 10
}

M.ACK_RESULT = {
    OK = 0,
    INVALID_COMMAND = 1,
    NOT_ALLOWED = 2,
    EXECUTION_FAILED = 3
}


-- ---------------------------------------------------------------------
-- Utility functions
-- ---------------------------------------------------------------------

-- CRC-8/ATM 다항식 0x07을 사용하여 CRC를 계산한다.
local function calculate_crc8(bytes, length)
    local crc = 0x00

    for i = 1, length do
        crc = crc ~ bytes[i]

        for _ = 1, 8 do
            if (crc & 0x80) ~= 0 then
                crc = ((crc << 1) ~ 0x07) & 0xFF
            else
                crc = (crc << 1) & 0xFF
            end
        end
    end

    return crc
end


-- 바이트 배열을 UART로 전송한다.
local function send_packet(packet)
    if port == nil then
        return false
    end

    for i = 1, #packet do
        port:write(packet[i])
    end

    return true
end


-- 값을 지정된 범위로 제한한다.
local function clamp(value, min_value, max_value)
    if value < min_value then
        return min_value
    end

    if value > max_value then
        return max_value
    end

    return value
end


-- 실수를 가장 가까운 정수로 반올림한다.
local function round(value)
    if value >= 0 then
        return math.floor(value + 0.5)
    end

    return math.ceil(value - 0.5)
end


-- yaw를 -180 ~ +180 deg 범위로 정규화한다.
local function normalize_yaw_deg(yaw)
    while yaw > 180.0 do
        yaw = yaw - 360.0
    end

    while yaw < -180.0 do
        yaw = yaw + 360.0
    end

    return yaw
end


-- ---------------------------------------------------------------------
-- Packet encoding
-- ---------------------------------------------------------------------

local function append_u8(packet, value)
    packet[#packet + 1] = value % 256
end


local function append_i16(packet, value)
    value = clamp(round(value), -32768, 32767)
    value = value % 65536

    packet[#packet + 1] = value % 256
    packet[#packet + 1] = math.floor(value / 256) % 256
end


local function append_u32(packet, value)
    value = math.floor(value) % 4294967296

    packet[#packet + 1] = value % 256
    packet[#packet + 1] = math.floor(value / 256) % 256
    packet[#packet + 1] = math.floor(value / 65536) % 256
    packet[#packet + 1] = math.floor(value / 16777216) % 256
end


local function append_i32(packet, value)
    value = clamp(round(value), -2147483648, 2147483647)
    value = value % 4294967296

    packet[#packet + 1] = value % 256
    packet[#packet + 1] = math.floor(value / 256) % 256
    packet[#packet + 1] = math.floor(value / 65536) % 256
    packet[#packet + 1] = math.floor(value / 16777216) % 256
end


-- ---------------------------------------------------------------------
-- Command reception
-- ---------------------------------------------------------------------

local function process_command_packet()
    local header = rx_buffer[1]
    local sequence = rx_buffer[2]
    local command = rx_buffer[3]
    local received_crc = rx_buffer[4]

    if header ~= LORA.COMMAND_HEADER then
        return
    end

    local expected_crc =
        calculate_crc8(
            rx_buffer,
            LORA.COMMAND_LENGTH - 1
        )

    if received_crc ~= expected_crc then
        gcs:send_text(4, "COMET: [LORA] Command CRC error")
        return
    end

    if command < M.COMMAND.SERVO_MAIN_LOCK
        or command > M.COMMAND.FORCE_LANDED then

        M.send_ack(
            sequence,
            command,
            M.ACK_RESULT.INVALID_COMMAND
        )
        return
    end

    -- 이미 처리한 동일 명령이 재전송되면 기존 ACK만 다시 전송한다.
    if last_ack_sequence == sequence
        and last_ack_command == command then

        M.send_ack(
            sequence,
            command,
            last_ack_result
        )
        return
    end

    -- 상태머신이 아직 이전 명령을 처리하지 않았다면 새 명령을 보류한다.
    if pending_command ~= nil then
        return
    end

    pending_command = {
        sequence = sequence,
        command = command
    }
end


local function process_received_byte(byte)
    if rx_index == 1
        and byte ~= LORA.COMMAND_HEADER then

        return
    end

    rx_buffer[rx_index] = byte
    rx_index = rx_index + 1

    if rx_index > LORA.COMMAND_LENGTH then
        process_command_packet()

        rx_buffer = {}
        rx_index = 1
    end
end


local function receive_commands()
    if port == nil then
        return false
    end

    while port:available() > 0 do
        local byte = port:read()

        if byte == nil then
            break
        end

        process_received_byte(byte)
    end

    return true
end


-- ---------------------------------------------------------------------
-- Telemetry transmission
-- ---------------------------------------------------------------------

local function send_telemetry(
    now,
    sensor_data,
    flight_state,
    status_flags,
    fault_flags
)
    if port == nil or sensor_data == nil then
        return false
    end

    local packet = {}

    append_u8(packet, LORA.TELEMETRY_HEADER)
    append_u8(packet, telemetry_sequence)
    append_u8(packet, flight_state or 0)
    append_u8(packet, status_flags or 0)
    append_u8(packet, fault_flags or 0)
    append_u32(packet, now)

    -- latitude, longitude: deg × 1e7
    -- altitude: relative m × 100
    append_i32(
        packet,
        (sensor_data.latitude or 0) * 10000000
    )

    append_i32(
        packet,
        (sensor_data.longitude or 0) * 10000000
    )

    append_i32(
        packet,
        (sensor_data.altitude or 0) * 100
    )

    -- NED velocity: m/s × 10
    append_i16(
        packet,
        (sensor_data.velocity_n or 0) * 10
    )

    append_i16(
        packet,
        (sensor_data.velocity_e or 0) * 10
    )

    append_i16(
        packet,
        (sensor_data.velocity_d or 0) * 10
    )

    -- attitude: deg × 100
    append_i16(
        packet,
        (sensor_data.roll or 0) * 100
    )

    append_i16(
        packet,
        (sensor_data.pitch or 0) * 100
    )

    local yaw =
        normalize_yaw_deg(sensor_data.yaw or 0)

    append_i16(packet, yaw * 100)

    if #packet ~= LORA.TELEMETRY_LENGTH - 1 then
        gcs:send_text(
            3,
            "COMET: [LORA] Invalid telemetry packet length"
        )
        return false
    end

    packet[#packet + 1] =
        calculate_crc8(
            packet,
            LORA.TELEMETRY_LENGTH - 1
        )

    return send_packet(packet)
end


local function get_telemetry_period(flight_state)
    -- 비행 중에는 빠른 주기를 사용한다.
    if flight_state == 1
        or flight_state == 2
        or flight_state == 3 then

        return LORA.FAST_PERIOD_MS
    end

    return LORA.NORMAL_PERIOD_MS
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize()
    port = serial:find_serial(LORA.SERIAL_INSTANCE)

    if port == nil then
        gcs:send_text(3, "COMET: [LORA] Initialization failed")
        return false
    end

    port:begin(LORA.BAUD)
    port:set_flow_control(0)

    rx_buffer = {}
    rx_index = 1
    pending_command = nil

    last_ack_sequence = nil
    last_ack_command = nil
    last_ack_result = nil

    telemetry_sequence = 0
    last_send_ms = 0

    gcs:send_text(6, "COMET: [LORA] Initialized")
    return true
end


function M.receive()
    return receive_commands()
end


function M.send_ack(sequence, command, result)
    if port == nil then
        return false
    end

    local packet = {
        LORA.ACK_HEADER,
        sequence,
        result
    }

    packet[LORA.ACK_LENGTH] =
        calculate_crc8(
            packet,
            LORA.ACK_LENGTH - 1
        )

    local sent = send_packet(packet)

    if sent then
        last_ack_sequence = sequence
        last_ack_command = command
        last_ack_result = result
    end

    return sent
end


function M.send_telemetry_if_due(
    now,
    sensor_data,
    flight_state,
    status_flags,
    fault_flags
)
    if port == nil then
        return false
    end

    local period =
        get_telemetry_period(flight_state)

    if now - last_send_ms < period then
        return true
    end

    local sent = send_telemetry(
        now,
        sensor_data,
        flight_state,
        status_flags,
        fault_flags
    )

    if sent then
        last_send_ms = now
        telemetry_sequence =
            (telemetry_sequence + 1) % 256
    end

    return sent
end


function M.get_command()
    if pending_command == nil then
        return nil
    end

    local command = pending_command
    pending_command = nil

    return command
end


return M
