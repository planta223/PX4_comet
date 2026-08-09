-- =====================================================================
-- LoRa Communication
-- =====================================================================
-- LoRa UART를 통해 지상국 명령을 수신하고,
-- 센서 및 비행 상태 정보와 마지막 CMD 처리 결과를
-- 바이너리 텔레메트리 패킷으로 송신한다.
--
-- 별도의 ACK 패킷은 사용하지 않는다.
-- CMD 처리 결과는 이후 송신되는 telemetry에 반복 포함한다.
--
-- 메인 루프 처리 순서:
-- 센서 갱신 → 명령 수신 → 상태머신 처리 → 텔레메트리 송신
-- =====================================================================

local CONFIG = require("comet_config")
local LORA = CONFIG.LORA

local M = {}


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local port = nil

-- 수신 CMD 조립 버퍼
local rx_buffer = {}
local rx_index = 1

-- 상태머신이 아직 처리하지 않은 CMD
local pending_command = nil


-- 마지막으로 처리한 CMD와 결과
-- 동일 sequence + command가 수동 재전송되면
-- 명령을 재실행하지 않고 기존 결과를 telemetry에 다시 포함한다.
local last_ack_sequence = 0
local last_ack_command = nil
local last_ack_result = 0xFF


-- telemetry 송신 상태
local telemetry_sequence = 0
local last_telemetry_ms = 0


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
    EXECUTION_FAILED = 3,

    -- 아직 처리된 CMD가 없음
    NONE = 0xFF
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
    packet[#packet + 1] =
        math.floor(value / 256) % 256
end


local function append_u32(packet, value)
    value = math.floor(value) % 4294967296

    packet[#packet + 1] = value % 256
    packet[#packet + 1] =
        math.floor(value / 256) % 256
    packet[#packet + 1] =
        math.floor(value / 65536) % 256
    packet[#packet + 1] =
        math.floor(value / 16777216) % 256
end


local function append_i32(packet, value)
    value = clamp(
        round(value),
        -2147483648,
        2147483647
    )

    value = value % 4294967296

    packet[#packet + 1] = value % 256
    packet[#packet + 1] =
        math.floor(value / 256) % 256
    packet[#packet + 1] =
        math.floor(value / 65536) % 256
    packet[#packet + 1] =
        math.floor(value / 16777216) % 256
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
        gcs:send_text(
            4,
            "COMET: [LORA] Command CRC error"
        )
        return
    end

    gcs:send_text(
        6,
        "COMET: [LORA] Command CRC valid"
    )

    -- 지원하지 않는 명령값
    if command < M.COMMAND.SERVO_MAIN_LOCK
        or command > M.COMMAND.FORCE_LANDED then

        M.schedule_ack(
            sequence,
            command,
            M.ACK_RESULT.INVALID_COMMAND
        )

        return
    end

    -- 동일 sequence + command가 재수신된 경우
    -- 이미 처리된 명령이므로 다시 실행하지 않는다.
    -- 기존 ACK 결과가 이후 telemetry에 계속 포함된다.
    if last_ack_result ~= M.ACK_RESULT.NONE
        and last_ack_sequence == sequence
        and last_ack_command == command then

        gcs:send_text(
            6,
            "COMET: [LORA] Duplicate command received"
        )

        return
    end

    -- 상태머신이 아직 이전 명령을 처리하지 않았다면
    -- 새로운 명령을 받지 않는다.
    if pending_command ~= nil then
        gcs:send_text(
            4,
            "COMET: [LORA] Command pending"
        )

        return
    end

    pending_command = {
        sequence = sequence,
        command = command
    }
end


local function process_received_byte(byte)
    -- CMD 헤더를 찾을 때까지 바이트를 무시한다.
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

    local received_count = 0

    while port:available() > 0 do
        local byte = port:read()

        if byte == nil then
            break
        end

        received_count = received_count + 1
        process_received_byte(byte)
    end

    -- 통신 시험 종료 후 삭제 가능
    if received_count > 0 then
        gcs:send_text(
            6,
            "COMET: [LORA] RX bytes "
                .. tostring(received_count)
        )
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

    -- 기본 상태
    append_u8(packet, LORA.TELEMETRY_HEADER)
    append_u8(packet, telemetry_sequence)
    append_u8(packet, flight_state or 0)
    append_u8(packet, status_flags or 0)
    append_u8(packet, fault_flags or 0)

    -- 시스템 시간
    append_u32(packet, now)

    -- latitude, longitude: deg × 1e7
    -- altitude: relative m × 100
    append_i32(
        packet,
        (sensor_data.latitude or 0)
            * 10000000
    )

    append_i32(
        packet,
        (sensor_data.longitude or 0)
            * 10000000
    )

    append_i32(
        packet,
        (sensor_data.altitude or 0)
            * 100
    )

    -- NED velocity: m/s × 10
    append_i16(
        packet,
        (sensor_data.velocity_n or 0)
            * 10
    )

    append_i16(
        packet,
        (sensor_data.velocity_e or 0)
            * 10
    )

    append_i16(
        packet,
        (sensor_data.velocity_d or 0)
            * 10
    )

    -- attitude: deg × 100
    append_i16(
        packet,
        (sensor_data.roll or 0)
            * 100
    )

    append_i16(
        packet,
        (sensor_data.pitch or 0)
            * 100
    )

    local yaw =
        normalize_yaw_deg(
            sensor_data.yaw or 0
        )

    append_i16(packet, yaw * 100)

    -- 마지막 CMD 처리 결과
    -- 유효한 CMD가 아직 없으면:
    -- ack_sequence = 0
    -- ack_result = 0xFF
    append_u8(packet, last_ack_sequence)
    append_u8(packet, last_ack_result)

    -- CRC를 제외한 길이가 35바이트인지 검사한다.
    if #packet ~= LORA.TELEMETRY_LENGTH - 1 then
        gcs:send_text(
            3,
            "COMET: [LORA] Invalid telemetry packet length"
        )

        return false
    end

    append_u8(
        packet,
        calculate_crc8(
            packet,
            LORA.TELEMETRY_LENGTH - 1
        )
    )

    return send_packet(packet)
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

function M.initialize()
    port =
        serial:find_serial(
            LORA.SERIAL_INSTANCE
        )

    if port == nil then
        gcs:send_text(
            3,
            "COMET: [LORA] Initialization failed"
        )

        return false
    end

    port:begin(LORA.BAUD)
    port:set_flow_control(0)

    rx_buffer = {}
    rx_index = 1
    pending_command = nil

    last_ack_sequence = 0
    last_ack_command = nil
    last_ack_result = M.ACK_RESULT.NONE

    telemetry_sequence = 0
    last_telemetry_ms = 0

    gcs:send_text(
        6,
        "COMET: [LORA] Initialized"
    )

    return true
end


function M.receive()
    return receive_commands()
end


-- CMD 처리 결과를 저장한다.
-- 저장된 결과는 이후 telemetry에 반복 포함된다.
function M.schedule_ack(sequence, command, result)
    if port == nil then
        return false
    end

    if sequence == nil
        or command == nil
        or result == nil then

        gcs:send_text(
            3,
            "COMET: [LORA] Invalid ACK result"
        )

        return false
    end

    last_ack_sequence = sequence % 256
    last_ack_command = command
    last_ack_result = result % 256

    return true
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

    if now - last_telemetry_ms
        < CONFIG.TELEMETRY_PERIOD_MS then

        return true
    end

    local sent =
        send_telemetry(
            now,
            sensor_data,
            flight_state,
            status_flags,
            fault_flags
        )

    if not sent then
        return false
    end

    last_telemetry_ms = now

    telemetry_sequence =
        (telemetry_sequence + 1) % 256

    return true
end


function M.update_tx(
    now,
    sensor_data,
    flight_state,
    status_flags,
    fault_flags
)
    -- 별도 ACK 패킷은 송신하지 않는다.
    -- 마지막 CMD 처리 결과는 telemetry에 포함된다.
    return M.send_telemetry_if_due(
        now,
        sensor_data,
        flight_state,
        status_flags,
        fault_flags
    )
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