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

-- 수신 CMD 조립 버퍼
local rx_buffer = {}
local rx_index = 1

-- 상태머신이 아직 처리하지 않은 CMD
local pending_command = nil


-- 아직 송신되지 않은 ACK
local pending_ack = nil

-- ACK 송신 예정 시각
local ack_due_ms = nil

-- CMD 처리 이후 다음 telemetry 송신을 기다리는 상태
local ack_wait_for_next_telemetry = false

-- ACK가 보호구간에 걸려 연기됐는지 여부
local ack_deferred = false


-- 마지막으로 처리한 CMD와 결과
-- 동일 sequence + command가 수동 재전송됐을 때
-- 명령을 재실행하지 않고 기존 ACK만 다시 예약하기 위해 유지
local last_ack_sequence = nil
local last_ack_command = nil
local last_ack_result = nil


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

-- 이미 처리한 동일 CMD가 재전송된 경우,
-- 명령을 다시 실행하지 않고 기존 ACK 결과만 재예약한다.
local function reschedule_last_ack()
    if last_ack_sequence == nil then
        return false
    end

    pending_ack = {
        sequence = last_ack_sequence,
        command = last_ack_command,
        result = last_ack_result
    }

    ack_due_ms = nil
    ack_wait_for_next_telemetry = true
    ack_deferred = false

    return true
end


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

    gcs:send_text(
        6,
        "COMET: [LORA] Command CRC valid"
    )

    if command < M.COMMAND.SERVO_MAIN_LOCK
        or command > M.COMMAND.FORCE_LANDED then

        M.schedule_ack(
            sequence,
            command,
            M.ACK_RESULT.INVALID_COMMAND
        )
        return
    end

    -- 이미 처리한 동일 명령이 재전송되면 기존 ACK만 다시 전송한다.
    if last_ack_sequence == sequence
        and last_ack_command == command then

        -- 명령은 다시 실행하지 않고 기존 ACK만 재예약한다.
        reschedule_last_ack()
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

    local received_count = 0

    while port:available() > 0 do
        local byte = port:read()

        if byte == nil then
            break
        end

        received_count = received_count + 1
        process_received_byte(byte)
    end

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


-- ACK 패킷을 실제 UART로 송신한다.
local function send_ack_packet(sequence, result)
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

    return send_packet(packet)
end


-- 예약된 ACK를 지정된 슬롯에서 송신한다.
local function send_ack_if_due(now)
    if pending_ack == nil then
        return true
    end

    -- CMD 처리 후 아직 다음 telemetry가 송신되지 않았다.
    if ack_wait_for_next_telemetry then
        return true
    end

    if ack_due_ms == nil or now < ack_due_ms then
        return true
    end

    local next_telemetry_ms =
        last_telemetry_ms
        + CONFIG.TELEMETRY_PERIOD_MS

    local time_until_next_telemetry =
        next_telemetry_ms - now

    -- 다음 telemetry 직전 보호구간이면 ACK를 연기한다.
    if time_until_next_telemetry
        <= CONFIG.ACK_GUARD_BEFORE_TELEMETRY_MS then

        if not ack_deferred then
            ack_deferred = true
            ack_wait_for_next_telemetry = true
            ack_due_ms = nil

            gcs:send_text(
                4,
                "COMET: [LORA] ACK deferred"
            )

            return true
        end

        -- 이미 한 번 연기된 ACK라면 폐기한다.
        gcs:send_text(
            4,
            "COMET: [LORA] ACK slot missed"
        )

        pending_ack = nil
        ack_due_ms = nil
        ack_wait_for_next_telemetry = false
        ack_deferred = false

        -- last_ack_*는 유지한다.
        return false
    end

    local sent =
        send_ack_packet(
            pending_ack.sequence,
            pending_ack.result
        )

    if sent then
        pending_ack = nil
        ack_due_ms = nil
        ack_wait_for_next_telemetry = false
        ack_deferred = false
    end

    return sent
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

    pending_ack = nil
    ack_due_ms = nil
    ack_wait_for_next_telemetry = false
    ack_deferred = false

    last_ack_sequence = nil
    last_ack_command = nil
    last_ack_result = nil

    telemetry_sequence = 0
    last_telemetry_ms = 0

    gcs:send_text(6, "COMET: [LORA] Initialized")
    return true
end


function M.receive()
    return receive_commands()
end


-- CMD 처리 결과를 저장하고 ACK 송신을 예약한다.
function M.schedule_ack(sequence, command, result)
    if port == nil then
        return false
    end

    -- 마지막 처리 결과 저장
    last_ack_sequence = sequence
    last_ack_command = command
    last_ack_result = result

    -- 실제 ACK 송신은 다음 telemetry 이후 수행
    pending_ack = {
        sequence = sequence,
        command = command,
        result = result
    }

    ack_due_ms = nil
    ack_wait_for_next_telemetry = true
    ack_deferred = false

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

    local sent = send_telemetry(
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

    -- CMD 처리 이후 처음 송신된 telemetry를 기준으로
    -- ACK 송신 시점을 설정한다.
    if pending_ack ~= nil
        and ack_wait_for_next_telemetry then

        ack_wait_for_next_telemetry = false

        ack_due_ms =
            now
            + CONFIG.ACK_DELAY_AFTER_TELEMETRY_MS
    end

    return true
end


function M.update_tx(
    now,
    sensor_data,
    flight_state,
    status_flags,
    fault_flags
)
    -- 동일 주기에 telemetry와 ACK가 모두 due이면
    -- telemetry를 먼저 처리한다.
    local telemetry_ok =
        M.send_telemetry_if_due(
            now,
            sensor_data,
            flight_state,
            status_flags,
            fault_flags
        )

    local ack_ok =
        send_ack_if_due(now)

    return telemetry_ok and ack_ok
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
