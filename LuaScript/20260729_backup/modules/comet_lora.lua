-- =====================================================================
-- LoRa Communication
-- =====================================================================
-- LoRa UART를 통해 지상국 명령을 수신하고 ACK를 반환하며,
-- 센서 및 비행 상태 정보를 바이너리 텔레메트리 패킷으로 송신한다.
--
-- 명령은 comet_command_queue.lua의 공통 FIFO 큐에 적재되며, SiK/MAVLink
-- 경로(comet_mavlink.lua)와 큐를 공유한다. 비행 상태머신은 이 큐만
-- 소비하므로 두 경로의 명령이 동일하게 처리된다.
--
-- 센서 갱신 → 명령 수신(LoRa/MAVLink 각각 큐에 적재) → 상태머신 처리
-- (큐 소비) → 텔레메트리 송신
-- =====================================================================

local CONFIG = require("comet_config")
local LORA = CONFIG.LORA
local queue = require("comet_command_queue")

local M = {}


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

local port = nil

local rx_buffer = {}
local rx_index = 1

-- 재전송된 동일 명령에 대해 중복으로 큐에 넣지 않고 마지막 결과를
-- 재전송하기 위한 상태. last_result는 아직 상태머신이 처리하지 않았으면
-- nil이고(수신 ACK만 다시 보냄), 처리 완료 후 결과값으로 채워진다.
local last_received_sequence = nil
local last_received_command = nil
local last_result = nil

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

    -- 원격 강제 상태전이 (description_flight_state_machine.txt의 각 전이별
    -- "원격 강제전이" 항목). 각 명령은 바로 이전 상태에서만 허용된다
    -- (is_command_allowed() 참고).
    FORCE_ASCENT = 7,
    FORCE_DEPLOY = 8,
    FORCE_DESCENT = 9,
    FORCE_LANDED = 10,

    RESET = 11
}

M.ACK_RESULT = {
    OK = 0,
    INVALID_COMMAND = 1,
    NOT_ALLOWED = 2,
    EXECUTION_FAILED = 3,
    RECEIVED = 4  -- CRC/패킷 검증 통과, 큐에 적재됨 (실행 결과는 아직 아님)
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
        gcs:send_text(4, "COMET: LoRa command CRC error")
        return
    end

    gcs:send_text(
        6,
        "COMET: Command received (LORA seq="
            .. tostring(sequence) .. " cmd="
            .. tostring(command) .. ")"
    )

    if command < M.COMMAND.SERVO_MAIN_LOCK
        or command > M.COMMAND.RESET then

        M.send_result(
            sequence,
            command,
            M.ACK_RESULT.INVALID_COMMAND
        )
        return
    end

    -- 이미 수신한 동일 명령이 재전송되면 새로 큐에 넣지 않고, 지금까지
    -- 알고 있는 결과(아직 처리 전이면 수신 ACK, 처리됐으면 결과)를
    -- 다시 보낸다.
    if last_received_sequence == sequence
        and last_received_command == command then

        if last_result ~= nil then
            M.send_result(sequence, command, last_result)
        else
            M.send_ack(sequence, command, M.ACK_RESULT.RECEIVED)
        end
        return
    end

    last_received_sequence = sequence
    last_received_command = command
    last_result = nil

    -- CRC/패킷 검증이 끝났으므로 실행 결과와 무관하게 즉시 수신 ACK을
    -- 보낸다. 실행 결과(OK/NOT_ALLOWED/EXECUTION_FAILED)는 상태머신이
    -- 큐에서 꺼내 처리한 뒤 M.send_result()로 별도 전송한다.
    M.send_ack(sequence, command, M.ACK_RESULT.RECEIVED)

    queue.push({
        source = "LORA",
        sequence = sequence,
        command = command
    })
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
    fault_flags
)
    if port == nil or sensor_data == nil then
        return false
    end

    local packet = {}

    append_u8(packet, LORA.TELEMETRY_HEADER)
    append_u8(packet, telemetry_sequence)
    append_u8(packet, flight_state or 0)
    append_u8(packet, fault_flags or 0)
    append_u32(packet, now)

    -- north, east: HOME 기준 오프셋 m × 100 (cm 단위, int16 -> ±327.67m
    -- 범위를 벗어나면 saturate됨 - append_i16()이 clamp 후 인코딩하므로
    -- wraparound은 안 나지만, 실제 회수 반경이 이보다 크면 그 방향으로
    -- 327.67m에서 값이 눌려붙는다. 발사장 예상 이동거리 확인 후 필요하면
    -- comet_config.lua의 TELEMETRY_LENGTH와 함께 폭을 늘릴 것.
    -- altitude: relative m × 100 (int32, 범위 문제 없음)
    append_i16(
        packet,
        (sensor_data.north or 0) * 100
    )

    append_i16(
        packet,
        (sensor_data.east or 0) * 100
    )

    append_i32(
        packet,
        (sensor_data.altitude or 0) * 100
    )

    if #packet ~= LORA.TELEMETRY_LENGTH - 1 then
        gcs:send_text(
            3,
            "COMET: Invalid telemetry packet length"
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
        gcs:send_text(3, "COMET: LoRa initialization failed")
        return false
    end

    port:begin(LORA.BAUD)
    port:set_flow_control(0)

    rx_buffer = {}
    rx_index = 1

    last_received_sequence = nil
    last_received_command = nil
    last_result = nil

    telemetry_sequence = 0
    last_send_ms = 0

    gcs:send_text(6, "COMET: LoRa initialized")
    return true
end


function M.receive()
    return receive_commands()
end


-- CRC/패킷 검증 직후 즉시 보내는 수신 확인, 또는 재전송된 명령에 대한
-- 재응답. 실행 결과는 담지 않는다.
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

    return send_packet(packet)
end


-- 상태머신이 큐에서 명령을 꺼내 실행한 뒤 그 결과(OK/NOT_ALLOWED/
-- EXECUTION_FAILED)를 전송한다. 재전송 시 재응답할 수 있도록 결과를
-- last_result에 기억해 둔다.
function M.send_result(sequence, command, result)
    local sent = M.send_ack(sequence, command, result)

    if sent
        and last_received_sequence == sequence
        and last_received_command == command then

        last_result = result
    end

    return sent
end


function M.send_telemetry_if_due(
    now,
    sensor_data,
    flight_state,
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
        fault_flags
    )

    if sent then
        last_send_ms = now
        telemetry_sequence =
            (telemetry_sequence + 1) % 256
    end

    return sent
end


return M