-- =====================================================================
-- COMET LoRa Throughput Test
-- Pixhawk / ArduPilot Lua
--
-- Control loop : 10 ms
-- Packet length: 34 bytes
-- TX period    : 100 ms (10 Hz)
-- UART         : 9600 baud, 8-N-1
-- =====================================================================

local SERIAL_INSTANCE = 0
local UART_BAUD = 9600

local LOOP_PERIOD_MS = 10
local TX_PERIOD_MS = 100

local HEADER = 0xA5
local MARKER = 0x5A
local PACKET_LENGTH = 34
local TEST_PAYLOAD_LENGTH = 24
local REPORT_PERIOD_MS = 5000

local port = nil
local sequence = 0
local packet_counter = 0

local last_tx_ms = 0
local last_loop_ms = 0
local last_report_ms = 0

local tx_attempt_packets = 0
local tx_complete_packets = 0
local tx_partial_packets = 0
local tx_written_bytes = 0

-- CRC-8/ATM
-- poly=0x07, init=0x00, refin=false, refout=false, xorout=0x00
local function crc8_atm(bytes, count)
    local crc = 0

    for i = 1, count do
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

local function append_u16_le(bytes, value)
    value = value & 0xFFFF
    bytes[#bytes + 1] = value & 0xFF
    bytes[#bytes + 1] = (value >> 8) & 0xFF
end

local function append_u32_le(bytes, value)
    value = value & 0xFFFFFFFF
    bytes[#bytes + 1] = value & 0xFF
    bytes[#bytes + 1] = (value >> 8) & 0xFF
    bytes[#bytes + 1] = (value >> 16) & 0xFF
    bytes[#bytes + 1] = (value >> 24) & 0xFF
end

local function bytes_to_string(bytes)
    local chars = {}

    for i = 1, #bytes do
        chars[i] = string.char(bytes[i])
    end

    return table.concat(chars)
end

-- Fixed 34-byte test packet:
--
-- Byte  0      Header             uint8       0xA5
-- Byte  1      Sequence           uint8       0..255
-- Byte  2..5   Packet counter     uint32 LE
-- Byte  6..7   Loop delta         uint16 LE   ms
-- Byte  8..31  Test payload       24 bytes    sequence-based pattern
-- Byte 32      Marker             uint8       0x5A
-- Byte 33      CRC-8/ATM          uint8       over bytes 0..32
local function make_packet(loop_delta_ms)
    local bytes = {}

    bytes[#bytes + 1] = HEADER
    bytes[#bytes + 1] = sequence

    append_u32_le(bytes, packet_counter)
    append_u16_le(bytes, loop_delta_ms)

    -- Changing payload pattern verifies that all 24 data bytes arrive intact.
    for i = 0, TEST_PAYLOAD_LENGTH - 1 do
        bytes[#bytes + 1] = (sequence + i) & 0xFF
    end

    bytes[#bytes + 1] = MARKER
    bytes[#bytes + 1] = crc8_atm(bytes, PACKET_LENGTH - 1)

    if #bytes ~= PACKET_LENGTH then
        gcs:send_text(
            3,
            string.format(
                "COMET TEST: invalid packet length=%d expected=%d",
                #bytes,
                PACKET_LENGTH
            )
        )
        return nil
    end

    return bytes_to_string(bytes)
end

local function initialize()
    port = serial:find_serial(SERIAL_INSTANCE)

    if port == nil then
        gcs:send_text(3, "COMET TEST: scripting serial port not found")
        return false
    end

    port:begin(UART_BAUD)
    port:set_flow_control(0)

    local now = millis():toint()
    last_tx_ms = now
    last_loop_ms = now
    last_report_ms = now

    gcs:send_text(
        6,
        string.format(
            "COMET TEST: baud=%d packet=%dB rate=10Hz",
            UART_BAUD,
            PACKET_LENGTH
        )
    )

    return true
end

local function send_test_packet(loop_delta_ms)
    local packet = make_packet(loop_delta_ms)

    if packet == nil then
        return
    end

    tx_attempt_packets = tx_attempt_packets + 1

    local written = port:writestring(packet)

    if written == nil then
        written = 0
    end

    tx_written_bytes = tx_written_bytes + written

    if written == PACKET_LENGTH then
        tx_complete_packets = tx_complete_packets + 1
    else
        tx_partial_packets = tx_partial_packets + 1
    end

    sequence = (sequence + 1) & 0xFF
    packet_counter = (packet_counter + 1) & 0xFFFFFFFF
end

local function report(now)
    if (now - last_report_ms) < REPORT_PERIOD_MS then
        return
    end

    local elapsed_s = (now - last_report_ms) / 1000.0
    local accepted_rate = 0

    if elapsed_s > 0 then
        accepted_rate = tx_written_bytes / elapsed_s
    end

    gcs:send_text(
        6,
        string.format(
            "COMET TEST: try=%d full=%d partial=%d accepted=%.1fB/s",
            tx_attempt_packets,
            tx_complete_packets,
            tx_partial_packets,
            accepted_rate
        )
    )

    tx_attempt_packets = 0
    tx_complete_packets = 0
    tx_partial_packets = 0
    tx_written_bytes = 0
    last_report_ms = now
end

local function update()
    local now = millis():toint()
    local loop_delta_ms = now - last_loop_ms
    last_loop_ms = now

    if (now - last_tx_ms) >= TX_PERIOD_MS then
        last_tx_ms = last_tx_ms + TX_PERIOD_MS

        -- Avoid a burst if the script was delayed for more than one TX period.
        if (now - last_tx_ms) >= TX_PERIOD_MS then
            last_tx_ms = now
        end

        send_test_packet(loop_delta_ms)
    end

    report(now)

    return update, LOOP_PERIOD_MS
end

if not initialize() then
    return
end

return update, LOOP_PERIOD_MS
