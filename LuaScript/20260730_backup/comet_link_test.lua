-- =====================================================================
-- Comet LoRa Link Test (독립 실행용 - 실제 비행 스크립트가 아님)
-- =====================================================================
-- comet_main.lua/comet_lora.lua 등 실제 비행 로직과 완전히 별개인 단독
-- 테스트 스크립트다. FC의 /APM/scripts/ 밑에 이 파일 "하나만" 올려서
-- 돌린다 (comet_main.lua와 같이 올리면 LoRa 포트를 두 스크립트가 동시에
-- 잡으려 해서 충돌한다 - 실제 비행 스크립트는 잠깐 내려두고 테스트할 것).
--
-- 목적: LoRa 링크 자체의 한계를 4단계로 실측한다.
--   1) 단독 송신(TX-only) - 텔레메트리 모양 더미 패킷만 주기적으로 쏨
--   2) 단독 수신(RX-only) - 명령 모양 패킷을 받으면 즉시 ACK만 돌려줌
--   3) 동시 송수신 - TX/RX를 둘 다 켜고 패킷 크기를 바꿔가며 안정성 확인
--   4) 제어주기(LOOP_PERIOD_MS) 자체를 줄여도 스크립팅 엔진이 버티는지 확인
--
-- 아래 CONFIG 값들을 바꿔가며 재업로드/리부트하는 방식으로 스윕한다
-- (실기에서 값 하나 바꿀 때마다 자동 재시작은 안 되므로 수동 스윕).
-- 결과 판정은 GCS 쪽 도구(lora_link_test.py)가 수신 통계로 담당하고,
-- 이 스크립트는 "설정된 대로 정직하게 쏘고 받기"만 한다.
-- =====================================================================

local CONFIG = {
    -- 4) 제어주기 테스트용 - 기본 50ms(20Hz)에서 시작해서 낮춰본다
    -- (10ms 단위 권장: 50 -> 40 -> 30 -> 20 -> 10). 너무 낮추면 콘솔에
    -- "PreArm: Scripting: ..." 류 overrun 경고가 뜨는지 확인할 것.
    LOOP_PERIOD_MS = 20,

    -- 1)/3) 송신(텔레메트리 모양) 테스트
    TX_ENABLED = true,
    TX_PACKET_SIZE = 10,   -- 실제 comet_lora.lua TELEMETRY_LENGTH와 동일(N=10)
    TX_PERIOD_MS = 100,    -- 실제 FAST_PERIOD_MS 기본값(N=350) - 스윕 대상

    -- 2)/3) 수신(명령 모양) 테스트 - 받으면 즉시 ACK
    RX_ENABLED = true,

    SERIAL_INSTANCE = 0,
    BAUD = 9600,

    REPORT_INTERVAL_MS = 5000  -- 이 주기로 누적 통계를 gcs:send_text로 보고
}

-- 실제 프로토콜과 동일한 헤더/길이(comet_config.lua CONFIG.LORA 참고)
local TX_HEADER = 0xA5        -- 텔레메트리 헤더
local RX_HEADER = 0x5A        -- 명령 헤더
local ACK_HEADER = 0xA6       -- ACK 헤더
local RX_LENGTH = 4           -- 실제 COMMAND_LENGTH와 동일(N=4): header+seq+cmd+crc8
local ACK_LENGTH = 4          -- 실제 ACK_LENGTH와 동일(N=4)


local port = nil

local tx_seq = 0
local tx_sent_count = 0
local last_tx_ms = 0

local rx_buffer = {}
local rx_index = 1
local rx_received_count = 0
local rx_bad_crc_count = 0

local last_report_ms = 0


-- ---------------------------------------------------------------------
-- CRC-8 (poly=0x07, init=0x00) - 실제 프로토콜과 동일
-- ---------------------------------------------------------------------
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


local function send_bytes(packet)
    if port == nil then
        return false
    end
    for i = 1, #packet do
        port:write(packet[i])
    end
    return true
end


-- ---------------------------------------------------------------------
-- TX: TX_PACKET_SIZE 바이트짜리 더미 텔레메트리 모양 패킷을 만든다.
-- header(1) + seq(1) + padding(TX_PACKET_SIZE-3) + crc8(1).
-- padding은 그냥 0으로 채운다 - 내용은 중요하지 않고 "이 크기의 패킷을
-- 이 주기로 계속 쏠 수 있는가"만 본다.
-- ---------------------------------------------------------------------
local function send_tx_packet()
    local packet = {}
    packet[1] = TX_HEADER
    packet[2] = tx_seq % 256

    for i = 3, CONFIG.TX_PACKET_SIZE - 1 do
        packet[i] = 0
    end

    packet[CONFIG.TX_PACKET_SIZE] =
        calculate_crc8(packet, CONFIG.TX_PACKET_SIZE - 1)

    if send_bytes(packet) then
        tx_seq = (tx_seq + 1) % 256
        tx_sent_count = tx_sent_count + 1
    end
end


-- ---------------------------------------------------------------------
-- RX: RX_LENGTH 바이트 명령 모양 프레임을 모아서 CRC 확인 후 즉시 ACK.
-- 내용(명령 코드)은 검사하지 않는다 - "받으면 바로 응답한다"만 본다.
-- ---------------------------------------------------------------------
local function send_ack(sequence)
    local packet = { ACK_HEADER, sequence, 0 }
    packet[ACK_LENGTH] = calculate_crc8(packet, ACK_LENGTH - 1)
    send_bytes(packet)
end


local function process_rx_packet()
    local header = rx_buffer[1]
    local sequence = rx_buffer[2]
    local received_crc = rx_buffer[RX_LENGTH]

    if header ~= RX_HEADER then
        return
    end

    local expected_crc = calculate_crc8(rx_buffer, RX_LENGTH - 1)

    if received_crc ~= expected_crc then
        rx_bad_crc_count = rx_bad_crc_count + 1
        return
    end

    rx_received_count = rx_received_count + 1
    send_ack(sequence)
end


local function receive_bytes()
    if port == nil then
        return
    end

    while port:available() > 0 do
        local byte = port:read()
        if byte == nil then
            break
        end

        if rx_index == 1 and byte ~= RX_HEADER then
            -- 재동기화 - 헤더 아니면 버림
        else
            rx_buffer[rx_index] = byte
            rx_index = rx_index + 1

            if rx_index > RX_LENGTH then
                process_rx_packet()
                rx_buffer = {}
                rx_index = 1
            end
        end
    end
end


-- ---------------------------------------------------------------------
-- Main
-- ---------------------------------------------------------------------

local function startup()
    port = serial:find_serial(CONFIG.SERIAL_INSTANCE)

    if port == nil then
        gcs:send_text(3, "LINKTEST: LoRa port not found")
        return false
    end

    port:begin(CONFIG.BAUD)
    port:set_flow_control(0)

    gcs:send_text(
        6,
        "LINKTEST: started (TX="
            .. tostring(CONFIG.TX_ENABLED) .. " size="
            .. tostring(CONFIG.TX_PACKET_SIZE) .. "B period="
            .. tostring(CONFIG.TX_PERIOD_MS) .. "ms, RX="
            .. tostring(CONFIG.RX_ENABLED) .. ", loop="
            .. tostring(CONFIG.LOOP_PERIOD_MS) .. "ms)"
    )

    return true
end


local function update()
    local now = millis():toint()

    if CONFIG.RX_ENABLED then
        receive_bytes()
    end

    if CONFIG.TX_ENABLED
        and now - last_tx_ms >= CONFIG.TX_PERIOD_MS then

        last_tx_ms = now
        send_tx_packet()
    end

    if now - last_report_ms >= CONFIG.REPORT_INTERVAL_MS then
        last_report_ms = now

        gcs:send_text(
            6,
            "LINKTEST: tx_sent=" .. tostring(tx_sent_count)
                .. " rx_ok=" .. tostring(rx_received_count)
                .. " rx_crc_fail=" .. tostring(rx_bad_crc_count)
        )
    end

    return update, CONFIG.LOOP_PERIOD_MS
end


startup()

return update, CONFIG.LOOP_PERIOD_MS
