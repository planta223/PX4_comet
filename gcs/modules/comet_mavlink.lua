-- =====================================================================
-- MAVLink (SiK) Communication
-- =====================================================================
-- SiK 라디오를 통해 들어오는 MAVLink NAMED_VALUE_INT 메시지로 원격 명령을
-- 수신하고, NAMED_VALUE_FLOAT으로 ACK를 응답한다.
--
-- comet_lora.lua와 동일한 명령/ACK 코드 체계(M.COMMAND / M.ACK_RESULT는
-- comet_lora.lua 것을 그대로 참조)를 쓰고, 수신한 명령은 LoRa와 동일하게
-- comet_command_queue.lua의 공통 FIFO 큐에 적재한다. 상태머신은 이 큐만
-- 소비하므로 SiK 명령도 LoRa와 완전히 동일한 경로(상태 전이, 서보 제어,
-- Fault 처리, ACK)를 거친다.
--
-- 원래는 MAVLink/mavlink_msgs(ArduPilot 표준 인코드/디코드 모듈)를 썼는데,
-- 이 보드에서 SCR_HEAP_SIZE를 200000까지 올려도 그 모듈 하나를 로드할
-- 메모리가 없어서("not enough mem", 이후 SCR_HEAP_SIZE 자체를 못 늘려서
-- "Scripting: Unable to allocate memory") 결국 포기했다. mavlink_msgs는
-- 모든 MAVLink 메시지의 인코드/디코드 테이블을 다 담는 무거운 범용
-- 모듈이라, NAMED_VALUE_INT/FLOAT 두 개만 쓰는 우리한텐 낭비가 크다.
--
-- 대신:
--  - 수신(GCS -> FC, COMET_CMD): mavlink:receive_chan()이 주는 원시
--    payload(msg.msgbuf로 가정 - 확인 필요, 아래 참고)를 string.unpack으로
--    직접 해석한다. NAMED_VALUE_INT(msgid=252) payload는 MAVLink 표준
--    필드 재정렬 규칙(크기 내림차순, 같은 크기끼리는 XML 선언 순서 유지,
--    char 배열은 항상 맨 뒤)에 따라
--    time_boot_ms(uint32,4B) + value(int32,4B) + name(char[10],10B)
--    = 18바이트 고정 레이아웃이다.
--  - 송신(FC -> GCS, COMET_ACK): 추가 모듈 없이 ArduPilot 내장
--    gcs:send_named_float(name, value)로 NAMED_VALUE_FLOAT(msgid=251)을
--    보낸다. 우리 패킹값(최대 0xFFFFFF=16777215)은 float32 24비트
--    가수부로 정확히 표현되는 범위 안이라 정밀도 손실이 없다.
--
-- NAMED_VALUE_INT/FLOAT는 name(char[10]) + value(int32 또는 float) +
-- time_boot_ms뿐이라 sequence(0~255)/command(0~255)/result-or-unused
-- (0~255)를 value 하나(3바이트)에 패킹해서 주고받는다 - pack_value()/
-- unpack_value() 참고.
--
-- 확인 필요: mavlink:receive_chan()이 반환하는 메시지 객체의 원시 payload
-- 필드 이름을 .msgbuf로 가정했다. 실기/SITL에서 다른 이름으로 확인되면
-- receive_commands()의 그 한 줄만 고치면 된다.
--
-- comet_main.lua는 이 모듈을 톱레벨에서 require()한다 - 이 파일이 로드
-- 자체에는 실패하지 않도록(예전에 require("mavlink_msgs") 실패가
-- comet_main.lua까지 전파돼 LoRa까지 완전히 먹통이 된 사고가 있었다)
-- MAVLink 관련 준비는 전부 M.initialize() 안에서 pcall로 감싸고, 실패하면
-- mavlink_available=false로 남겨 이후 모든 공개 함수가 조용히 아무 일도
-- 안 하도록(no-op) 한다. LoRa는 이 모듈과 완전히 독립적으로 계속 동작한다.
-- =====================================================================

local CONFIG = require("comet_config")
local MAVLINK_CFG = CONFIG.MAVLINK
local lora = require("comet_lora") -- COMMAND / ACK_RESULT 코드 공유
local queue = require("comet_command_queue")

local M = {}


-- ---------------------------------------------------------------------
-- Internal state
-- ---------------------------------------------------------------------

-- MAVLink common.xml MAVLINK_MSG_ID_NAMED_VALUE_INT - mavlink_msgs 없이
-- 상수로 직접 박아둔다 (get_msgid() 호출이 필요 없어짐).
local NAMED_VALUE_INT_MSGID = 252

local mavlink_available = false

local last_received_sequence = nil
local last_received_command = nil
local last_result = nil


-- ---------------------------------------------------------------------
-- Value packing (sequence << 16 | command << 8 | aux)
-- ---------------------------------------------------------------------

local function pack_value(sequence, command, aux)
    return ((sequence % 256) * 65536)
        + ((command % 256) * 256)
        + (aux % 256)
end


local function unpack_value(value)
    value = math.floor(value)

    local sequence = math.floor(value / 65536) % 256
    local command = math.floor(value / 256) % 256
    local aux = value % 256

    return sequence, command, aux
end


-- ---------------------------------------------------------------------
-- Command reception
-- ---------------------------------------------------------------------

local function process_command_message(sequence, command)
    gcs:send_text(
        6,
        "COMET: Command received (MAVLINK seq="
            .. tostring(sequence) .. " cmd="
            .. tostring(command) .. ")"
    )

    if command < lora.COMMAND.SERVO_MAIN_LOCK
        or command > lora.COMMAND.RESET then

        M.send_result(
            sequence,
            command,
            lora.ACK_RESULT.INVALID_COMMAND
        )
        return
    end

    if last_received_sequence == sequence
        and last_received_command == command then

        if last_result ~= nil then
            M.send_result(sequence, command, last_result)
        else
            M.send_ack(sequence, command, lora.ACK_RESULT.RECEIVED)
        end
        return
    end

    last_received_sequence = sequence
    last_received_command = command
    last_result = nil

    M.send_ack(sequence, command, lora.ACK_RESULT.RECEIVED)

    queue.push({
        source = "MAVLINK",
        sequence = sequence,
        command = command
    })
end


-- msg.id / msg.msgid (field access) both came back nil on hardware, and
-- pairs(msg) found nothing - msg is userdata with method-style access
-- (msg:method()), not a plain table. Probe a list of plausible method
-- names by calling them (via msg[name](msg), equivalent to msg:name())
-- inside pcall, and report which ones actually return a value. English
-- only - Korean text was getting mangled when STATUSTEXT chunking split
-- multi-byte UTF-8 sequences. Diagnostic only, delete once the real
-- accessor names are confirmed.
local ID_METHOD_CANDIDATES = {"id", "msgid", "get_msgid", "message_id"}
local BUFFER_METHOD_CANDIDATES = {"buffer", "msgbuf", "data", "string", "bytes", "payload", "get_buffer"}

local function probe_msg_methods(msg)
    local found = {}

    for _, name in ipairs(ID_METHOD_CANDIDATES) do
        local ok, v = pcall(function() return msg[name](msg) end)
        if ok and v ~= nil then
            found[#found + 1] = "id:" .. name .. "()=" .. tostring(v)
        end
    end

    for _, name in ipairs(BUFFER_METHOD_CANDIDATES) do
        local ok, v = pcall(function() return msg[name](msg) end)
        if ok and v ~= nil then
            found[#found + 1] = "buf:" .. name .. "()=" .. type(v)
        end
    end

    if #found == 0 then
        return "(no candidate method worked - msg type=" .. type(msg) .. ")"
    end

    return table.concat(found, "; ")
end


-- NAMED_VALUE_INT raw payload (18 bytes: time_boot_ms u32 + value i32 +
-- name char[10]) decoded directly, checked against our COMET_CMD name.
--
-- Diagnostic: probe_msg_methods() dumps which accessor actually works on
-- this board's mavlink message object, since both msg.msgid/msg.msgbuf
-- and msg.id/msg.buffer field access came back nil on hardware. Delete
-- probe_msg_methods() and this log once the real accessor is confirmed
-- and hardcoded.
local function try_decode_command(msg)
    gcs:send_text(6, "COMET: MAVLink msg probe: " .. probe_msg_methods(msg))
end


local function receive_commands()
    local seen_any = false

    for _ = 1, MAVLINK_CFG.POLL_MSG_COUNT do
        local ok, msg = pcall(function() return mavlink:receive_chan() end)

        if not ok then
            gcs:send_text(4, "COMET: MAVLink receive_chan failed - " .. tostring(msg))
            break
        end

        if msg == nil then
            break
        end

        seen_any = true
        try_decode_command(msg)
    end

    return seen_any
end


-- ---------------------------------------------------------------------
-- Public functions
-- ---------------------------------------------------------------------

-- mavlink:init()/register_rx_msgid()를 pcall로 감싼다. 실패하면
-- mavlink_available이 false로 남고, 이후 M.receive()/M.send_ack()/
-- M.send_result()는 아무 것도 하지 않고 조용히 리턴한다 - LoRa 쪽은 이
-- 실패와 무관하게 정상 동작한다.
function M.initialize()
    local ok, err = pcall(function()
        -- 큐 길이 10, 메시지 사이즈 1 (확인 필요: 보드 리소스에 맞게 조정)
        mavlink:init(10, 1)
        mavlink:register_rx_msgid(NAMED_VALUE_INT_MSGID)
    end)

    if not ok then
        mavlink_available = false
        gcs:send_text(
            4,
            "COMET: MAVLink command interface unavailable - " .. tostring(err)
        )
        return false
    end

    mavlink_available = true

    last_received_sequence = nil
    last_received_command = nil
    last_result = nil

    gcs:send_text(6, "COMET: MAVLink command interface initialized")
    return true
end


-- 진단용 - mavlink:receive_chan()이 아예 아무것도 못 받고 있는지(등록 자체가
-- 안 됐거나 GCS 패킷이 안 오는 경우) vs 뭔가는 받는데 파싱/이름에서 걸리는
-- 건지 구분하기 위해, 5초에 한 번씩 그 사이 메시지를 하나라도 봤는지
-- 보고한다. 원인이 특정되면 이 카운터/로그는 지워도 된다.
local no_message_report_last_ms = 0
local seen_any_since_report = false
local NO_MESSAGE_REPORT_INTERVAL_MS = 5000

function M.receive()
    if not mavlink_available then
        return false
    end

    if receive_commands() then
        seen_any_since_report = true
    end

    local now = millis():toint()
    if now - no_message_report_last_ms >= NO_MESSAGE_REPORT_INTERVAL_MS then
        no_message_report_last_ms = now
        if not seen_any_since_report then
            gcs:send_text(4, "COMET: MAVLink no message received recently")
        end
        seen_any_since_report = false
    end

    return true
end


-- gcs:send_named_float()는 ArduPilot 내장 함수라 추가 모듈/메모리가
-- 필요 없다. NAMED_VALUE_FLOAT(msgid=251)로 보내진다.
function M.send_ack(sequence, command, result)
    if not mavlink_available then
        return false
    end

    local ok = pcall(function()
        local value = pack_value(sequence, command, result)
        gcs:send_named_float(MAVLINK_CFG.ACK_NAME, value)
    end)

    return ok
end


function M.send_result(sequence, command, result)
    local sent = M.send_ack(sequence, command, result)

    if sent
        and last_received_sequence == sequence
        and last_received_command == command then

        last_result = result
    end

    return sent
end


return M
