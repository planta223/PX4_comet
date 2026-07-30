-- =====================================================================
-- Command Queue
-- =====================================================================
-- LoRa 원격 명령을 적재하는 FIFO 링버퍼. 비행 상태머신
-- (comet_flight_state.lua)은 이 큐만 소비한다.
--
-- 이전에는 comet_lora.lua가 pending_command 단일 변수로 명령을 들고
-- 있다가 상태머신이 처리하기 전에 새 명령이 오면 그냥 버렸다
-- (FORCE_ASCENT -> FORCE_DEPLOY 같은 연속 명령이 유실될 수 있었음).
-- 이제는 큐가 가득 찰 때만 새 명령을 버리고, 그 경우도 로그를 남긴다.
-- =====================================================================

local CONFIG = require("comet_config")
local CAPACITY = CONFIG.COMMAND_QUEUE.CAPACITY

local M = {}

-- 1-based 고정 크기 배열을 링버퍼로 사용한다.
local buffer = {}
local head = 1  -- 다음에 pop할 위치
local tail = 1  -- 다음에 push할 위치
local count = 0


function M.reset()
    buffer = {}
    head = 1
    tail = 1
    count = 0
end


-- entry: { source = "LORA", sequence = n, command = n }
function M.push(entry)
    if count >= CAPACITY then
        gcs:send_text(
            4,
            "COMET: Command queue full - dropped ("
                .. tostring(entry.source) .. " cmd="
                .. tostring(entry.command) .. ")"
        )
        return false
    end

    buffer[tail] = entry
    tail = (tail % CAPACITY) + 1
    count = count + 1

    gcs:send_text(
        6,
        "COMET: Command queued ("
            .. tostring(entry.source) .. " seq="
            .. tostring(entry.sequence) .. " cmd="
            .. tostring(entry.command) .. ", depth="
            .. tostring(count) .. ")"
    )

    return true
end


function M.pop()
    if count == 0 then
        return nil
    end

    local entry = buffer[head]
    buffer[head] = nil
    head = (head % CAPACITY) + 1
    count = count - 1

    return entry
end


function M.count()
    return count
end


function M.is_empty()
    return count == 0
end


M.reset()

return M
