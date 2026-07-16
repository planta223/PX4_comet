
local THRESHOLD_LAUNCH_ACCEL = 25.0
local THRESHOLD_TILT_DEG     = 70.0
local THRESHOLD_FREEFALL     = 5.0
local MAX_FLIGHT_TIME_MS     = 15000
local SENSOR_TIMEOUT_MS      = 1500
local DESCENT_CONFIDENCE     = 5
local ALT_DROP_MARGIN        = 2.0
local LAND_ALT               = 3.0
local APOGEE_HOLD_MS         = 1000
local ERROR_MAX_WAIT_MS      = 2000
local RESET_RC_CHANNEL       = 7
local RESET_RC_HIGH_PWM      = 1700

local REMOTE_ARM_CHANNEL  = 8
local REMOTE_FIRE_CHANNEL = 9
local REMOTE_HIGH_PWM     = 1700
local REMOTE_ARM_WINDOW_MS = 5000

local SERVO_CLOSED_PWM = 1000
local SERVO_OPEN_PWM   = 2000
local SERVO_SCRIPT_CHAN = 1

local LOOP_MS        = 100
local LORA_FAST_MS = 250
local LORA_FLIGHT_MS = 1000
local LORA_LAND_MS   = 1000
local EVENT_BURST    = 2
local BURST_GAP_MS   = 100
local LORA_BAUD      = 57600

local STATE = { READY=0, ASCENT=1, APOGEE=2, DESCENT=3, LANDED=4, ERROR=5 }
local currentState = STATE.READY

local maxAltitude        = 0.0
local descentCount       = 0
local launchTimeMs       = 0
local deployTimeMs       = 0
local deployStep         = 0
local apogeeFired        = false
local apogeeEnterMs      = 0
local errorEnterMs       = 0
local lastSensorMs       = 0
local lastLoraMs         = 0
local loraSeq            = 0
local prevState          = STATE.READY
local eventBurstLeft     = 0
local remoteArmedMs      = 0
local loraTxFailures     = 0

local curAlt   = 0.0
local totalAcc = 0.0
local accX, accY, accZ = 0.0, 0.0, 0.0

local lora = serial:find_serial(0)
if lora then
    lora:begin(LORA_BAUD)
    lora:set_flow_control(0)
    gcs:send_text(6, "LoRa UART found: " .. LORA_BAUD .. " baud, 8N1")
else
    gcs:send_text(0, "LoRa UART NOT FOUND: set SERIALx_PROTOCOL=28")
end

local function servo_open()
    SRV_Channels:set_output_pwm_chan(SERVO_SCRIPT_CHAN - 1, SERVO_OPEN_PWM)
end
local function servo_close()
    SRV_Channels:set_output_pwm_chan(SERVO_SCRIPT_CHAN - 1, SERVO_CLOSED_PWM)
end

local function u16(v) return v & 0xFFFF end
local function u32(v) return v & 0xFFFFFFFF end

local function push16(t, v) v = u16(v); t[#t+1] = v & 0xFF; t[#t+1] = (v>>8) & 0xFF end
local function push32(t, v) v = u32(v)
    t[#t+1] = v & 0xFF; t[#t+1] = (v>>8) & 0xFF
    t[#t+1] = (v>>16) & 0xFF; t[#t+1] = (v>>24) & 0xFF
end

local function send_bytes(bytes)
    if not lora then return false end
    local crc = 0
    for i = 2, #bytes do crc = crc ~ bytes[i] end
    bytes[#bytes+1] = crc
    for i = 1, #bytes do
        if lora:write(bytes[i]) ~= 1 then
            loraTxFailures = loraTxFailures + 1
            if loraTxFailures == 1 or (loraTxFailures % 10) == 0 then
                gcs:send_text(4, "LoRa UART TX buffer full; failures=" .. loraTxFailures)
            end
            return false
        end
    end
    loraSeq = (loraSeq + 1) % 256
    return true
end

local function send_accel(state, ax, ay, az, alt)
    local b = {}
    b[1] = 0xA5
    b[2] = (0 << 4) | (state & 0x0F)
    b[3] = loraSeq & 0xFF
    push16(b, math.floor(ax * 50))
    push16(b, math.floor(ay * 50))
    push16(b, math.floor(az * 50))
    push16(b, math.floor(alt))
    send_bytes(b)
end

local function send_gps(state, lat, lng, alt)
    local b = {}
    b[1] = 0xA5
    b[2] = (1 << 4) | (state & 0x0F)
    b[3] = loraSeq & 0xFF
    push32(b, math.floor(lat * 1e7))
    push32(b, math.floor(lng * 1e7))
    push16(b, math.floor(alt))
    send_bytes(b)
end

local function get_latlng()
    local loc = ahrs:get_position()
    if loc then
        return loc:lat() / 1e7, loc:lng() / 1e7
    end
    return 0.0, 0.0
end

local function reset_for_next_flight()
    currentState    = STATE.READY
    maxAltitude     = 0.0
    descentCount    = 0
    launchTimeMs    = 0
    deployTimeMs    = 0
    deployStep      = 0
    apogeeFired     = false
    apogeeEnterMs   = 0
    errorEnterMs    = 0
    prevState       = STATE.READY
    eventBurstLeft  = 0
    servo_close()
    gcs:send_text(6, "ROK2 RESET: Ready for next flight.")
end

function update()
    local now = millis():toint()

    local baro_alt = baro:get_altitude()
    local accel = ahrs:get_accel()
    if baro_alt and accel then
        curAlt   = baro_alt
        totalAcc = accel:length()
        accX, accY, accZ = accel:x(), accel:y(), accel:z()
        lastSensorMs = now
    end

    local pitch = math.abs(math.deg(ahrs:get_pitch()))
    local roll  = math.abs(math.deg(ahrs:get_roll()))

    if currentState == STATE.ASCENT or currentState == STATE.APOGEE then
        if (now - lastSensorMs) > SENSOR_TIMEOUT_MS then
            if apogeeFired then
                gcs:send_text(4, "Sensor lost after deploy. Proceeding to DESCENT.")
                currentState = STATE.DESCENT
            else
                gcs:send_text(0, "CRITICAL: Sensor Timeout! Emergency Deploy!")
                currentState = STATE.ERROR
                errorEnterMs = now
            end
        end
    end

    if currentState == STATE.READY then
        if totalAcc > THRESHOLD_LAUNCH_ACCEL then
            currentState = STATE.ASCENT
            launchTimeMs = now
            gcs:send_text(6, "LIFTOFF DETECTED!")
        end

    elseif currentState == STATE.ASCENT then
        if curAlt > maxAltitude then
            maxAltitude = curAlt
            descentCount = 0
        elseif curAlt < (maxAltitude - ALT_DROP_MARGIN) then
            descentCount = descentCount + 1
        end
        local tipped = (pitch > THRESHOLD_TILT_DEG) or (roll > THRESHOLD_TILT_DEG)
        if descentCount >= DESCENT_CONFIDENCE or tipped then
            currentState = STATE.APOGEE
            gcs:send_text(6, "APOGEE DETECTED! Preparing ejection.")
        end
        if (now - launchTimeMs) > MAX_FLIGHT_TIME_MS then
            currentState = STATE.ERROR
            errorEnterMs = now
            gcs:send_text(0, "SAFETY TIMER EXCEEDED!")
        end

    elseif currentState == STATE.APOGEE then
        if not apogeeFired then
            servo_open()
            gcs:send_text(6, "PARACHUTE DEPLOYED (1st Attempt)!")
            deployTimeMs   = now
            deployStep     = 1
            apogeeFired    = true
            apogeeEnterMs  = now
        else
            local descending = curAlt < (maxAltitude - ALT_DROP_MARGIN)
            local held       = (now - apogeeEnterMs) > APOGEE_HOLD_MS
            if descending or held then
                currentState = STATE.DESCENT
                maxAltitude  = curAlt
            end
        end

    elseif currentState == STATE.DESCENT then
        if deployStep == 1 and (now - deployTimeMs) > 1500 then
            if totalAcc < THRESHOLD_FREEFALL then
                gcs:send_text(4, "WARNING: Parachute failed!")
                servo_close()
                deployTimeMs = now
                deployStep = 2
            else
                gcs:send_text(6, "Parachute opened normally.")
                deployStep = 3
            end
        elseif deployStep == 2 and (now - deployTimeMs) > 300 then
            servo_open()
            gcs:send_text(6, "PARACHUTE DEPLOYED (2nd Attempt)!")
            deployStep = 3
        end
        if curAlt < LAND_ALT and totalAcc < 12.0 and totalAcc > 8.0 then
            currentState = STATE.LANDED
            gcs:send_text(6, "LANDING CONFIRMED. Flight ended.")
        end

    elseif currentState == STATE.ERROR then
        local freefall_ok = totalAcc < THRESHOLD_FREEFALL
        local waited_too_long = (now - errorEnterMs) > ERROR_MAX_WAIT_MS

        if freefall_ok or waited_too_long then
            servo_open()
            if freefall_ok then
                gcs:send_text(0, "ERROR: Freefall confirmed. Emergency deploy executed.")
            else
                gcs:send_text(0, "ERROR: Max wait exceeded. Forced deploy (freefall unconfirmed).")
            end
            apogeeFired = true
            deployTimeMs = now
            deployStep = 3
            currentState = STATE.DESCENT
        else
            gcs:send_text(3, "ERROR state: waiting for freefall before deploy.")
        end

    elseif currentState == STATE.LANDED then
        local reset_sw = rc:get_pwm(RESET_RC_CHANNEL)
        if reset_sw and reset_sw > RESET_RC_HIGH_PWM then
            reset_for_next_flight()
        end
    end

    local can_remote_deploy = (currentState == STATE.APOGEE)
                            or (currentState == STATE.DESCENT)

    local arm_sw  = rc:get_pwm(REMOTE_ARM_CHANNEL)
    local fire_sw = rc:get_pwm(REMOTE_FIRE_CHANNEL)

    if currentState == STATE.ASCENT then
        if (arm_sw and arm_sw > REMOTE_HIGH_PWM)
        or (fire_sw and fire_sw > REMOTE_HIGH_PWM) then
            gcs:send_text(0, "DENIED: remote deploy ignored during ASCENT")
        end
        remoteArmedMs = 0
    elseif can_remote_deploy then
        if arm_sw and arm_sw > REMOTE_HIGH_PWM then
            if remoteArmedMs == 0 then
                remoteArmedMs = now
                gcs:send_text(2, "Remote deploy ARMED. Send FIRE within 5s.")
            end
        else
            remoteArmedMs = 0
        end

        if remoteArmedMs > 0 and (now - remoteArmedMs) > REMOTE_ARM_WINDOW_MS then
            remoteArmedMs = 0
            gcs:send_text(3, "Remote ARM window expired.")
        end

        if remoteArmedMs > 0 and fire_sw and fire_sw > REMOTE_HIGH_PWM then
            servo_open()
            apogeeFired = true
            deployStep = 3
            deployTimeMs = now
            remoteArmedMs = 0
            gcs:send_text(0, "Remote FIRE executed.")
        end
    end

    if currentState ~= prevState then
        eventBurstLeft = EVENT_BURST
        prevState = currentState
        gcs:send_text(6, "LoRA event burst: state -> " .. currentState)
    end

    local lora_interval
    if currentState == STATE.ASCENT or currentState == STATE.APOGEE
    or (currentState == STATE.DESCENT and deployStep < 3) then
        lora_interval = LORA_FAST_MS
    elseif currentState == STATE.LANDED then
        lora_interval = LORA_LAND_MS
    else
        lora_interval = LORA_FLIGHT_MS
    end
    local periodic = (now - lastLoraMs) >= lora_interval
    local burst    = (eventBurstLeft > 0)
    local burst_ok = burst and ((now - lastLoraMs) >= BURST_GAP_MS)

    if periodic or burst_ok then
        local send_accel_now = (currentState == STATE.ASCENT)
                             or (currentState == STATE.APOGEE)
                             or (currentState == STATE.DESCENT and deployStep < 3)

        if send_accel_now then
            send_accel(currentState, accX, accY, accZ, curAlt)
        else
            local lat, lng = get_latlng()
            send_gps(currentState, lat, lng, curAlt)
        end
        lastLoraMs = now
        if burst_ok then
            eventBurstLeft = eventBurstLeft - 1
        end
    end

    return update, LOOP_MS
end

servo_close()
gcs:send_text(6, "ROK2 FLIGHT COMPUTER (Lua) READY!")
return update, LOOP_MS
