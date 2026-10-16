--[[
   Automatic payload release on loss of control.

   Releases the payload when the vehicle is both tilted past PREL_ANGLE and at
   least one motor is commanded at or above PREL_PWM, continuously for PREL_TIME
   seconds, while armed and flying. Together these mean the attitude controller
   has run out of authority: the vehicle is leaning far past its normal limit
   with a motor already saturated.

   The release latches until the vehicle is disarmed. With PREL_ACTION 0 (the
   default) the script only warns and logs, so the trigger can be checked in
   flight before it is allowed to release anything.
--]]

local SCRIPT_NAME = "PREL"
local RUN_INTERVAL_MS = 20          -- 50 Hz
local LOG_INTERVAL_MS = 100         -- 10 Hz PREL log while armed
local SERVO_HOLD_MS = 1000          -- servo override re-asserted every loop while released
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local MOTOR_FUNCTION_FIRST = 33     -- k_motor1
local MOTOR_FUNCTION_LAST = 40      -- k_motor8

local PARAM_TABLE_KEY = 111
local PARAM_TABLE_PREFIX = "PREL_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 9), "PREL: could not add param table")

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("PREL: could not add param %s", name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: PREL_ENABLE
  // @DisplayName: Payload release enable
  // @Description: Enables automatic payload release on loss of control
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PREL_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  // @Param: PREL_ANGLE
  // @DisplayName: Payload release tilt angle
  // @Description: Total tilt from level (combined roll and pitch) at or above which the release can trigger
  // @Units: deg
  // @Range: 10 90
  // @User: Standard
--]]
local PREL_ANGLE = bind_add_param("ANGLE", 2, 45)

--[[
  // @Param: PREL_PWM
  // @DisplayName: Payload release motor PWM
  // @Description: Motor output PWM at or above which the release can trigger. Any one motor reaching it is enough.
  // @Units: PWM
  // @Range: 1000 2200
  // @User: Standard
--]]
local PREL_PWM = bind_add_param("PWM", 3, 1950)

--[[
  // @Param: PREL_TIME
  // @DisplayName: Payload release trigger time
  // @Description: Time both conditions must hold continuously before the payload is released
  // @Units: s
  // @Range: 0 5
  // @User: Standard
--]]
local PREL_TIME = bind_add_param("TIME", 4, 0.5)

--[[
  // @Param: PREL_ACTION
  // @DisplayName: Payload release action
  // @Description: What the script does when it triggers. 0 only warns and logs, for checking the trigger in flight.
  // @Values: 0:Warn and log only,1:Relay,2:Servo PWM
  // @User: Standard
--]]
local PREL_ACTION = bind_add_param("ACTION", 5, 0)

--[[
  // @Param: PREL_RELAY
  // @DisplayName: Payload release relay
  // @Description: Relay number (1 is RELAY1) switched when PREL_ACTION is 1
  // @Range: 1 16
  // @User: Standard
--]]
local PREL_RELAY = bind_add_param("RELAY", 6, 1)

--[[
  // @Param: PREL_RLY_ON
  // @DisplayName: Payload release relay state
  // @Description: Relay state that releases the payload when PREL_ACTION is 1
  // @Values: 0:Off releases,1:On releases
  // @User: Standard
--]]
local PREL_RLY_ON = bind_add_param("RLY_ON", 7, 1)

--[[
  // @Param: PREL_SRV_CH
  // @DisplayName: Payload release servo channel
  // @Description: Servo output channel (1 is SERVO1) driven to PREL_SRV_PWM when PREL_ACTION is 2. 0 disables.
  // @Range: 0 32
  // @User: Standard
--]]
local PREL_SRV_CH = bind_add_param("SRV_CH", 8, 0)

--[[
  // @Param: PREL_SRV_PWM
  // @DisplayName: Payload release servo PWM
  // @Description: PWM sent on PREL_SRV_CH to release the payload when PREL_ACTION is 2
  // @Units: PWM
  // @Range: 800 2200
  // @User: Standard
--]]
local PREL_SRV_PWM = bind_add_param("SRV_PWM", 9, 1900)

local trigger_start_ms = nil     -- when both conditions first held, nil when they do not
local released = false
local warned_pending = false
local last_log_ms = 0

-- tilt from level in degrees, combining roll and pitch
local function tilt_deg()
    local roll = ahrs:get_roll_rad()
    local pitch = ahrs:get_pitch_rad()
    if not roll or not pitch then
        return nil
    end
    local c = math.cos(roll) * math.cos(pitch)
    c = math.max(-1, math.min(1, c))
    return math.deg(math.acos(c))
end

-- highest PWM across the motor outputs, and which motor
local function max_motor_pwm()
    local max_pwm, max_motor = 0, 0
    for fn = MOTOR_FUNCTION_FIRST, MOTOR_FUNCTION_LAST do
        local pwm = SRV_Channels:get_output_pwm(fn)
        if pwm and pwm > max_pwm then
            max_pwm, max_motor = pwm, fn - MOTOR_FUNCTION_FIRST + 1
        end
    end
    return max_pwm, max_motor
end

local function do_release()
    local action = PREL_ACTION:get()
    if action == 1 then
        local inst = math.floor(PREL_RELAY:get()) - 1
        if PREL_RLY_ON:get() >= 1 then
            relay:on(inst)
        else
            relay:off(inst)
        end
    elseif action == 2 then
        local ch = math.floor(PREL_SRV_CH:get())
        if ch >= 1 then
            SRV_Channels:set_output_pwm_chan_timeout(ch - 1, math.floor(PREL_SRV_PWM:get()), SERVO_HOLD_MS)
        end
    end
end

local function check_config()
    local action = PREL_ACTION:get()
    if action == 1 and not relay:enabled(math.floor(PREL_RELAY:get()) - 1) then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("PREL: RELAY%d is not enabled", math.floor(PREL_RELAY:get())))
    elseif action == 2 and PREL_SRV_CH:get() < 1 then
        gcs:send_text(MAV_SEVERITY.WARNING, "PREL: PREL_SRV_CH not set")
    end
end

local function update()
    local now = millis():toint()

    if PREL_ENABLE:get() < 1 then
        trigger_start_ms, warned_pending = nil, false
        return update, 500
    end

    local armed = arming:is_armed()
    if not armed then
        -- disarming clears the latch so the next flight starts armed for release again
        if released then
            gcs:send_text(MAV_SEVERITY.INFO, "PREL: reset on disarm")
        end
        released, trigger_start_ms, warned_pending = false, nil, false
        return update, RUN_INTERVAL_MS
    end

    local tilt = tilt_deg() or 0
    local pwm, motor = max_motor_pwm()
    local flying = vehicle:get_likely_flying()
    local cond = flying and tilt >= PREL_ANGLE:get() and pwm >= PREL_PWM:get()

    if released then
        -- keep a servo release asserted; a relay holds its state on its own
        if PREL_ACTION:get() == 2 then
            do_release()
        end
    elseif cond then
        if not trigger_start_ms then
            trigger_start_ms = now
        end
        if not warned_pending then
            gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("PREL: tilt %.0f deg, M%d %d PWM", tilt, motor, pwm))
            warned_pending = true
        end
        if now - trigger_start_ms >= PREL_TIME:get() * 1000 then
            released = true
            do_release()
            if PREL_ACTION:get() == 0 then
                gcs:send_text(MAV_SEVERITY.CRITICAL, "PREL: release triggered (warn only, PREL_ACTION 0)")
            else
                gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("PREL: PAYLOAD RELEASED tilt %.0f deg M%d %d PWM", tilt, motor, pwm))
            end
        end
    else
        trigger_start_ms, warned_pending = nil, false
    end

    if now - last_log_ms >= LOG_INTERVAL_MS or cond then
        last_log_ms = now
        logger:write("PREL", "Tilt,PWM,Mot,Fly,Cond,Rel", "fHBBBB",
                     tilt, pwm, motor, flying and 1 or 0, cond and 1 or 0, released and 1 or 0)
    end

    return update, RUN_INTERVAL_MS
end

-- report a runtime error and keep running, rather than letting the release check stop silently
local function protected_wrapper()
    local success, err_or_fn, interval_ms = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "PREL: internal error: " .. tostring(err_or_fn))
        return protected_wrapper, 1000
    end
    return protected_wrapper, interval_ms or RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("PREL: loaded, %s", PREL_ENABLE:get() >= 1 and "enabled" or "disabled"))
if PREL_ENABLE:get() >= 1 then
    check_config()
end

return protected_wrapper, 1000
