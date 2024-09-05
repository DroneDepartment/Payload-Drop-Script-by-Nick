--Constants
local SERVO_CHANNEL = 94 --Servo Output
local RC_CHANNEL = 8 --RC input
local SERVO_PWM_DROP = 1900 --PWM Trigger Value
local SERVO_PWM_NEUTRAL = 1500 --Neutral PWM Value
local RC_PWM_SRTL = 1900
local DROP_RADIUS = 10 --Meters
local PAYLOAD_MASS = 1.5 --kg
local DRAG_COEFFICIENT = 10 --kg/s

--Variables
local target = nil
local target_location = Location()
local target_wp_index = 0
local ground_speed = 0 --m/s
local wind_speed = 0 --m/s
local wind_velocity = 0 --m/s (vector)
local altitude = 0 --meters
local drop_distance = 0 --meters
local current_pos = nil
local payload_dropped = false
local counter = 0
local count_on = false --Bool whether or not to send distance data
local sensor_val = 0
local ticker = 0
local tocker = 0
local prev = nil
local rtl_waypoints = {}
local second_drop = false


-- -- Function to log the current waypoint in reverse order
-- function log_waypoint()
--     local mission_item_index = mission:get_current_nav_index()
--     local mission_item = mission:get_item(mission_item_index)
--     if mission_item ~= prev then
--         if prev then
--             local wp = location()
--             wp:lat(prev:x())
--             wp:lng(prev:y())
--             wp:alt(prev:z())

--             table.insert(rtl_waypoints, 1, wp) -- Insert at the beginning to store in reverse order

--             prev = mission_item
--         end
--     end
-- end

--Executes SmartRTL logic
function execute_smart_rtl()
    gcs:send_text(0, "Initiating SmartRTL")
    local num_cmds = mission:num_commands()
    local second_half = math.ceil(mission:num_commands() / 2)
    local current_pos = ahrs:get_position()

    local min_dist = 6000
    local closest = 0
    for i = second_half + 1, num_cmds, 1 do
        local current_item = mission:get_item(i)
        local current_location = Location()

        current_location:lat(current_item:x())
        current_location:lng(current_item:y())
        current_location:alt(current_item:z())

        local distance = current_pos:get_distance(current_location)

        if distance < min_dist then
            min_dist = distance
            closest = i
        end
    end

    gcs:send_text(0, string.format("Jumping to waypoint: %d", closest))
    mission:set_current_cmd(closest)

end

--Prints the staus of the payload sensor
function spit()
    if port:available() > 0 then
        sensor_val = port:read()
        gcs:send_text(0, string.format("sensor_val: %d", sensor_val))

        while port:available() > 0 do
            port:read()  -- Discard the rest of the data
        end
    end   
end

function drop_check()
    if port:available() > 0 then
        sensor_val = port:read()

        while port:available() > 0 do
            port:read()  -- Discard the rest of the data
        end
    else
        if tocker > 2000 then --Arduino took too long to respond
            if not second_drop then
                gcs:send_text(0, "No feedback received from arduino, attempting drop again")
                mission:set_current_cmd(target_wp_index)
                tocker = 0
                second_drop = true
                return update, 100
            else
                gcs:send_text(0, "No feedback from arduino on second attempt, terminating script")
                payload_dropped = true
                return update, 100
            end
        else
            tocker = tocker + 100
            return drop_check, 100
        end
    end 

    --Doors opened
    if sensor_val == 1 then
        payload_dropped = true
        gcs:send_text(0, "Payload sequence completed")
        gcs:send_text(0, "Terminating script")
        return update, 100
    --5 seconds without sequence completing
    elseif ticker > 5000 then
        gcs:send_text(0, "Payload sequence failed")
        if not second_drop then
            gcs:send_text(0, "Attemptng drop again")
            mission:set_current_cmd(target_wp_index)
            ticker = 0
            tocker = 0
            second_drop = true
            return update, 100
        else
            gcs:send_text(0, "Second attempt failed, terminating script")
            payload_dropped = true
            return update, 100
        end
    else
        ticker = ticker + 100
        return drop_check, 100
    end
end

--Monitors for DO_SEND_SCRIPT_MESSAGE command and initiates paylod functionality
function handle_mission_item()

    --Logs current waypoint
    --log_waypoint()

    if rc:get_pwm(RC_CHANNEL) == RC_PWM_SRTL then
        execute_smart_rtl()
    end

    --Checks for incoming mission items, param1 is the command ID
    local new_var, param1 = mission_receive()

    --ID parameter of DO_SEND_SCRIPT_MESSAGE should be manually set to 176 in MissionPlanner
    if param1 == 176 then 
        gcs:send_text(0, "Received drop command, initiating drop functionality")
        spit()
        target_wp_index = mission:get_current_nav_index()
        gcs:send_text(0, string.format("Waypoint index: %.2f", target_wp_index))
        fetch_target_wp()
        count_on = true
        return update, 100
    else
        return handle_mission_item, 100
    end
end

--Returns Target Waypoint
local function fetch_target_wp()
    gcs:send_text(0, "Fetching target waypoint")
    target = mission:get_item(target_wp_index)
    if target == nil then
        gcs:send_text(0, "Target waypoint not found")
    else
        target_location:lat(target:x())
        target_location:lng(target:y())
        target_location:alt(target:z())
    end
end

--Calculates distance from drop point from which to drop
local function fetch_drop_distance() --Assumes wind only in direction of motion
    return DROP_RADIUS
end

--Main Update Function
function update()
    if not payload_dropped then
        if target == nil then
            fetch_target_wp()
            if target == nil then
                return update, 100
            end
        end

        --Fetch current position
        current_pos = ahrs:get_position()
        if current_pos == nil then
            return update, 100
        end

        --Fetch drop distance based on current conditions
        drop_distance = fetch_drop_distance()

        if drop_distance == nil then
            drop_distance = fetch_drop_distance()
            if drop_distance == nil then
                return update, 100
            end
        end

        --Determine action of servo based on current state and distance from target
        local current_distance = current_pos:get_distance(target_location)
        if counter % 50 == 0 and count_on and not payload_dropped then
            gcs:send_text(0, string.format("Current distance to target: %.2f m", current_distance))
        end

        --Initiate drop
        if current_distance <= drop_distance then
            SRV_Channels:set_output_pwm(SERVO_CHANNEL, SERVO_PWM_DROP)
            if not second_drop then
                gcs:send_text(0, "Payload drop triggered")
                --Check for arduino feedback
                drop_check()
            else
                gcs:send_text(0, "2nd payload drop triggered")
                --Check for arduino feedback
                drop_check()
            end
        end

        --Counter to slow rate of printing to GCS
        counter = counter + 1

        return update, 100
    end
end

return handle_mission_item(), 100