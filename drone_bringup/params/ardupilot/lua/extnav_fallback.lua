-- extnav_fallback.lua - runs inside ArduPilot (SCR_ENABLE 1). Test B6.
-- When our external position (VISION_POSITION_ESTIMATE from the bridge) stops, switch EKF3 to source set 2:
-- no position, optical-flow velocity, rangefinder height. When it is back and steady for 1 s, switch back
-- to source set 1 (our position). Runs on the flight controller, so it works when the companion computer
-- freezes or dies. Only acts while armed. Messages appear as STATUSTEXT ("ExtNav lost ..." / "ExtNav back ...").

local SET_EXTNAV = 0          -- EK3_SRC1_*
local SET_FLOW   = 1          -- EK3_SRC2_*
local BACK_MS    = 1000       -- ExtNav must be healthy this long before switching back
local current    = SET_EXTNAV
local healthy_since = nil

function update()
  local now = millis()
  if not arming:is_armed() then
    if current ~= SET_EXTNAV then
      ahrs:set_posvelyaw_source_set(SET_EXTNAV)
      current = SET_EXTNAV
    end
    healthy_since = nil
    return update, 100
  end
  local ok = visual_odom:healthy()          -- false ~0.3 s after the last pose
  if not ok then
    healthy_since = nil
    if current ~= SET_FLOW then
      ahrs:set_posvelyaw_source_set(SET_FLOW)
      current = SET_FLOW
      gcs:send_text(4, "ExtNav lost: flow fallback (source set 2)")
    end
  else
    if healthy_since == nil then healthy_since = now end
    if current ~= SET_EXTNAV and (now - healthy_since) >= BACK_MS then
      ahrs:set_posvelyaw_source_set(SET_EXTNAV)
      current = SET_EXTNAV
      gcs:send_text(6, "ExtNav back: source set 1")
    end
  end
  return update, 100
end

gcs:send_text(6, "extnav_fallback.lua running")
return update()
