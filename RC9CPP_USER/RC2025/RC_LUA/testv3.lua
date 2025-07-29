
local function my_init()
    -- init is called once when model is loaded
 
end

local function my_background()
    -- background is called periodically
  



end

local function my_run(event)
    lcd.clear()

    -- 在屏幕上显示一行文本
    local voltage = getValue("RxBt") or 0
    local current = getValue("Curr") or 0
    local capacity = getValue("Capa") or 0
    local remaining = getValue("Bat%") or 0



    local vo_str = voltage and string.format("%.2f", voltage) or "N/A"
    local cur_str = current and string.format("%.2f", current) or "N/A"
    local cap_str = capacity and string.format("%.2f", capacity) or "N/A"
    local rem_str = remaining and string.format("%.2f", remaining) or "N/A"
    
    
    -- 显示标题
    lcd.drawText(0, 0, "bat", SMLSIZE)
    lcd.drawText(0, 15, "po: " .. vo_str .. " " .. cur_str .. " " .. cap_str, SMLSIZE)
    lcd.drawText(0, 25, "radar yaw " .. rem_str, SMLSIZE)
   
    playFile("/SCRIPTS/TELEMETRY/cn/alnch.wav")

    return 0
end

return { run = my_run, background = my_background, init = my_init }
