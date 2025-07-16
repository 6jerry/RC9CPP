
local function my_init()
    -- init is called once when model is loaded
 
end

local function my_background()
    -- background is called periodically
  



end

local function my_run(event)
    lcd.clear()

    -- 在屏幕上显示一行文本
    

    
    -- 显示标题
    lcd.drawText(0, 0, "bat", SMLSIZE)

   
    playFile("/SCRIPTS/TELEMETRY/cn/alnch.wav")

    return 0
end

return { run = my_run, background = my_background, init = my_init }
