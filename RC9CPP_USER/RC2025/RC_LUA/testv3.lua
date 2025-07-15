
local function my_init()
    -- init is called once when model is loaded
   playFile("/SCRIPTS/TELEMETRY/cn/ladar_offline.wav")
end

local function my_background()
  -- background is called periodically
end

local function my_run(event)
    lcd.clear()

    -- 在屏幕上显示一行文本
    

    -- 获取电池遥测数据
    local voltage = getValue("RxBt")  -- 电压，单位：V
    local current = getValue("Curr")    -- 电流，单位：A
    local capacity = getValue("Capa")   -- 容量，单位：mAh
    local remaining = getValue("Bat%") -- 剩余电量，单位：%
    

    -- 姿态数据


    local ptch = getValue("Ptch")
    local roll = getValue("Roll")
    local yaw = getValue("Yaw")



     -- 获取 GPS 遥测数据
    local gps = getValue("GPS")    -- GPS 坐标（表，包含 lat 和 lon）
    local altitude = getValue("Alt")  -- 海拔，单位：米
    local speed = getValue("GSpd")     -- 地面速度，单位：km/h
    local sats = getValue("Sats")    -- 卫星数量
    local heading = getValue("Hdg")
    

    local packed_data = heading + speed / 1000
    
    local packed_data_str= packed_data and string.format("%.3f",packed_data) or "N/A" 


-- 格式化数据，如果数据不可用则显示 "N/A"
    local lat_str = (gps and gps.lat) and string.format("%.3f", gps.lat) or "N/A"  -- 三位小数 -200~200
    local lon_str = (gps and gps.lon) and string.format("%.3f", gps.lon) or "N/A"  -- 三位小数 -200~200
    local alt_str = altitude and string.format("%.3f", altitude) or "N/A"        -- 整数，毫米数 ,0~30000
    local speed_str = speed and string.format("%.3f", speed) or "N/A"         -- 整数 0~0.1*30000
    local sats_str = sats and tostring(sats) or "N/A"                             -- 整数 0~100
    local heading_str=heading and string.format("%.3f", heading ) or "N/A"     --整数 0~0.01*30000

 

    local voltage_str = voltage and string.format("%.3f", voltage) or "N/A" --0~10*300
    local current_str = current and string.format("%.3f", current) or "N/A" --0~10*300
    local capacity_str = capacity and string.format("%.3f", capacity) or "N/A" --0~300
    local remaining_str = remaining and string.format("%.3f", remaining) or "N/A" --0~100

    local ptch_str = ptch and string.format("%.3f", ptch) or "N/A" -- -3.14~3.14
    local roll_str = roll and string.format("%.3f", roll) or "N/A"
    local yaw_str = yaw and string.format("%.3f", yaw) or "N/A"



    -- 显示标题
    lcd.drawText(0, 0, "bat", SMLSIZE)

    -- 显示电压
    lcd.drawText(0, 10, "position: " .. packed_data_str .. " " .. roll_str .. " " ..  yaw_str ,SMLSIZE)

    -- 显示电流
    --lcd.drawText(10, 70, "电流: " .. string.format("%.2f A", current), SMLSIZE)

    -- 显示容量
    --lcd.drawText(10, 100, "容量: " .. string.format("%d mAh", capacity), SMLSIZE)

    -- 显示剩余电量
    --lcd.drawText(10, 130, "剩余: " .. string.format("%d %%", remaining), SMLSIZE)

    playFile("/SCRIPTS/TELEMETRY/cn/ladar_offline.wav")

    return 0
end

return { run = my_run, background = my_background, init = my_init }
