
Last_sats = 0
Last_chassis_ef=0
Last_shooter_ef=0
Last_yunball_ef=0
Last_pos_ef=0




local function my_init()
    -- init 在模型加载时调用一次
end

local function my_background()
    -- background 周期性调用
end

local function my_run(event)
    lcd.clear()

    -- 获取电池遥测数据，添加 nil 防护
    local chassis_ef = getValue("RxBt") or 0
    local shooter_ef = getValue("Curr") or 0
    local yunball_ef = getValue("Capa") or 0
    local pos_ef = getValue("Bat%") or 0

    -- 姿态数据
    local ptch = getValue("Ptch") or 0
    local roll = getValue("Roll") or 0
    local yaw = getValue("Yaw") or 0

    -- 获取 GPS 遥测数据
    local gps = getValue("GPS")  -- 如果没有 GPS，可能返回 nil
    local altitude = getValue("Alt") or 0
    local speed = getValue("GSpd") or 0
    local sats = getValue("Sats") or 0
    local heading = getValue("Hdg") or 0

    -- 计算角度
    local position_heading = ptch * 57.296  -- 弧度转角度
    local mid360_heading = roll * 57.296
    local now_accle=yaw * 2

    local dis_2_target = heading + speed / 1000
    local debug_dis=altitude/1000

    -- GPS 坐标和角度字符串
    local position_x_str = (gps and gps.lat) and string.format("%.3f", gps.lat) or "N/A"
    local position_y_str = (gps and gps.lon) and string.format("%.3f", gps.lon) or "N/A"
    local position_heading_str = position_heading and string.format("%.2f", position_heading) or "N/A"
    local mid360_heading_str = mid360_heading and string.format("%.2f", mid360_heading) or "N/A"
    local now_accle_str = now_accle and string.format("%.2f", now_accle) or "N/A"


    local dis_2_target_str = dis_2_target and string.format("%.3f", dis_2_target) or "N/A"
    local debug_dis_str = debug_dis and string.format("%.3f", debug_dis) or "N/A"


   



    if sats == 0 then
        lcd.drawText(0, 0, "attack move", MIDSIZE)
    elseif sats == 1 then
        lcd.drawText(0, 0, "auto reload ball", MIDSIZE)
    elseif sats == 2 then
        lcd.drawText(0, 0, "all auto yunball", MIDSIZE)
    elseif sats == 3 then
        lcd.drawText(0, 0, "pos lock", MIDSIZE)
    elseif sats == 4 then
        lcd.drawText(0, 0, "lock on r2", MIDSIZE)
    elseif sats == 5 then
        lcd.drawText(0, 0, "shoot to center point", MIDSIZE)
    elseif sats == 6 then
        lcd.drawText(0, 0, "shoot to r2", MIDSIZE)
    elseif sats == 7 then
        lcd.drawText(0, 0, "yunball reload", MIDSIZE)
    elseif sats == 8 then
        lcd.drawText(0, 0, "wait mode move", MIDSIZE)
    elseif sats == 9 then
        lcd.drawText(0, 0, "reset all imu", MIDSIZE)
    elseif sats == 10 then
        lcd.drawText(0, 0, "reset sw motor", MIDSIZE)
    elseif sats == 11 then
        lcd.drawText(0, 0, "hand set lifter", MIDSIZE)
    elseif sats == 12 then
        lcd.drawText(0, 0, "hand set clawpos", MIDSIZE)
    elseif sats == 13 then
        lcd.drawText(0, 0, "reset shooter yunball", MIDSIZE)
    elseif sats == 15 then
        lcd.drawText(0, 0, "auto shoot center", MIDSIZE)
    elseif sats == 16 then
        lcd.drawText(0, 0, "auto shoot r2", MIDSIZE)
    elseif sats == 14 then
        lcd.drawText(0, 0, "set center point", MIDSIZE)

    elseif sats == 17 then
        lcd.drawText(0, 0, "cam lock", MIDSIZE)

    elseif sats == 30 then
        lcd.drawText(0, 0, "cam lock success", MIDSIZE)

    elseif sats == 31 then
        lcd.drawText(0, 0, "cam error", MIDSIZE)
    else



        lcd.drawText(10, 0, "GDUT Robocon2025", MIDSIZE)
    end
    


    if Last_sats ~= 1 and sats == 1 then
        playFile("/SCRIPTS/TELEMETRY/cn/loadball.wav")
    end
    if Last_sats ~= 3 and sats == 3 then
        playFile("/SCRIPTS/TELEMETRY/cn/lockcenter.wav")
    end

    if Last_sats ~= 4 and sats == 4 then
        playFile("/SCRIPTS/TELEMETRY/cn/lockr2.wav")
    end

  
    
    if Last_sats ~= 30 and sats == 30 then
        playFile("/SCRIPTS/TELEMETRY/cn/camlocked.wav")
    end


    if Last_sats ~= 10 and sats == 10 then
        playFile("/SCRIPTS/TELEMETRY/cn/resetsw.wav")
    end

    if Last_sats ~= 17 and sats == 17 then
        playFile("/SCRIPTS/TELEMETRY/cn/camlocking.wav")
    end



    if Last_sats ~= 31 and sats == 31 then
        playFile("/SCRIPTS/TELEMETRY/cn/camerror.wav")
    end



    if Last_chassis_ef ~= 0 and chassis_ef == 0 then

        playFile("/SCRIPTS/TELEMETRY/cn/chassisok.wav")

    end
    if Last_chassis_ef ~= 10 and chassis_ef == 10 then
        playFile("/SCRIPTS/TELEMETRY/cn/cu8off.wav")
    end

    if Last_chassis_ef ~= 20 and chassis_ef == 20 then
        playFile("/SCRIPTS/TELEMETRY/cn/c2006off.wav")
    end
    
     if Last_chassis_ef ~= 30 and chassis_ef == 30 then
       playFile("/SCRIPTS/TELEMETRY/cn/chassisdead.wav")
    end

    Last_chassis_ef = chassis_ef
    

    if Last_shooter_ef ~= 0 and shooter_ef == 0 then
        playFile("/SCRIPTS/TELEMETRY/cn/shooterok.wav")
    end
    
    if Last_shooter_ef ~= 10 and shooter_ef == 10 then
        playFile("/SCRIPTS/TELEMETRY/cn/shootermoff.wav")
       
    end

    Last_shooter_ef = shooter_ef


    if Last_yunball_ef ~= 0 and yunball_ef == 0 then
       playFile("/SCRIPTS/TELEMETRY/cn/yunok.wav")
    end
    if Last_yunball_ef ~= 1 and yunball_ef == 1 then
       playFile("/SCRIPTS/TELEMETRY/cn/yunmoff.wav")
    end

    Last_yunball_ef = yunball_ef


    if Last_pos_ef ~= 0 and pos_ef == 0 then
       playFile("/SCRIPTS/TELEMETRY/cn/posok.wav")
    end
    if Last_pos_ef ~= 1 and pos_ef == 1 then
        playFile("/SCRIPTS/TELEMETRY/cn/positionoff.wav")
    end

    if Last_pos_ef ~= 2 and pos_ef == 2 then
        playFile("/SCRIPTS/TELEMETRY/cn/ladaroff.wav")
    end

    if Last_pos_ef ~= 3 and pos_ef == 3 then
        playFile("/SCRIPTS/TELEMETRY/cn/posdead.wav")
    end
    

    Last_pos_ef = pos_ef




    







	Last_sats = sats


    -- 显示标题
    --lcd.drawText(10, 0, "RC25 pos datas", MIDSIZE)
    lcd.drawText(0, 15, "position: " .. position_x_str .. " " .. position_y_str .. " " .. position_heading_str, SMLSIZE)
    lcd.drawText(0, 25, "radar yaw " .. mid360_heading_str, SMLSIZE)
    lcd.drawText(0, 35, "dis to target " .. dis_2_target_str, SMLSIZE)
    lcd.drawText(0, 45, "debug dis " .. debug_dis_str, SMLSIZE)
    lcd.drawText(0, 55, "accele " .. now_accle_str, SMLSIZE)

    return 0
end

return { run = my_run, background = my_background, init = my_init }
