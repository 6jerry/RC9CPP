
local function my_init()
  -- init is called once when model is loaded
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
    local remaining = getValue("Bat%")  -- 剩余电量，单位：%

    -- 检查数据是否有效，若无效则显示0
    voltage = voltage or 0
    current = current or 0
    capacity = capacity or 0
    remaining = remaining or 0

    -- 显示标题
    lcd.drawText(10, 10, "电池状态", DBLSIZE)

    -- 显示电压
    lcd.drawText(10, 40, "电压: " .. string.format("%.2f V", voltage), MIDSIZE)

    -- 显示电流
    lcd.drawText(10, 70, "电流: " .. string.format("%.2f A", current), MIDSIZE)

    -- 显示容量
    lcd.drawText(10, 100, "容量: " .. string.format("%d mAh", capacity), MIDSIZE)

    -- 显示剩余电量
    lcd.drawText(10, 130, "剩余: " .. string.format("%d %%", remaining), MIDSIZE)

    return 0
end

return { run = my_run, background = my_background, init = my_init }
