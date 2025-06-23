
local function my_init()
  -- init is called once when model is loaded
end

local function my_background()
  -- background is called periodically
end

local function my_run(event)
    lcd.clear()

    -- 在屏幕上显示一行文本
    lcd.drawText(10, 10, "Hello, EdgeTX!", MIDSIZE)

    -- 返回 0，表示脚本成功运行
    return 0
end

return { run = my_run, background = my_background, init = my_init }
