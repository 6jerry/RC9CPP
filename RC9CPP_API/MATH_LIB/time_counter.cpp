#include "time_counter.h"

void time_counter::init_time_counter()
{
    HAL_TIM_Base_Start(&htim2);
}

float time_counter::get_DeltaTime_ms()
{
    if (now_cnt == 0 & last_cnt == 0)
    {
        last_cnt = htim2.Instance->CNT;

        return 0.0f;
    }
    else
    {
        now_cnt = htim2.Instance->CNT;

        if (now_cnt > last_cnt) // 无溢出
        {
            delta_time_ms = (float)((float)now_cnt - (float)last_cnt) / 1000.0f;
        }
        else // 有溢出
        {
            delta_time_ms = (float)(now_cnt + (4294967295 - last_cnt)) / 1000.0f;
        }

        last_cnt = now_cnt;

        return delta_time_ms;
    }
}