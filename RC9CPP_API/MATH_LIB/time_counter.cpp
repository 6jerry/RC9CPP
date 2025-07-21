#include "time_counter.h"
time_counter *error_manager::time_counter_ptr[18] = {nullptr};
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

time_counter::time_counter(uint8_t counter_id_, uint32_t max_time_out_ms_, uint32_t max_time_out_init)
{
    counter_id = counter_id_;
    time_out_ms = max_time_out_ms_;
    time_out_init = max_time_out_init;
    error_manager::time_counter_ptr[counter_id] = this;
}

void time_counter::check_tick()
{
    if (now_cnt == 0 & last_cnt == 0)
    {
        last_cnt = HAL_GetTick();
    }
    else
    {
        now_cnt = HAL_GetTick();

        delta_cnt = now_cnt - last_cnt;

        last_cnt = now_cnt;
    }
}

void time_counter::detect_error()
{
    if (now_cnt == 0) // 还没有初始化
    {
        if (!if_detected_not_init)
        {
            if_detected_not_init = true;
            detected_init_error = HAL_GetTick();
        }
        if (if_detected_not_init)
        {
            if ((HAL_GetTick() - detected_init_error) > (time_out_init))
            {
                error_code = 1;
            }
        }
    }
    else if (now_cnt != 0)
    {

        if ((HAL_GetTick() - now_cnt) > (time_out_ms))
        {
            error_code = 1;
        }
        else
        {
            error_code = 0;
            if_detected_not_init = false;
            detected_init_error = 0;
        }
    }
}

void error_manager::process_data()
{
    for (uint8_t i = 0; i < 18; i++)
    {
        if (time_counter_ptr[i] != nullptr)
        {
            time_counter_ptr[i]->detect_error();
        }
    }
}
