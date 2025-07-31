#include "time_counter.h"
time_counter *error_manager::time_counter_ptr[18] = {nullptr};
void time_counter::init_time_counter()
{
    HAL_TIM_Base_Start(&htim2);
}

float time_counter::get_DeltaTime_ms()
{
    if (now_cnt_us == 0 & last_cnt_us == 0)
    {
        last_cnt_us = htim2.Instance->CNT;

        return 0.0f;
    }
    else
    {
        now_cnt_us = htim2.Instance->CNT;

        if (now_cnt_us > last_cnt_us) // 无溢出
        {
            delta_time_ms = (float)((float)now_cnt_us - (float)last_cnt_us) / 1000.0f;
        }
        else // 有溢出
        {
            delta_time_ms = (float)(now_cnt_us + (4294967295 - last_cnt_us)) / 1000.0f;
        }

        last_cnt_us = now_cnt_us;

        return delta_time_ms;
    }
}

void time_counter::config_param(uint8_t counter_id_, uint32_t max_time_out_ms_, uint32_t max_time_out_init)
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

uint8_t time_counter::get_ec_code()
{
    return error_code;
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

void error_manager::handle_r3_error()
{
    if ((time_counter_ptr[4]->get_ec_code() == 0) && (time_counter_ptr[5]->get_ec_code() == 0) && (time_counter_ptr[6]->get_ec_code() == 0))
    {
        u8_off_line = false;
    }
    else
    {
        u8_off_line = true;
    }

    if ((time_counter_ptr[10]->get_ec_code() == 0) && (time_counter_ptr[11]->get_ec_code() == 0) && (time_counter_ptr[12]->get_ec_code() == 0))
    {
        m3508_off_line = false;
    }
    else
    {
        m3508_off_line = true;
    }

    if (time_counter_ptr[0]->get_ec_code() == 0)
    {
        remote_off_line = false;
    }
    else
    {
        remote_off_line = true;
    }

    if ((time_counter_ptr[7]->get_ec_code() == 0) && (time_counter_ptr[8]->get_ec_code() == 0))
    {
        shooter_motor_offline = false;
    }
    else
    {
        shooter_motor_offline = true;
    }

    if (time_counter_ptr[1]->get_ec_code() == 0)
    {
        shooter_encoder_offline = false;
    }
    else
    {
        shooter_encoder_offline = true;
    }

    if (time_counter_ptr[9]->get_ec_code() == 0)
    {
        yunball_motor_offline = false;
    }
    else
    {
        yunball_motor_offline = true;
    }

    if (time_counter_ptr[3]->get_ec_code() == 0)
    {
        position_offline = false;
    }
    else
    {
        position_offline = true;
    }




}