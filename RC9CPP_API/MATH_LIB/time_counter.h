#ifndef TIME_COUNTER_H
#define TIME_COUNTER_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"
#include "TaskManager.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum error_check_id
{
    ELRS_receiver,
    Motor_front_u8,
    Motor_front_2006,

};

class time_counter
{
public:
    float get_DeltaTime_ms();

    void check_tick();

    static void init_time_counter();
    float delta_time_ms = 0.0f;
    uint32_t now_cnt = 0, last_cnt = 0, detected_init_error = 0, delta_cnt = 0;

    uint32_t now_cnt_us = 0, last_cnt_us = 0;

    uint32_t time_out_ms = 0, time_out_init = 0;

    bool if_detected_not_init = false;
    uint8_t error_code = 0; // 0:正常 1:断线 2:数据异常

    uint8_t counter_id = 0;

    uint8_t get_ec_code();

    void detect_error();
    // time_counter(uint8_t counter_id_ = 0, uint32_t max_time_out_ms_ = 500, uint32_t max_time_out_init = 2000);

    void config_param(uint8_t counter_id_ = 0, uint32_t max_time_out_ms_ = 500, uint32_t max_time_out_init = 2000);

private:
};

class error_manager : public ITaskProcessor
{
public:
    void process_data();
    static time_counter *time_counter_ptr[18];

    void handle_r3_error();

    uint8_t chassis_ec_code = 0, shooter_ec_code = 0, yunball_ec_code = 0, locate_ec_code = 0;

    bool u8_off_line = false, m3508_off_line = false, remote_off_line = false;

    bool shooter_motor_offline = false, shooter_encoder_offline = false;

    bool yunball_motor_offline = false, yunall_gaslow = false;

    bool position_offline = false;

    // CrsfReceiver *crsf_check = nullptr;
};

#endif

#endif