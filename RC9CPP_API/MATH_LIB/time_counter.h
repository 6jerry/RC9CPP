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

    static void init_time_counter();
    float delta_time_ms = 0.0f, time_out_ms = 0.0f;
    uint32_t now_cnt = 0, last_cnt = 0, detected_init_error = 0;

    bool if_detected_not_init = false;
    uint8_t error_code = 0; // 0:正常 1:断线 2:数据异常

    uint8_t counter_id = 0;

    void detect_error();
    time_counter(uint8_t counter_id_ = 0, float max_time_out_ms_ = 500.0f);

private:
};

class error_manager : public ITaskProcessor
{
public:
    void process_data();
    static time_counter *time_counter_ptr[18];
};

#endif

#endif