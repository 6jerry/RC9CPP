#ifndef TIME_COUNTER_H
#define TIME_COUNTER_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class time_counter
{
public:
    float get_DeltaTime_ms();

    static void init_time_counter();
    float delta_time_ms = 0.0f;

private:
  
    uint32_t now_cnt = 0, last_cnt = 0;
};









#endif

#endif