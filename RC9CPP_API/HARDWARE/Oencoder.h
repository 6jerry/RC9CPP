#ifndef OENCODER_H
#define OENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "tim.h"
#include "imu.h"
#include <arm_math.h>
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define overflow_min 800
#define overflow_max 64735
#define max_count 65535
typedef struct
{
    TIM_HandleTypeDef *ecoder_tim = nullptr;
    uint32_t current_count = 0, last_count = 0;
    int32_t delta_count = 0;

} encoderwheel_t;

class Oencoder : public ITaskProcessor, public RC9subscriber
{
private:
    encoderwheel_t left_wheel, right_wheel;
    float delta_time = 0.0f;
    uint32_t previous_time = 0;
    imu *IMU;

    void calc_delta_count(encoderwheel_t *wheel);

    void pos_tf();

    const double pos_2_rpm = 6.103515625e-05, wheel_p = 119.4433527, COS45 = 0.70710678;

    double robot_delta_x = 0.0f, robot_delta_y = 0.0f; // mm

    double world_delta_x = 0.0f, world_delta_y = 0.0f; // mm

    double world_pos_x = 0.0f, world_pos_y = 0.0f; // mm

    float test_left = 0.0f, test_right = 0.0f;

    float data_send[6] = {0.0f};

public:
    void process_data();
    void add_imu(imu *IMU_);
    Oencoder(TIM_HandleTypeDef *left_encoder_tim, TIM_HandleTypeDef *right_encoder_tim, float wheel_perimeter_ = 119.443f);
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
    void init();

    void check_if_error();

    uint8_t left_wheel_ec = 0, right_wheel_ec = 0, imu_ec = 0; // 0:正常 1:掉线，2:数据异常

    bool detected_imu_error = false;
    uint32_t find_imu_error = 0;
};

#endif

#endif
