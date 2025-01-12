#ifndef VESC_H
#define VESC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include "can_device.h"
#include "TaskManager.h"
#include "motor.h"
#include "SuperPID.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define CAN_CMD_SET_CURRENT 0x01 // VESC设置电流的命令ID
class vesc : public CanDevice, public ITaskProcessor, public power_motor
{
private:
    float gear_ratio = 1.0f;
    uint8_t motor_polse = 7;

public:
    float get_rpm();
    void set_rpm(float power_motor_rpm);
    // void set_rpm_ff(float power_motor_rpm, float ff) override;

    void can_update(uint8_t can_RxData[8]);
    void process_data();

    uint32_t extid = 0;

    vesc(uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t motor_polse_ = 7, float gear_ratio_ = 1.0f, float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float r_ = 0.0f);

    IncrePID rpm_control;

    float target_rpm = 0.0f, now_rpm = 0.0f, rcurrent = 0.0f;
    int32_t target_current = 0;
};
#endif
#endif
