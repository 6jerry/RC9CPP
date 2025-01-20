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
<<<<<<< HEAD
#include "PID.h"
=======
#include "SuperPID.h"
>>>>>>> 40b0e7df49798d9cb74baf2e9eea4ee4dc4618a2

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define CAN_CMD_SET_CURRENT 0x01 // VESC设置电流的命令ID
<<<<<<< HEAD
#define CAN_CMD_SET_ERPM 0x03
#define CAN_CMD_SET_BRAKE 0x02
class vesc : public CanDevice, public ITaskProcessor, public power_motor
{
private:
    float gear_ratio = 3.0f;
=======
class vesc : public CanDevice, public ITaskProcessor, public power_motor
{
private:
    float gear_ratio = 1.0f;
>>>>>>> 40b0e7df49798d9cb74baf2e9eea4ee4dc4618a2
    uint8_t motor_polse = 7;

public:
    float get_rpm();
    void set_rpm(float power_motor_rpm);
    // void set_rpm_ff(float power_motor_rpm, float ff) override;

    void can_update(uint8_t can_RxData[8]);
    void process_data();
<<<<<<< HEAD
    float delta_rpm = 0.0f;
    bool if_invert = false;
    uint32_t extid = 0;

    vesc(uint8_t can_id_, CAN_HandleTypeDef *hcan_, bool invert = false, uint8_t motor_polse_ = 21, float gear_ratio_ = 3.0f, float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float r_ = 0.0f);

    pid rpm_control;

    float target_rpm = 0.0f, now_rpm = 0.0f, rcurrent = 0.0f;
    int32_t target_erpm = 0, senderpm = 0, brake = 10000;
=======

    uint32_t extid = 0;

    vesc(uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t motor_polse_ = 7, float gear_ratio_ = 1.0f, float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float r_ = 0.0f);

    IncrePID rpm_control;

    float target_rpm = 0.0f, now_rpm = 0.0f, rcurrent = 0.0f;
    int32_t target_current = 0;
>>>>>>> 40b0e7df49798d9cb74baf2e9eea4ee4dc4618a2
};
#endif
#endif
