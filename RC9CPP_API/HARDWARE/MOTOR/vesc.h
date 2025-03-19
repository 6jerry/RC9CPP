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
#include "filters.h"
#include "RC9Protocol.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define CAN_CMD_SET_CURRENT 0x01 // VESC设置电流的命令ID
#define CAN_CMD_SET_ERPM 0x03
#define CAN_CMD_SET_BRAKE 0x02

enum vesc_mode
{
    vesc_current,
    vesc_erpm,
    vesc_rpm_increpid

};
class vesc : public CanDevice, public ITaskProcessor, public power_motor, public RC9subscriber
{
private:
    float gear_ratio = 3.0f;
    float motor_polse = 7.0f;

    vesc_mode vesc_mode = vesc_current;

    void erpm_mode();
    void current_mode();
    void rpm_increpid_mode();
    SimpleMeanFilter rpm_filter;

    bool debug_mode = false;

public:
    float
    get_rpm();
    void set_rpm(float power_motor_rpm);
    // void set_rpm_ff(float power_motor_rpm, float ff) override;

    void set_current(float target_c_) override;
    void send_rpm(float power_motor_rpm) override;
    void set_ff_current(float target_c_) override;
    void can_update(uint8_t can_RxData[8]);
    void process_data();

    void start_debug();
    float delta_rpm = 0.0f;

    uint32_t extid = 0;

    vesc(uint8_t can_id_, CAN_HandleTypeDef *hcan_, uint8_t motor_polse_ = 21, float gear_ratio_ = 3.0f, float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float r_ = 0.0f);

    IncrePID rpm_control;

    float target_rpm = 0.0f, now_rpm = 0.0f, rcurrent = 0.0f, target_current = 0.0f, filted_rpm = 0.0f, target_ff_current = 0.0f, test_rpm = 0.0f;
    int32_t target_erpm = 0, senderpm = 0, brake = 10000, send_current = 0;
};
#endif
#endif
