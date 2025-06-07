/*******************************************************************************
 * @file M6020.h
 * @author 6Jerry (1517752988@qq.com)
 * @brief m6020 motor type.
 * @version 1.0
 * @date 2024-10-26
 *
 * @copyright Copyright (c) 2024-10-26 6Jerry
 *
 * @license MIT
 *
 * @disclaimer This software is provided "as is", without warranty of any kind, express or implied,
 *             including but not limited to the warranties of merchantability, fitness for a
 *             particular purpose and noninfringement. In no event shall the authors be liable for any
 *             claim, damages or other liability, whether in an action of contract, tort or otherwise,
 *             arising from, out of or in connection with the software or the use or other dealings
 *             in the software.
 ******************************************************************************/
#ifndef M6020_H
#define M6020_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_device.h"
#include "pid.h"
#include "motor.h"
#include <math.h>
#include "SuperPID.h"
#include "RC9Protocol.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum m6020_mdoe
{
    m6020_servo_pid,        // pid角度控制
    m6020_servo_speed_plan, // 梯形速度控制
    m6020_rpm_pid,          // pid转速控制
    m6020_F                 // 力矩控制
};

class m6020s : public CanDevice, public dji_motor, public power_motor, public RC9subscriber
{
private:
    uint8_t gear_ratio = 1;

    m6020_mdoe work_mode = m6020_F;

    const float TorqueConstant = 0.741f; // 转矩常数，单位为 N·m/A

    float send_vc = 0.0f;

    int16_t
    servo_pid();
    int16_t servo_speed_plan();
    int16_t rpm_ctrl();
    int16_t F_ctrl();

    bool ebable_debug = false;

public:
    m6020s(uint8_t can_id, CAN_HandleTypeDef *hcan_);

    int16_t motor_process() override;
    void can_update(uint8_t can_RxData[8]);
    float target_angle = 0;
    float target_rpm = 0, target_F = 0, real_F = 0.0f;
    int16_t target_v = 0;

    float real_angle = 0.0f, target_test_v = 0.0f;
    float angle_error = 0.0f;

    superpid rpm_pid;

    bool if_double_control = true; // 是否使用双环控制，因为有些电机无法360度旋转导致无法调速度环
    pid pos_pid;

    float get_relative_pos();
    float get_absolute_pos();
    void set_relative_pos(float relative_pos_);
    void set_absolute_pos_multi(float absolute_pos_multi_);
    void set_absolute_pos_single(float absolute_pos_single_);
    void relocate(float new_zero_point);

    float convert_angle_to_signed(float current_angle);

    float get_rpm();
    void set_rpm(float power_motor_rpm);
    float get_pos() override;
    void set_pos(float pos) override;

    void set_F(float F_) override;
    float get_F() override;

    void target_angle_tf();
    void set_init_angle(float init_angle_);
    float delta_angle = 0.0f, init_angle = 0.0f, target_relative_angle = 0.0f;

    void config_rpm_pid(float kp, float ki, float kd, float output_limit, float deadzone, float integral_separation_threshold);

    void start_debug();
};

#endif

#endif