
/*******************************************************************************
 * @file M3508.h
 * @author 6Jerry (1517752988@qq.com)
 * @brief m3508 motor type.
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
#ifndef M3508_H
#define M3508_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_device.h"
#include "SuperPID.h"
#include "motor.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define M3508_KT 0.01562f   // 3508的内圈转矩常数 单位：N.M/A
#define M3508_G 19.2032f    // 最常见的3508减速比，还是建议不要直接取整，0.2也不算小了
#define M3508_MAXT 0.15622f // 内圈最大转矩

// PID输入速度误差，输出内圈期望转矩

class m3508p : public CanDevice, public power_motor, public dji_motor // 动力电机版本的m3508
{
private:
    float gear_ratio = 19.2032f;

public:
    m3508p(uint8_t can_id, CAN_HandleTypeDef *hcan_, float gear_ratio = M3508_G, float kp_ = 32.0f, float ki_ = 0.76f, float kd_ = 8.6f, float r_ = 106.0f);

    int16_t motor_process() override;
    void can_update(uint8_t can_RxData[8]);
    float rtarget_angle = 0;
    float target_rpm = 0;

    float targrt_T = 0.0f;        // 期望转矩
    float Tff = 0.0f, Cff = 0.0f; // 前馈力矩和换算成的前馈电流

    // 动力电机通用接口
    float get_rpm();
    void set_rpm(float power_motor_rpm);
    void set_fTff(float Tff_); // 设置动摩擦力前馈补偿

    void T_TO_C(); // 力矩转换为电流

    // bool if_stalling(); // 判断电机是否堵转

    IncrePID rpm_control;
};

#endif

#endif