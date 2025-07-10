
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

#include "fdcan_device.h"

#include "SuperPID.h"
#include "motor.h"
#include "encoder.h"
#include "PID.h"
#include "TrapezoidalPlanner.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define M3508_KT 0.01562f // 3508的内圈转矩常数 单位：N.M/A
#define M3508_G 19.2032f  // 最常见的3508减速比，还是建议不要直接取整，0.2也不算小了

#define M3508_MAX_CURRENT 20000.0f // mA
#define M3508_CURRENT_MAP 16384    // 电流映射值

#define M2006_MAX_CURRENT 10000.0f // mA
#define M2006_CURRENT_MAP 10000

#define M2006_G 36.0f
#define M3508_MAXT 0.15622f // 内圈最大转矩
#define PI 3.14159265f

// PID输入速度误差，输出内圈期望转矩

enum m3508_mode
{
    m3508_increPID_speed,
    m3508_pid_speed,
    m3508_angle_pid,
    m3508_angle_speedplan,
    m3508_distance_pid,
    m3508_distance_speedplan,
    m3508_F
};

class m3508p : public CanDevice,
               public power_motor,
               public dji_motor
               
{
private:
    float gear_ratio = 19.2032f, wheel_perimeter = 0.0f, wheel_R = 0.0f, v_2_rpm_k = 0.0f, rpm_2_v_k = 0.0f, speed_plan_end_dis = 0.2f, min_start_rpm = 60.0f, speed_plan_end_pos = 2.0f;

    uint8_t time_cnt = 0;

    m3508_mode work_mode = m3508_F;

    

    int16_t increPID_speed();
    int16_t pid_speed();
    int16_t angle_pid();
    int16_t angle_speedplan();
    int16_t distance_pid();
    int16_t distance_speedplan();
    int16_t F();

    float v_2_rpm(float v);
    float rpm_2_v(float rpm_);

public:
    m3508p(uint32_t can_id, FDCAN_HandleTypeDef *hcan_, float gear_ratio = M3508_G, float max_c = M3508_MAX_CURRENT, int16_t max_c_m = M3508_CURRENT_MAP);

    Encoder *encoder = nullptr;
    int16_t motor_process() override;
    void can_update(uint8_t can_RxData[8]);
    void add_encoder(Encoder *encoder_){encoder = encoder_;}
    float target_angle = 0.0f, target_distance = 0.0f, angle_error = 0.0f, dis_error = 0.0f;
    float target_rpm = 0.0f;

    float targrt_T = 0.0f; // 期望转矩

    float last_pos = 0.0f, delta_pos = 0.0f, pos_sum = 0.0f, all_pos = 0.0f, now_pos = 0.0f, temp_delta = 0.0f, dis_sum = 0.0f;
    uint8_t init_cnt = 0;
    bool if_init = true;

    void many_pos_locate(); // 3508多圈差分定位
    void locate_restart();

    void relocate_dis(float dis) override;
    float get_dis() override;

    // 动力电机通用接口
    float get_rpm();
    void set_rpm(float power_motor_rpm);

    void set_pos(float pos) override;
    float get_pos() override;
    void relocate_pos(float angle) override;
    void set_dis(float dis) override;
    void set_current(float target_c_) override;
    bool set_dis_speedplan(float targetdis, float max_speed, float max_acc, float max_dec, float finalspeed) override;
    bool set_pos_speedplan(float target_angle_, float max_speed, float max_acc, float max_dec, float finalspeed) override;

    void dis_speedplan_restart() override;
    void pos_speedplan_restart() override;

    void config_mech_param(float gear_ratio_, float wheel_D_);

    float get_target_rpm() override;
    float get_target_pos() override;
    float get_target_dis() override;

    IncrePID rpm_control;

    pid distance_pid_control, angle_pid_control, rpm_pid_control;

    TrapezoidalPlanner1D dis_speed_plan;
    TrapezoidalPlanner1D pos_speed_plan;

   
};

#endif

#endif