#ifndef GCFSM_H
#define GCFSM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "motor.h"
#include "TaskManager.h"
#include <arm_math.h>
#include "RC9Protocol.h"
#include "robot_chassis.h"
#include "servo.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

enum GCCLAW_STATE
{
    claw_hold,

    claw_open_1ball,
    claw_open_2ball,
    claw_open_3ball,

    claw_grab_1ball,
    claw_grab_2ball,
    claw_grab_3ball,

    claw_move_1ball,

    claw_throw_ball,
};

#define left_foward 70
#define right_foward 197
#define left_inside 170
#define right_inside 96
#define left_outside 40
#define right_outside 235
#define left_ready_for_ball 92
#define right_ready_for_ball 174

class catcher_fsm : public ITaskProcessor
{
private:
    servo *servo_left = nullptr, *servo_right = nullptr;
    GCCLAW_STATE claw_state = claw_hold;

    uint32_t time_cnt = 0;

public:
    void add_servo(servo *servo_left_, servo *servo_right_);
    void process_data();

    void hold_claw();
    void move_ball();
    void ready_catch_ball();
    void throw_ball();
};

enum GCSTATE
{
    wait_init,
    go_to_first_catch_ball_point,
    serch_ball,
    catching_ball,
    ball_entered,
    go_to_throw_ball_point,
    ready_to_throw_ball,
    rush_to_throw_ball,
    back_to_serch_ball_point,
};

typedef struct
{
    float x_dis = 0.0f, y_dis = 0.0f; // 左右的虚拟位置和框的大小

    bool if_conflict = false; // 是否有冲突，无冲突就正常捡球，有冲突就找球
} ball_info;

class GC_fsm : public ITaskProcessor, public chassis_user, public RC9subscriber
{
private:
    catcher_fsm *catcher = nullptr;

    GCSTATE state = wait_init;

    ball_info ball_info_;

    uint8_t gc_priocode = 1;

    Vector2D f_ball_c_point, f_ball_point, throw_point;

    const float f_ball_y = 0.0f, throw_ball_y = 90.0f;

    uint32_t time_cnt = 0;

public:
    void process_data();
    // void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
    GC_fsm();
};

#endif
#endif