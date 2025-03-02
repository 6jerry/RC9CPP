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
#include "PID.h"
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

#define left_foward 165
#define right_foward 197
#define left_inside 260
#define right_inside 92
#define left_outside 128
#define right_outside 240
#define left_ready_for_ball 190
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


enum catch_ball_state
{
    catch_ball_standby,
    catch_ball_locking_ball,
    catch_ball_lock_yaw,
    catch_ball_ball_entered,
    catch_ball_move_to_throw_ball_point,
    catch_ball_ready_to_throw_ball,
    catch_ball_rush_to_throw_ball,
 

};

class catch_ball_fsm : public ITaskProcessor, public chassis_user, public RC9subscriber
{
private:
    

    catch_ball_state state = catch_ball_standby;

    uint32_t time_cnt = 0;

    uint8_t prio_code = 0, if_finish = 0;

    ball_info ball_info_;

    pid ball_locker_pid;

    float catch_ball_speed = 0.3f, entererd_ball_size = 500.0f, throw_ball_speed = 1.1f;

    Vector2D throw_point;

public:
    void process_data();
    catcher_fsm *catcher = nullptr;

    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

   void start_catch_ball();
   void stop_catch_ball();

   bool if_finish_catch_ball();

   void set_throw_point(Vector2D point);

    

    
};

#endif
#endif