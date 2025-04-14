#ifndef CHASSIS_XBOX_H
#define CHASSIS_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"
#include "TaskManager.h"
#include "motor.h"
#include "gpio.h"
#include "TrapezoidalPlanner.h"
#include "robot_chassis.h"
#include "Vector2D.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class chassis_xbox : public xbox_debug_base, public chassis_user
{
private:
    Vector2D t_points[2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

public:
    void not_start() override;
    void mode_0() override;
    void mode_2() override;
    void mode_1() override;
    void mode_3() override;

};

#endif
#endif