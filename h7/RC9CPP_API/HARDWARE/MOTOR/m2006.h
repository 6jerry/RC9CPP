#ifndef M2006_H
#define M2006_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "fdcan_device.h"

#include "SuperPID.h"
#include "motor.h"
#include "RC9Protocol.h"
#include "PID.h"
#include "TrapezoidalPlanner.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define M2006_GERR_RATIO 36.0f

enum m2006_mode
{
    m2006_increPID_speed,
    m2006_pid_speed,
    m2006_angle_pid,
    m2006_angle_speedplan
};

class m2006 : public CanDevice,
              public power_motor,
              public dji_motor,
              public RC9subscriber
{
private:
public:
    m2006(uint32_t can_id, FDCAN_HandleTypeDef *hcan_, float gear_ratio = M2006_GERR_RATIO);
    int16_t motor_process() override;
    void can_update(uint8_t can_RxData[8]);
};

#endif

#endif
