#ifndef __R3_TEST_H__
#define __R3_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "M3508.h"
#include "vesc.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "fdcan_device.h"
#include "gpio.h"
#include "robot_chassis.h"
#include "ros_sensor.h"
#include "position.h"
#include "R3_testxbox.h"
#include "auto_yunball_R3.h"
#include "camera.h"
  void r3_setup();
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class demo : public ITaskProcessor, public RC9subscriber
{
public:
  int flag = 0;
  uint32_t last_tick = 0;
  uint32_t last_tick2 = 0;
  void encoder_check();
  void process_data();
  void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
  R3_xbox *xbox;
  float recive_data[4] = {0};
  float send_data[6] = {0};

  void add_xbox(R3_xbox *xbox_);

  float pian_x;
  float pian_y;

  Vector2D robot_pos;
  Vector2D robot_v;
};
#endif
#endif
