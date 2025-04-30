#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "freertos_task.h"
// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Client");
  create_freertos_tasks();
  // xboxController.begin();
}

void loop()
{

}
