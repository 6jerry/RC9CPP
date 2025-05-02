#ifndef __WIT_GYRO_H
#define __WIT_GYRO_H

#include "Serial_Device.h"
#include "usart.h"
#include "wit_c_sdk.h"
#include <string.h>
#include <stdio.h>
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 常量定义 */
#define ACC_UPDATE      0x01
#define GYRO_UPDATE     0x02
#define ANGLE_UPDATE    0x04
#define MAG_UPDATE      0x08
#define READ_UPDATE     0x80
#define to_rad          0.0174533

/* 全局变量声明 */
extern uint8_t GYR_Buffer[1];
extern unsigned char ucTemp[1];
extern float fAngle[3], fAcc[3], fGyro[3];
extern volatile char s_cDataUpdate, s_cCmd;

/* 函数声明 */
void GYR_Init(void);
void GYR_Updata(void);
void CopeCmdData(unsigned char ucData);

#ifdef __cplusplus
}
#endif

/* C++ 类定义 */
#ifdef __cplusplus
class wit_gyro : public SerialDevice , public imu{
public:
    float Yaw_angle = 0,Pitch_angle = 0 ,Roll_angle = 0;
    UART_HandleTypeDef *huart_ = nullptr;

    wit_gyro(UART_HandleTypeDef *huartx) 
		: SerialDevice(huartx), huart_(huartx) 
		{
			GYR_Init();
		};

    void Get_Data() {
        GYR_Updata();
        Yaw_angle = fAngle[2] * to_rad;
				Pitch_angle = fAngle[1] * to_rad;
				Roll_angle = fAngle[0] * to_rad;
    };

    void handleReceiveData(uint8_t byte) {
        WitSerialDataIn(byte);
				Get_Data();
    }
    float get_yaw_rad() {
			return fAngle[2];
		};
		float get_pitch_rad() {
			return fAngle[1];
		};
		float get_roll_rad() {
			return fAngle[0];
		};
		
		float get_acc_x() {return fAcc[0];};
    float get_acc_y() {return fAcc[1];};
		float get_acc_z() {return fAcc[2];};
		
		float get_gyro_x() {return fGyro[0];};
    float get_gyro_y() {return fGyro[1];};
		float get_gyro_z() {return fGyro[2];};
};
#endif

#endif