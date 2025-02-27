#include "GYRO_fuc.h"
#define to_rad 0.01745
GYRO::GYRO(UART_HandleTypeDef *huartx)
     :SerialDevice(huartx)
{
	huart_ = huartx;
}

void GYRO::Get_Data()
{
	 GYR_Updata();
	 Yaw_angle = fAngle[2]*to_rad;
	 Pitch_angle = fAngle[1]*to_rad;
	 Roll_angle =  fAngle[0]*to_rad;
}

void GYRO::handleReceiveData(uint8_t byte)
{
//	WitSerialDataIn(GYR_Buffer[0]);
	WitSerialDataIn(byte);
//	HAL_UART_Receive_IT(&huart5, (uint8_t *)GYR_Buffer, 1);
}