
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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