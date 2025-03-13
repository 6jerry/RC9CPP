
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

#ifndef __GYRO_FUC_H
#define __GYRO_FUC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*在此处引用外部文件：       begin*/	
#include "Serial_device.h"
#include "gyro.h"
#include "wit_c_sdk.h"
	
/*引用外部文件end*/	

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/*在此处进行宏定义：         begin*/	

/*宏定义end*/	


/*在此处进枚举类型定义：         begin*/	

/*枚举定义end*/	 


/*在此处进行类和结构体的定义：begin*/	
class GYRO: public SerialDevice
{
	public:
		float Yaw_angle = 0;
		float Roll_angle =  0;
		float Pitch_angle = 0;
		UART_HandleTypeDef *huart_ = NULL;
		GYRO(UART_HandleTypeDef *huartx);
		void Get_Data();
		void handleReceiveData(uint8_t byte);
};
/*类和结构体定义end*/	


/*在此处进行函数定义：       begin*/	

/*函数定义end*/	

#endif

#endif 
