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
