#include "air_joy.h"

AirJoy air_joy;



void AirJoy::DataReceivedCallback(uint16_t GPIO_Pin)
{
    last_ppm_time=now_ppm_time;
    now_ppm_time=HAL_GetTick();  //获取当前时间
    ppm_time_delta=now_ppm_time-last_ppm_time;  //电平数据

    //开始解包PPM信号
	if(ppm_ready==1)	//判断帧结束后，进行下一轮解析
	{
    //帧结束电平至少2ms=2000us(留点余量)
    //由于部分老版本遥控器、接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
		if(ppm_time_delta >= 2100)  //帧头
		{
			ppm_ready = 1;
			ppm_sample_cnt=0;   //对应的通道值
		} 
		else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//单个PWM脉宽在1000-2000us，这里设定950-2050，提升容错
		{         
			
			PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//对应通道写入缓冲区 
			
			if(ppm_sample_cnt>=8)   //单次解析结束0-7表示8个通道。如果想要使用10通道，使用ibus协议(串口接收)
			{
        		LEFT_X=PPM_buf[3]; LEFT_Y=PPM_buf[2]; RIGHT_X=PPM_buf[0]; RIGHT_Y=PPM_buf[1];
        		SWA=PPM_buf[4]; SWB=PPM_buf[5]; SWC=PPM_buf[6]; SWD=PPM_buf[7];
				ppm_ready=0;
				ppm_sample_cnt=0;
			}
		}
		else  
            ppm_ready=0;
	}
    else if(ppm_time_delta>=2100)//帧尾电平至少2ms=2000us
	{
		ppm_ready=1;
		ppm_sample_cnt=0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    AirJoy.DataReceivedCallback(GPIO_Pin);
}