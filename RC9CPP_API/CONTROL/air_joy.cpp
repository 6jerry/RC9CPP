#include "air_joy.h"

AirJoy* AirJoy::current_instance = nullptr;

AirJoy::AirJoy() {
    registerInstance(this);
}


void AirJoy::registerInstance(AirJoy* instance) {
    current_instance = instance;
}

void AirJoy::DataReceivedCallback()
{
    last_ppm_time=now_ppm_time;
    now_ppm_time=TIM6->CNT;  //获取当前时间
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
        		RIGHT_X=PPM_buf[3]; RIGHT_Y=PPM_buf[2]; LEFT_X=PPM_buf[0]; LEFT_Y=PPM_buf[1];
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

void AirJoy::joymap_compute()
{
	if(LEFT_X>1450 && LEFT_X<1550)    Joy_msgs.LEFT_X_map = 0;
	else if(LEFT_X>900 && LEFT_X<2100) Joy_msgs.LEFT_X_map = -(LEFT_X-1500.0)/500.0;
	else Joy_msgs.LEFT_X_map = 0;
	
	if(LEFT_Y>1450 && LEFT_Y<1550)    Joy_msgs.LEFT_Y_map = 0;
	else if(LEFT_Y>900 && LEFT_Y<2100) Joy_msgs.LEFT_Y_map = (LEFT_Y-1500.0)/500.0;
	else Joy_msgs.LEFT_Y_map = 0;
	
	if(RIGHT_X>1450 && RIGHT_X<1550)    Joy_msgs.RIGHT_X_map = 0;
	else if(RIGHT_X>900 && RIGHT_X<2100) Joy_msgs.RIGHT_X_map = -(RIGHT_X-1500.0)/500.0;
	else Joy_msgs.RIGHT_X_map = 0;
	
	if(RIGHT_Y>1450 && RIGHT_Y<1550)    Joy_msgs.RIGHT_Y_map = 0;
	else if(RIGHT_Y>900 && RIGHT_Y<2100) Joy_msgs.RIGHT_Y_map = (RIGHT_Y-1500.0)/500.0;
	else Joy_msgs.RIGHT_Y_map = 0;
	
	if(SWA>950&&SWA<1050)  	Joy_msgs.SWA_map = 1;
	else if(SWA>1950&&SWA<2050)  Joy_msgs.SWA_map = 2;
	else Joy_msgs.SWA_map = 0;
	
	if(SWB>950&&SWB<1050)  	Joy_msgs.SWB_map = 1;
	else if(SWB>1450&&SWB<1550)  Joy_msgs.SWB_map = 2;
	else if(SWB>1950&&SWB<2050)  Joy_msgs.SWB_map = 3;
	else Joy_msgs.SWB_map = 0;
	
	if(SWC>950&&SWC<1050)  	Joy_msgs.SWC_map = 1;
	else if(SWC>1450&&SWC<1550)  Joy_msgs.SWC_map = 2;
	else if(SWC>1950&&SWC<2050)  Joy_msgs.SWC_map = 3;
	else Joy_msgs.SWC_map = 0;
	
	if(SWD>950&&SWD<1050)  	Joy_msgs.SWD_map = 1;
	else if(SWD>1950&&SWD<2050)  Joy_msgs.SWD_map = 2;
	else Joy_msgs.SWD_map = 0;
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

		if (AirJoy::current_instance != nullptr)
        AirJoy::current_instance->DataReceivedCallback();

}