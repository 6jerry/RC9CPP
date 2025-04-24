#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class AirJoy {
    public:
    //AirJoy(uint16_t GPIO_Pin_);
    void DataReceivedCallback(uint16_t GPIO_Pin);

    private:
    uint16_t GPIO_Pin;
    uint32_t last_ppm_time, now_ppm_time=0;
    uint16_t ppm_time_delta=0;   //得到上升沿与下降沿的时间
    uint8_t ppm_ready=0,ppm_sample_cnt=0,ppm_update_flag=0;

    uint16_t LEFT_X=0,LEFT_Y=0,RIGHT_X=0,RIGHT_Y=0;
    uint16_t SWA=0,SWB=0,SWC=0,SWD=0;
}


#endif