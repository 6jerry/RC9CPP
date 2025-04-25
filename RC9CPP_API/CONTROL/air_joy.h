#ifndef AIR_JOY_H
#define AIR_JOY_H

//#include <cstdint>
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

typedef struct {
    float LEFT_X_map;
    float LEFT_Y_map;
    float RIGHT_X_map;
    float RIGHT_Y_map;
    uint16_t SWA_map;
    uint16_t SWB_map;
    uint16_t SWC_map;
    uint16_t SWD_map;
} JoystickData;

class AirJoy {
    public:
    AirJoy();
    static void registerInstance(AirJoy* instance);
    void DataReceivedCallback();
    JoystickData Joy_msgs;

    void joymap_compute();
    static AirJoy* current_instance;
    
    private:
    uint16_t GPIO_Pin;
    uint32_t last_ppm_time, now_ppm_time=0;
    uint16_t ppm_time_delta=0;   //得到上升沿与下降沿的时间
    uint8_t ppm_ready=0,ppm_sample_cnt=0,ppm_update_flag=0;

    uint16_t LEFT_X=0,LEFT_Y=0,RIGHT_X=0,RIGHT_Y=0;
    uint16_t SWA=0,SWB=0,SWC=0,SWD=0;
    uint16_t PPM_buf[10]={0};   


};

#endif
#endif