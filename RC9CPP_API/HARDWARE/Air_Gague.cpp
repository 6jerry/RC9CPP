#include "Air_Gague.h"
#include <cstdint>

Air_gague::Air_gague(UART_HandleTypeDef *huart, uart_type type) : SerialDevice(huart, type){ 

    get_air_presure();
}

void Air_gague::handleReceiveData(uint8_t byte){
    static uint8_t state = 0;
    if(state == 0 && byte == 0x12){
        state = 1;
        return;
    }
    else if(state == 1 && byte == 0x04){ 
        state = 2;
        return;
    }
    else if(state == 2 && byte == 0x02){
        state = 3;
        return;
    }
    else if(state == 3){
        state = 4;
        air_data.tmp[1] = byte; 
        return;
    }
    else if(state == 4){
        state = 5;
        air_data.tmp[0] = byte;
    }else{
        state = 0;
        return;
    }
}

float Air_gague::get_air_presure(void){
    HAL_UART_Transmit(huart_, cmd, 8, HAL_MAX_DELAY);
    return ((float)air_data.data / 1000.0f);
}

void Air_gague::process_data(){
    air_pressure = get_air_presure();
}