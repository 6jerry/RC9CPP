#include "test_laser.h"
#include <cstdint>
#include <stdio.h>

Laser::Laser(UART_HandleTypeDef *huart): SerialDevice(huart){
}

void Laser::init(){
	HAL_UART_Transmit(huart_, cmd_init1, 9, HAL_MAX_DELAY);
}
void Laser::handleReceiveData(uint8_t byte){
    volatile static uint8_t tmp[13] =  {0};
    volatile static uint8_t cnt = 0;
    volatile static uint32_t tmp_dis = 0.0f;
//	if(cnt < 13){
//			tmp[cnt] = byte;
//			cnt++;
//	}else{
//		cnt = 0;
//	}

	tmp[cnt] = byte;
	cnt++;
    switch(status) { 
         case head0:
            if(tmp[0] == 0xAA) {
                status = head1;
            }else{
                 status = head0;
                 cnt = 0;
            }
            break;
        case head1:
            if(tmp[1] == 0x00) {
                status = head2;
            }else{
                 status = head0;
                 cnt = 0;
            }
            break;
        case head2:
            if(tmp[2] == 0x00) {
                status = head3;
            }else{
                 status = head0;
                 cnt = 0;
            }
            break;
        case head3:
            if(tmp[3] == 0x22) {
                status = head4;
            }else{
                 status = head0;
                 cnt = 0;
            }
            break;
        case head4:
            if(tmp[4] == 0x00) {
                status = head5;
            }else{
                 status = head0;
                 cnt = 0;
            }
            break;
        case head5:
            if(tmp[5] == 0x03) {
                status = data;
            }else{
                 status = head0;
                 cnt = 0;
            }
            break;
        case data:
            if(cnt == 9){
                tmp_dis = tmp[6] << 24 | tmp[7] << 16 | tmp[8] << 8 | tmp[9];
				distance = (float)tmp_dis / 1000.0f;
                status = qua;
            }else{

            }
            break;
        case qua:
            if(cnt == 11){
                uint16_t tmp_qua = tmp[10] << 8 | tmp[11];
                status = crc;
            }else{

            }
            break;
        case crc:
            uint8_t crc_cal;
            for(int i = 1; i < 12; i++){
                crc_cal += tmp[i];
            }

			status = head0;
			cnt = 0;
            break;
        default:
            break;      
    }

}

void Laser::imu_rst(){

    HAL_UART_Transmit(huart_, cmd_init1, 9, HAL_MAX_DELAY);
}

float Laser::get_distance(void){
    return distance;
}