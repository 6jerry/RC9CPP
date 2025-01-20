#include "dt35.h"


dt35::dt35(UART_HandleTypeDef *huart, bool enableCrcCheck) : RC9Protocol(huart, enableCrcCheck){};

void process_data(){
    //接收数据存储到类内私有变量中
    for(int i = 0; i < 4; i+=4){
        for(int j = 0; j < 4; j++){
            dr_.data[j] = rx_frame_mat.rx_temp_data_mat[j];
        }
        
    }
}

