#include "dt35.h"

void dt35::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount){
    if(byteCount == 16){
        data_tmp[0] = floatData[0] * p[0] + b[0];
        data_tmp[1] = floatData[1] * p[1] + b[1];
        data_tmp[2] = floatData[2] * p[2] + b[2];
        data_tmp[3] = floatData[3] * p[3] + b[3];
    }

}

float dt35::get_heading(){
    return calibration_loaction.yaw_angle;
}

float dt35::get_yaw_rad(){
    return calibration_loaction.yaw_angle * 3.14159265358979323846 / 180.0;
}

Vector2D dt35::get_world_pos(){
    return calibration_loaction.world_pos;
}