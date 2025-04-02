//
// Created by 15828 on 2025/4/2.
//

#include "Shoot_info.h"

float Shoot_info::Get_Current_Angle() {return Current_Angle;}
float Shoot_info::Get_Current_distance() {return Current_distance;}

float Shoot_info::Set_Target_distance(float Target_distance_){
    Target_distance=Target_distance_;
}

float Shoot_info::Speed_Cal(){
    float error=(Target_distance-Current_distance)*100;

    if (error>5) {
        return 1;
    }else if(error<5||error>2){
        return 0.2+(error/5)*0.8;
    }
}


void Shoot_info::process_data(){
    Get_Data();
    Current_Angle=Yaw_angle;
    Current_distance=Encoder::get_distance();


}