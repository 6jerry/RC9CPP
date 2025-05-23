#include "transformation_of_coordinates.h"
#include <arm_math.h>

#define PI 3.141592653589793f
/* 
 * 确定安装位置距机械中心半径
 */

//差分定位
void tf::localize_with_diff(Vector2D* pos){
    pos->x -= map_origin.x;
    pos->y -= map_origin.y;
}

//差分定位逆映射
void tf::localize_with_diff_inverse(Vector2D* pos){
    pos->x += map_origin.x;
    pos->y += map_origin.y;
}

// 变换到旋转中心
void tf::coordinate_map(Vector2D *original, Vector2D *target, float r_,float now_rad){
		r = r_;
        offset_x =   r * arm_cos_f32(now_rad); 
        offset_y =   r * arm_sin_f32(now_rad);
        target -> x = original -> x - offset_x ;
        target -> y = original -> y - offset_y ;

}

// 逆变换回imu位置
void tf::coordinate_map_inverse(Vector2D *original, Vector2D *target, float r_,float now_rad){
		r = r_;
        offset_x =  r * arm_cos_f32(now_rad); 
        offset_y =  r * arm_sin_f32(now_rad);
        target -> x = original -> x + offset_x ;
        target -> y = original -> y + offset_y ;

}
