#include "transformation_of_coordinates.h"
#include <arm_math.h>

#define PI 3.141592653589793f
/* 
 * 确定安装位置距机械中心半径
 */


inline void tf::coordinate_map(Vector2D *original, Vector2D *target, float r,float now_rad){

        float offset_x =  r * arm_cos_f32(now_rad); 
        float offset_y =  r * arm_sin_f32(now_rad);
        target -> x = original -> x + offset_x ;
        target -> y = original -> y - offset_y ;

}

inline void tf::coordinate_map_inverse(Vector2D *original, Vector2D *target, float r,float now_rad){

        float offset_x =  r * arm_cos_f32(now_rad); 
        float offset_y =  r * arm_sin_f32(now_rad);
        target -> x = original -> x - offset_x ;
        target -> y = original -> y + offset_y ;

}