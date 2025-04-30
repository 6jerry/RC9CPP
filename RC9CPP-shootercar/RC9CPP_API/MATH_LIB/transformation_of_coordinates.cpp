#include "transformation_of_coordinates.h"
#include <arm_math.h>

#define PI 3.141592653589793f
/* 确定在世界坐标系下的旋转方向（角度增大的方向）flase为顺时针
 * 确定安装位置距机械中心半径
 * 确定初始角度
 */
void tf::tf_init(bool inverse_, float r_, float theta_){ 
    inverse = inverse_;
    r = r_;
    theta = theta_;
}

void tf::coordinate_map(Vector2D *original, Vector2D *target, float now_theta){
    if(inverse){ //左手坐标系（action角度取负过一次，参数需要再取反获得原始数据；安装方向旋转了180度，x需要取反（y读取已经取反）
        float offset_x =  r * arm_cos_f32((now_theta + theta) * 0.0174532925); 
        float offset_y =  r * arm_sin_f32((now_theta + theta) * 0.0174532925);
        target -> x = -original -> x - offset_x ;
        target -> y = original -> y - offset_y ;
	}else{ // 右手坐标系（ros），需要翻转为左手系再进行映射（ros读取时，y和angle进行了取反，注意三角函数符号）
        float offset_x =  r * arm_cos_f32(((180 - (now_theta + theta)) * 0.0174532925)); 
        float offset_y =  r * arm_sin_f32(((180 - (now_theta + theta)) * 0.0174532925));
        target -> x = -original -> x - offset_x ;
        target -> y = original -> y - offset_y ;
    }
}