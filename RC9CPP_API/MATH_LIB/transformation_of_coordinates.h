#ifndef COORDINATE_TF_H
#define COORDINATE_TF_H
#include "Vector2D.h"
#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class tf{
    public:
		float offset_x = 0.0f;
		float offset_y = 0.0f;
		float r = 0.0f;  // m
		float rad = 0.0f; // radium
		Vector2D map_origin; // different point
		bool map_origin_init_flag = false; //different mark flag 
		void localize_with_diff(Vector2D* pos); //差分定位
		void localize_with_diff_inverse(Vector2D* pos); //差分定位逆映射
		void coordinate_map(Vector2D *original, Vector2D *target, float r_,float now_rad); //源坐标，目标坐标，极坐标半径，弧度
		void coordinate_map_inverse(Vector2D *original, Vector2D *target, float r_, float now_rad); //源坐标，目标坐标，极坐标半径，弧度(逆变换)
};



#endif
#endif
