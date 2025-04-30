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
        float r = 0.0f;  // m
        float theta = 0.0f; // degree
        bool inverse = false;
        void tf_init(bool inverse_, float r_, float theta_); //顺时针（false） m , degree
        void coordinate_map(Vector2D *original, Vector2D *target, float now_theta);
};



#endif
#endif
