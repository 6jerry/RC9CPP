#pragma  once 

#ifdef __cplusplus
extern "C" 
{
#endif
#include "RC9Protocol.h"
#include "Vector2D.h"
#include "imu.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


class dt35 : public RC9subscriber, public imu{
    public:
        //信息储存结构体
    struct{
        Vector2D world_pos;
        float yaw_angle = 0.0f;
    } calibration_loaction;
    float data_tmp [4] = {0.0f};
    // 拟合参数
    float p[4] = {4.12f, 0.01f, 0.01f, 4.09f};
    float b[4] = {-40.0f, 0.01f, 0.01f, -20.0f};
    // 信息获取接口
    Vector2D get_world_pos() override;
    //float get_yaw_angle() override;
    float get_heading() override;
	float get_yaw_rad() override;

    public:
        void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
};


#endif // __cplusplus