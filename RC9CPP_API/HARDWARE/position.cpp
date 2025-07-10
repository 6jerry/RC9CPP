#include "position.h"

void position::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{

    world_pos.x = floatData[0] / 1000.0f;
    world_pos.y = floatData[1] / 1000.0f;
    world_yaw = floatData[2];
    // global->x = x_offset + local->x * arm_cos_f32_y + local->y * sin_y;
    // global->y = y_offset - local->x * sin_y + local->y * arm_cos_f32_y;
    float deltaxx = map_plot.x * arm_cos_f32(get_yaw_rad()) - map_plot.y * arm_sin_f32(get_yaw_rad()); // ��任��imuλ��
    float deltayy = map_plot.x * arm_sin_f32(get_yaw_rad()) + map_plot.y * arm_cos_f32(get_yaw_rad());

    real_world_pos.x = world_pos.x - deltaxx; // ��任��imuλ��
    real_world_pos.y = world_pos.y - deltayy;
    real_world_pos.y = -real_world_pos.y; // ��ת��y����
    real_world_pos.x = -real_world_pos.x; // ��ת��x

    uint32_t current_time = HAL_GetTick(); // 获取当前时间，单位ms

    if (previous_time != 0)
    { // 确保上一次时间不为0
      // 计算时间差，转换为秒
        delta_time = float(current_time - previous_time) / 1000.0f;

        if (delta_time > 0.0f)
        {
            D_P_s = (world_yaw - last_yaw) / delta_time; // 计算角度变化率
        }
    }

    previous_time = current_time;
    last_yaw = world_yaw; // 更新上一次的角度

    /*



   // ʹ����ʱ����ϵ�任
   Vector2D world_pos_ = world_pos;
   world_pos_.y = -world_pos_.y;
   tf_.coordinate_map(&world_pos, &real_world_pos, 0.225f, -get_yaw_rad(), map_plot); //??????
   // ???????
   if(!tf_.map_origin_init_flag){
       tf_.map_origin = real_world_pos;
       tf_.map_origin_init_flag = true;
   }
   tf_.localize_with_diff(&real_world_pos);
   // ӳ���y����任��ԭ����ϵ
   real_world_pos.y = -real_world_pos.y;
   */
}

Vector2D position::get_world_pos()
{
    return real_world_pos;
}

float position::get_heading()
{
    return world_yaw;
}

float position::get_yaw_rad()
{
    return world_yaw * 0.0174532925f;
}

float position::get_world_pos_x()
{
    return real_world_pos.x;
}

float position::get_world_pos_y()
{
    return real_world_pos.y;
}

float position::get_yaw_speed()
{
    return D_P_s;
}

void position::imu_relocate(float x, float y, float angle)
{
    float deltaxx = map_plot.x * cos(get_yaw_rad()) - map_plot.y * sin(get_yaw_rad()); // 逆变换回imu位置
    float deltayy = map_plot.x * sin(get_yaw_rad()) + map_plot.y * cos(get_yaw_rad());

    x = x + deltaxx; // 逆变换回imu位置
    y = y + deltayy;

    float send_datas[2] = {x, y};

    sendFloatData(1, send_datas, 2);
}

void position::imu_rst() // 让position的imu重启
{
    float send_datas[2] = {map_plot.x, map_plot.y};
    sendFloatData(2, send_datas, 2);
}

void position::set_map_plot(float x_, float y_)
{
    map_plot.x = x_;
    map_plot.y = y_;
}