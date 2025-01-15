#include "pure_pursuit.h"

void pure_pursuit::compute_error()
{
    // 通过当前点，目标向量，目标向量的起点和终点，计算得到法向追踪误差和切向追踪距离

    now_dis = now_point - target_line_initpoint;

    projected = now_dis.project_onto(target_line); // 计算投影向量

    tangent_dis = projected.magnitude();               // 追踪当前目标向量的距离，即切向追踪距离
    project_point = projected + target_line_initpoint; // 计算得到投影点的坐标

    Vector2D normal_vector = project_point - now_point; // 法向量

    normal_dis = normal_vector.magnitude(); // 法向误差

    tangent_dir = -target_line.normalize(); // 切向单位向量，指向未追踪的点

    normal_dir = normal_vector.normalize(); // 法向单位向量，指向目标向量
}

void pure_pursuit::purepusit_normal_control()
{
    float nor_speed = normal_control.PID_ComputeError(normal_dis); // 法向纠偏速度的大小
    Vector2D normal_speed = nor_speed * normal_dir;                // 法向速度矢量
    Vector2D tangent_speed = tan_speed * tangent_dir;              // 切向速度矢量

    target_wspeed = normal_speed + tangent_speed; // 最终输出的期望速度合矢量
}
void pure_pursuit::purepusit_dir_control()
{
}
Vector2D pure_pursuit::pursuit(Vector2D now_pos)
{
    now_point = now_pos;

    switch (state)
    {
    case pp_standby: // 等待追踪，如果缓冲区中有超过两个以上的点则开始追踪
        target_wspeed.x = 0.0f;
        target_wspeed.y = 0.0f;
        if (points_buffer.queueSize() > 2)
        {
            points_buffer.dequeue(head);
            points_buffer.dequeue(tail); // 取出两个点作为第一次追踪的向量，注意取出顺序不要乱!!!
            state = pp_tracking;
        }
        else if (points_buffer.queueSize() == 2) // 只有两个点，可以直接停了
        {
            points_buffer.dequeue(head);
            points_buffer.dequeue(tail);
            state = pp_stoping;
        }

        break;

    case pp_tracking:

        target_line_initpoint = tail;
        target_line = head - tail;

        compute_error();

        // 使用用户选择的法向控制方式
        if (nor_mode == normalcontrol)
        {
            purepusit_normal_control();
        }
        else
        {
        }

        if (tangent_dis < change_point) // 要切换点了？
        {
            if (points_buffer.queueSize() > 1) // 还有超过一个点，可以继续追踪
            {
                head = tail;
                points_buffer.dequeue(tail);
            }
            else if (points_buffer.queueSize() <= 1) // 缓冲队列点数不够，准备停止追踪
            {
                state = pp_stoping;
            }
        }

        break;

    case pp_stoping:

        target_wspeed.x = 0.0f;
        target_wspeed.y = 0.0f;

        points_buffer.clear();

        state = pp_standby;

        break;

    case pp_restart: // 传入的首个待追踪向量的方向错了，先点追踪到起始点再开始纯追踪

        break;

    default:
        break;
    }

    return target_wspeed;
}

pure_pursuit::pure_pursuit(pure_pursuit_mode mode, float kp_, float ki_, float kd_, float ktp, float kti, float ktd, float r_) : normal_control(kp_, kd_, ki_, 100.0f, MAX_NORSPEED, 9.6f, 56.0f), dir_control(ktp, kti, ktd, 100.0f, MAX_SPEED, 9.6f, 56.0f)
{
}

bool pure_pursuit::pp_add_point(Vector2D new_point)
{
    return points_buffer.enqueue(new_point);
}
void pure_pursuit::pp_force_add_point(Vector2D new_point) // 添加新点，覆盖式
{
    points_buffer.forceEnqueue(new_point);
}
bool pure_pursuit::pp_add_points(Vector2D new_points[], uint8_t length)
{
    return points_buffer.enqueueArray(new_points, length);
}
void pure_pursuit::pp_force_add_points(Vector2D new_points[], uint8_t length)
{
    points_buffer.forceEnqueueArray(new_points, length);
}
void pure_pursuit::pp_refresh_points()
{
    points_buffer.clear();
}

Vector2D pointrack::track(Vector2D now_pos_, Vector2D target_point_)
{
    // 计算误差
    Vector2D now_to_target = target_point_ - now_pos_;
    taerget_dis = now_to_target.magnitude();
    float speed_sum = track_pid.PID_ComputeError(taerget_dis);

    return speed_sum * now_to_target.normalize();
}
float pointrack::get_dis()
{
    return taerget_dis;
}

pointrack::pointrack(float kp_, float ki_, float kd_, float deadzone_, float max_speed_) : track_pid(kp_, kd_, ki_, 0.0f, max_speed_, deadzone_, 0.0f)
{
}