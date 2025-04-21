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

    tangent_dir = -projected.normalize(); // 切向单位向量，指向未追踪的点

    normal_dir = normal_vector.normalize(); // 法向单位向量，指向目标向量
}

void pure_pursuit::purepusit_normal_control()
{
    float nor_speed = normal_control.PID_ComputeError(normal_dis); // 法向纠偏速度的大小

    tan_speed = tangent_control.PID_ComputeError(getRemainingDistance()); // 切向速度的大小

    // tan_speed = 0.2f;
    Vector2D normal_speed = nor_speed * normal_dir;   // 法向速度矢量
    Vector2D tangent_speed = tan_speed * tangent_dir; // 切向速度矢量

    target_wspeed = normal_speed + tangent_speed; // 最终输出的期望速度合矢量
}
/** 
 * @brief 梯形规划控制方式
 * 
*/
void pure_pursuit::purepusit_dir_control()
{
    if (trapezoidal_planner.isFinished())
    {
        trapezoidal_planner.start_plan(1.0f,1.0f,3.0f,now_vel.magnitude(),0.0f,tangent_dis,0.0f);
    }

    float nor_speed = normal_control.PID_ComputeError(normal_dis); // 法向纠偏速度的大小

    tan_speed = -trapezoidal_planner.plan(tangent_dis); // 梯形规划输出切向速度的大小

    //tan_speed = 0.2f;
    Vector2D normal_speed = nor_speed * normal_dir;   // 法向速度矢量
    Vector2D tangent_speed = tan_speed * tangent_dir; // 切向速度矢量

    target_wspeed = normal_speed + tangent_speed; // 最终输出的期望速度合矢量
}
Vector2D pure_pursuit::pursuit(Vector2D now_pos)
{
    now_point = now_pos;

    switch (state)
    {
    case pp_standby: // 等待追踪，如果缓冲区中有超过两个以上的点则开始追踪
        target_wspeed.x = 0.0f;
        target_wspeed.y = 0.0f;
        trapezoidal_planner.reset();  // 重置梯形规划
        /*
        if (points_buffer.queueSize() >= 2)
        {
            points_buffer.dequeue(head);
            points_buffer.dequeue(tail); // 取出两个点作为第一次追踪的向量，注意取出顺序不要乱!!!
            state = pp_tracking;
        }
            */

        break;

    case pp_tracking:

        compute_error();

        // 使用用户选择的法向控制方式
        if (nor_mode == normalcontrol)
        {
            purepusit_normal_control();
        }
        else// 使用轨迹规划控制方式
        {
            purepusit_dir_control();
        }

        /*

        if (tangent_dis < change_point) // 要切换点了？
        {
            if (points_buffer.queueSize() >= 1) // 还有超过一个点，可以继续追踪
            {
                head = tail;
                points_buffer.dequeue(tail);
            }
        }
        */

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

float pure_pursuit::getRemainingDistance()
{
    float remaining = 0.0f;
    remaining += points_buffer.totalDistance() + tangent_dis;

    if (points_buffer.queueSize() >= 1)
    {
        remaining += (tail - points_buffer.peek()).magnitude();
    }

    return remaining;
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
    state = pp_standby;
}

void pure_pursuit::pp_start_plan(Vector2D start_point, Vector2D end_point, Vector2D now_robvel)
{

    if (state == pp_standby)
    {
        head = start_point;
        tail = end_point;
        target_line_initpoint = tail;
        target_line = head - tail;
        now_vel = now_robvel;
        state = pp_tracking;
        nor_mode = dircontrol;  //选择轨迹规划控制方式
    }
}

void pure_pursuit::pp_rst_plan()
{
    state = pp_standby;
    trapezoidal_planner.reset();  // 重置梯形规划
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

void yaw_adjuster::calc_angle_error()
{
    if (real_angle * target_angle >= 0)
    {
        angle_error = target_angle - real_angle;
    }
    else
    {
        if (real_angle > 0 && target_angle < 0)
        {
            float positive = 180.0f - real_angle + 180.0f + target_angle; // 正路径
            float negative = target_angle - real_angle;
            if (abs(positive) <= abs(negative))
            {
                angle_error = positive;
            }
            else
            {
                angle_error = negative; // 选择一个较短的路径
            }
        }
        else if (real_angle < 0 && target_angle > 0)
        {
            float positive = target_angle - real_angle;
            float negative = -(360.0f + real_angle - target_angle);
            if (abs(positive) <= abs(negative))
            {
                angle_error = positive;
            }
            else
            {
                angle_error = negative; // 选择一个较短的路径
            }
        }
    }
}

float yaw_adjuster::yaw_adjust(float now_angle, float target_angle_)
{
    real_angle = now_angle;
    target_angle = target_angle_;
    calc_angle_error();
    return yaw_pid.PID_ComputeError(angle_error);
}
