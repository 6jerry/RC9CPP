#include "all_controller.h"
const uint8_t AllController::bitWidths[5] = {1, 1, 1, 2, 2};

AllController::AllController() : mode_selector(bitWidths, 5)
{
    efsm_init();
}

void AllController::update_flag()
{
    trigger_on_ = crsf_port->trigger_on;
    sal_flag_ = crsf_port->sal_flag;
    sar_flag_ = crsf_port->sar_flag;
    l_flag_ = crsf_port->l_flag;
    r_flag_ = crsf_port->r_flag;
}

void AllController::add_elrs(CrsfReceiver *crsf_port_)
{
    crsf_port = crsf_port_;
}

void AllController::add_yunball_and_shooter(AutoShooter *auto_shooter_ptr_, auto_yunball *auto_yunball_ptr_)
{
    auto_shooter = auto_shooter_ptr_;
    auto_yunball_ptr = auto_yunball_ptr_;
}

void AllController::efsm_init()
{
    uint16_t attack_move_modeflag[] = {0, 4, 2, 6, 8, 12};

    uint16_t auto_reload_ballmodeflag[] = {5, 7};

    uint16_t all_auto_yunballmodeflag[] = {1, 3};

    uint16_t lock_on_center_pointmodeflag[] = {10};
    uint16_t lock_on_r2modeflag[] = {14};
    uint16_t shoot_2_center_pointmodeflag[] = {9};
    uint16_t shoot_2_r2modeflag[] = {13};

    uint16_t defend_move_modeflag[] = {16};

    mode_selector.mapStateToIndices(0, attack_move_modeflag, 6);
    mode_selector.mapStateToIndices(1, auto_reload_ballmodeflag, 2);
    mode_selector.mapStateToIndices(2, all_auto_yunballmodeflag, 2);
    mode_selector.mapStateToIndices(3, lock_on_center_pointmodeflag, 1);
    mode_selector.mapStateToIndices(4, lock_on_r2modeflag, 1);
    mode_selector.mapStateToIndices(5, shoot_2_center_pointmodeflag, 1);
    mode_selector.mapStateToIndices(6, shoot_2_r2modeflag, 1);
    mode_selector.mapStateToIndices(7, defend_move_modeflag, 1);
}

void AllController::process_data()
{

    update_flag();

    calc_data();
    remote_move();
    uint8_t flagValues[5] = {trigger_on_, sal_flag_, sar_flag_, l_flag_, r_flag_};
    currentStateflag = mode_selector.getState(flagValues);
    switch (currentStateflag)
    {
    case 0:
        attack_move_mode();
        break;
    case 1:
        auto_reload_ball();
        break;
    case 2:
        all_auto_yunball();
        break;
    case 3:
        lock_on_center_point();
        break;
    case 4:
        lock_on_r2();
        break;
    case 5:
        shoot_2_center_point();
        break;
    case 6:
        shoot_2_r2();
        break;
    case 7:
        defend_move_mode();
        break;

    default:
        break;
    }

    // sendAttitude(233.0f, 233.3f, 233.0f);

    // sendBattery(1212.0f, 20.0f, 12.0f, 233);
}

void AllController::remote_move()
{
    Vector2D tvel_(crsf_port->right_H_map * max_x_speed, crsf_port->right_V_map * max_y_speed);
    set_WorldVel(tvel_, 0);
    set_RobotW(crsf_port->left_H_map * max_yaw_speed, 0);
}

void AllController::remote_move_revert()
{
    Vector2D tvel_(-crsf_port->right_H_map * max_x_speed, -crsf_port->right_V_map * max_y_speed);
    set_WorldVel(tvel_, 0);
    set_RobotW(-crsf_port->left_H_map * max_yaw_speed, 0);
}

void AllController::remote_move_robot()
{
    Vector2D tvel_(crsf_port->right_H_map * max_x_speed, crsf_port->right_V_map * max_y_speed);
    set_RobotVel(tvel_, 0);
    set_RobotW(crsf_port->left_H_map * max_yaw_speed, 0);
}

void AllController::calc_data()
{
    Vector2D now_point;
    now_point.x = get_world_x();
    now_point.y = get_world_y();

    Vector2D dis = {0, 0};

    dis = center_point - now_point;
    dis_2_center = dis.magnitude();
    nor_dir = dis.normalize();
    heading_2_center = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;

    dis = robot_point - now_point;
    dis_2_center = dis.magnitude();
    nor_dir = dis.normalize();
    heading_2_robot = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;
}

void AllController::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
}

void AllController::all_auto_yunball()
{
    if (auto_yunball_ptr->start_yunball())
    {
        crsf_port->reset_trigger_flag(); // 动作执行完后重置扳机flag
    }

    // 执行期间可以自由移动
    remote_move();
}

void AllController::auto_reload_ball()
{
    if (auto_yunball_ptr->start_putball())
    {
        crsf_port->reset_trigger_flag(); // 动作执行完后重置扳机flag
    }
    // 执行期间可以自由移动

    remote_move();
}

void AllController::lock_on_center_point()
{
}
void AllController::lock_on_r2()
{
}

void AllController::shoot_2_center_point()
{
}

void AllController::shoot_2_r2()
{
}

void AllController::attack_move_mode()
{

    remote_move();
}

void AllController::defend_move_mode()
{
    remote_move_revert();
}

void AllController::wait_mode_move()
{
    remote_move_robot();
}