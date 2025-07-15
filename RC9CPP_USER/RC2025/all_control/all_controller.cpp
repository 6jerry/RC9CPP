#include "all_controller.h"
const uint8_t AllController::bitWidths[5] = {1, 1, 1, 2, 2};

AllController::AllController() : mode_selector(bitWidths, 5)
{
    efsm_init();

    center_point.x = 3.000f;
    center_point.y = 14.355f;
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

void AllController::add_position_and_ros(imu *position_imu_ptr, imu *ros_imu_ptr)
{
    position_imu_ = position_imu_ptr;
    ros_imu = ros_imu_ptr;
}

void AllController::efsm_init()
{
    uint16_t attack_move_modeflag[] = {0, 4, 2, 6, 8, 12};

    uint16_t auto_reload_ballmodeflag[] = {5, 7};

    uint16_t all_auto_yunballmodeflag[] = {1, 3};

    uint16_t lock_on_center_pointmodeflag[] = {10, 74, 78};
    uint16_t lock_on_r2modeflag[] = {14};
    uint16_t shoot_2_center_pointmodeflag[] = {9, 11};
    uint16_t shoot_2_r2modeflag[] = {13, 15};

    uint16_t defend_move_modeflag[] = {16, 20, 18, 22, 17, 21, 19, 23};

    uint16_t wait_mode_movemodeflag[] = {
        32,
        36,
        34,
        38,
        72,
        76,
    };

    uint16_t reset_imumodeflag[] = {33, 35};

    uint16_t reset_sw_motormodeflag[] = {37, 39};

    uint16_t hand_shootmodeflag[] = {73, 77, 75, 79};

    uint16_t hand_set_clawposmodeflag[] = {64, 68, 66, 70};

    mode_selector.mapStateToIndices(0, attack_move_modeflag, 6);
    mode_selector.mapStateToIndices(1, auto_reload_ballmodeflag, 2);
    mode_selector.mapStateToIndices(2, all_auto_yunballmodeflag, 2);
    mode_selector.mapStateToIndices(3, lock_on_center_pointmodeflag, 3);
    mode_selector.mapStateToIndices(4, lock_on_r2modeflag, 1);
    mode_selector.mapStateToIndices(5, shoot_2_center_pointmodeflag, 2);
    mode_selector.mapStateToIndices(6, shoot_2_r2modeflag, 2);
    mode_selector.mapStateToIndices(7, defend_move_modeflag, 8);
    mode_selector.mapStateToIndices(8, wait_mode_movemodeflag, 6);
    mode_selector.mapStateToIndices(9, reset_imumodeflag, 2);
    mode_selector.mapStateToIndices(10, reset_sw_motormodeflag, 2);
    mode_selector.mapStateToIndices(11, hand_shootmodeflag, 4);
    mode_selector.mapStateToIndices(12, hand_set_clawposmodeflag, 4);
}

void AllController::process_data()
{

    HAL_UART_Receive_IT(crsf_port->huart_, crsf_port->rxBuffer_, RX_BUFFER_SIZE);
    update_flag();

    calc_data();
    set_accle();
    uint8_t flagValues[5] = {trigger_on_, sal_flag_, sar_flag_, l_flag_, r_flag_};
    currentStateflag = mode_selector.getState(flagValues);
    debug_dis = crsf_port->right_V_map * max_debug_dis;
    send_datas.debug_dis = debug_dis;
    // send_datas.status_flag = currentStateflag;
    send_crsf_datas();
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
    case 8:
        wait_mode_move();
        break;
    case 9:
        reset_all_imu();
        break;
    case 10:
        reset_sw_motor();
        break;
    case 11:
        hand_shoot();
        break;
    case 12:
        hand_set_clawpos();
        break;

    default:
        break;
    }
}

void AllController::send_crsf_datas()
{
    send_datas.position_yaw_rad = position_imu_->get_yaw_rad();
    send_datas.mid360_yaw_rad = ros_imu->get_yaw_rad();
    send_datas.position_x = position_imu_->get_world_pos_x();
    send_datas.position_y = position_imu_->get_world_pos_y();

    if (send_cnt == 0)
    {
        crsf_port->sendAttitude(send_datas.position_yaw_rad, send_datas.mid360_yaw_rad, target_accle * 0.5f);
    }

    else if (send_cnt == 1)
    {
        crsf_port->sendBattery(0.0f, 0.0f, 0, send_datas.error_flag);
    }
    else if (send_cnt == 2)
    {

        int integer_part = (int)send_datas.dis_2_target;
        float fractional_part = send_datas.dis_2_target - integer_part;
        crsf_port->sendGps(send_datas.position_x, send_datas.position_y, fractional_part * 10000.0f, integer_part * 100, send_datas.debug_dis * 1000.0f, send_datas.status_flag);
    }
    send_cnt++;

    if (send_cnt >= 3)
    {
        send_cnt = 0;
    }
}

void AllController::set_accle()
{
    target_accle = 2.0f + max_delta_acc * crsf_port->roll_map;
}

void AllController::remote_move()
{
    Vector2D tvel_(-crsf_port->left_H_map * max_x_speed, crsf_port->left_V_map * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);
    set_RobotW(-crsf_port->right_H_map * max_yaw_speed, 0);
}

void AllController::remote_move_revert()
{
    Vector2D tvel_(crsf_port->left_H_map * max_x_speed, -crsf_port->left_V_map * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);
    set_RobotW(-crsf_port->right_H_map * max_yaw_speed, 0);
}

void AllController::remote_move_robot()
{
    Vector2D tvel_(-crsf_port->left_H_map * max_x_speed, crsf_port->left_V_map * max_y_speed);
    set_RobotVel(tvel_, 0);
    set_RobotW(-crsf_port->right_H_map * max_yaw_speed, 0);
}

void AllController::all_stop()
{
    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);
    set_RobotW(0.0f, 0);
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

    heading_2_center += 180.0f;
    if (heading_2_center > 180.0f)
    {
        heading_2_center -= 360.0f;
    }

    dis = robot_point - now_point;
    dis_2_robot = dis.magnitude();
    nor_dir = dis.normalize();
    heading_2_robot = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;

    heading_2_robot += 180.0f;
    if (heading_2_robot > 180.0f)
    {
        heading_2_robot -= 360.0f;
    }
}

void AllController::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    robot_point.x = floatData[0] + pian_x;
    robot_point.y = floatData[1] + pian_y;
}

void AllController::all_auto_yunball()
{
    send_datas.status_flag = 2;
    auto_yunball_ptr->start_yunball();

    crsf_port->reset_trigger_flag(); // 动作执行完后重置扳机flag
}

void AllController::auto_reload_ball()
{
    send_datas.status_flag = 1;
    auto_yunball_ptr->start_putball();

    crsf_port->reset_trigger_flag(); // 动作执行完后重置扳机flag
}

void AllController::lock_on_center_point()
{
    send_datas.status_flag = 3;
    yaw_TurnTo(heading_2_center, 0);
    Vector2D tvel_(-crsf_port->left_H_map * max_x_speed, crsf_port->left_V_map * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);
    send_datas.dis_2_target = dis_2_center;
}
void AllController::lock_on_r2()
{
    send_datas.status_flag = 4;
    yaw_TurnTo(heading_2_robot, 0);
    Vector2D tvel_(-crsf_port->left_H_map * max_x_speed, crsf_port->left_V_map * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);
    send_datas.dis_2_target = dis_2_robot;
}

void AllController::shoot_2_center_point()
{
    send_datas.status_flag = 5;

    if (auto_yunball_ptr->if_is_finish())
    {
        all_stop();
        if (auto_shooter->set_auto_byFitter(PID, dis_2_center))
        {
            crsf_port->reset_trigger_flag();
        }
    }
}

void AllController::shoot_2_r2()
{
    send_datas.status_flag = 6;
    if (auto_yunball_ptr->if_is_finish())
    {
        all_stop();
        if (auto_shooter->set_auto_byFitter(PID, dis_2_robot))
        {
            crsf_port->reset_trigger_flag();
        }
    }
}

void AllController::attack_move_mode()
{
    send_datas.status_flag = 0;
    remote_move();
}

void AllController::defend_move_mode()
{
    send_datas.status_flag = 7;
    remote_move_revert();
}

void AllController::wait_mode_move()
{
    send_datas.status_flag = 8;
    remote_move_robot();
}

void AllController::reset_sw_motor()
{
    send_datas.status_flag = 10;
    reset_swerve();
    crsf_port->reset_trigger_flag();
}

void AllController::reset_all_imu()
{
    send_datas.status_flag = 9;
    ros_imu->imu_rst();
    position_imu_->imu_relocate(0.0f, 0.0f, 0.0f);
    crsf_port->reset_trigger_flag();
}

void AllController::hand_shoot()
{
    send_datas.status_flag = 11;

    all_stop();
    if (auto_shooter->set_auto_byDis(PID, debug_dis) == auto_shoot)
    {
        crsf_port->reset_trigger_flag();
    }
}

void AllController::hand_set_clawpos()
{
    send_datas.status_flag = 12;

    auto_yunball_ptr->control_turn_motor(crsf_port->right_H_map);
    Vector2D tvel_(-crsf_port->left_H_map * max_x_speed, crsf_port->left_V_map * max_y_speed);
    set_RobotVel(tvel_, 0);

    if (sar_flag_ == 0)
    {
        auto_yunball_ptr->control_claw(true);
    }
    else if (sar_flag_ == 1)
    {
        auto_yunball_ptr->control_claw(false);
    }
}
