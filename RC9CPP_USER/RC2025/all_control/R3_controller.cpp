#include "R3_controller.h"

const uint8_t R3Controller::bitWidths[5] = {1, 1, 1, 2, 2};

R3Controller::R3Controller() : mode_selector(bitWidths, 5)
{
    efsm_init();
    set_TDplanner_R(target_r);
    center_point.x = 3.63;
    center_point.y = -13.636f;
}

void R3Controller::update_flag()
{
    trigger_on_ = crsf_port->trigger_on;
    sal_flag_ = crsf_port->sal_flag;
    sar_flag_ = crsf_port->sar_flag;
    l_flag_ = crsf_port->l_flag;
    r_flag_ = crsf_port->r_flag;
}

void R3Controller::add_elrs(CrsfReceiver *crsf_port_)
{
    crsf_port = crsf_port_;
}

void R3Controller::add_yunball_and_shooter(R3Shooter *auto_shooter_ptr_, AutoYunballR3 *auto_yunball_ptr_)
{
    auto_shooter = auto_shooter_ptr_;
    auto_yunball_ptr = auto_yunball_ptr_;
}

void R3Controller::add_position_and_ros(imu *position_imu_ptr, imu *ros_imu_ptr)
{
    position_imu_ = position_imu_ptr;
    ros_imu = ros_imu_ptr;
}

void R3Controller::efsm_init()
{
    uint16_t attack_move_modeflag[] = {0, 4, 2, 6, 32, 34, 64, 66};

    uint16_t auto_reload_ballmodeflag[] = {3};

    uint16_t all_auto_yunballmodeflag[] = {1};

    uint16_t auto_yunball_and_loadballmodeflag[] = {7};

    uint16_t lock_on_center_pointmodeflag[] = {38};

    uint16_t auto_shoot_2_center_pointmodeflag[] = {33, 37};

    uint16_t lock_on_r2modeflag[] = {68, 70};
    uint16_t shoot_2_center_pointmodeflag[] = {35, 39};
    uint16_t shoot_2_r2modeflag[] = {65, 69, 67, 71};

    // uint16_t auto_shoot_2_r2modeflag[] = {67, 71};

    uint16_t wait_mode_movemodeflag[] = {16, 20, 18, 22

    };

    uint16_t reset_imumodeflag[] = {19};

    uint16_t reset_sw_motormodeflag[] = {17};

    uint16_t reset_yunball_shootermodeflag[] = {21};

    uint16_t set_center_pointmodeflag[] = {49};

    uint16_t hand_set_clawposmodeflag[] = {8, 12, 10, 14};

    // uint16_t hand_set_liftermodeflag[] = {10, 14};

    uint16_t lock_by_cam_modeflag[] = {36};

    uint16_t hand_shoot_modeflag[] = {41};

    uint16_t add_locate_pointmodeflag[] = {53};

    uint16_t calc_center_pointmodeflag[] = {55};

    uint16_t set_ladar_shoot_offsetmodeflag[] = {83};

    uint16_t set_cam_shoot_offsetmodeflag[] = {81};

    uint16_t set_cam_x_offsetmodeflag[] = {85};

    uint16_t cam_setmodeflag[] = {80, 84};

    uint16_t ladar_setmodeflag[] = {82};

    mode_selector.mapStateToIndices(0, attack_move_modeflag, 8);
    mode_selector.mapStateToIndices(1, auto_reload_ballmodeflag, 1);
    mode_selector.mapStateToIndices(2, all_auto_yunballmodeflag, 1);
    mode_selector.mapStateToIndices(3, lock_on_center_pointmodeflag, 2);
    mode_selector.mapStateToIndices(4, lock_on_r2modeflag, 2);
    mode_selector.mapStateToIndices(5, shoot_2_center_pointmodeflag, 2);
    mode_selector.mapStateToIndices(6, shoot_2_r2modeflag, 4);
    mode_selector.mapStateToIndices(7, auto_yunball_and_loadballmodeflag, 1);
    mode_selector.mapStateToIndices(8, wait_mode_movemodeflag, 4);
    mode_selector.mapStateToIndices(9, reset_imumodeflag, 1);
    mode_selector.mapStateToIndices(10, reset_sw_motormodeflag, 1);
    // mode_selector.mapStateToIndices(11, hand_set_liftermodeflag, 2);
    mode_selector.mapStateToIndices(12, hand_set_clawposmodeflag, 4);
    mode_selector.mapStateToIndices(13, reset_yunball_shootermodeflag, 1);
    mode_selector.mapStateToIndices(14, set_center_pointmodeflag, 1);
    mode_selector.mapStateToIndices(15, auto_shoot_2_center_pointmodeflag, 2);
    // mode_selector.mapStateToIndices(16, auto_shoot_2_r2modeflag, 2);
    mode_selector.mapStateToIndices(17, lock_by_cam_modeflag, 1);
    mode_selector.mapStateToIndices(18, hand_shoot_modeflag, 1);
    mode_selector.mapStateToIndices(19, add_locate_pointmodeflag, 1);
    mode_selector.mapStateToIndices(20, calc_center_pointmodeflag, 1);
    mode_selector.mapStateToIndices(21, set_ladar_shoot_offsetmodeflag, 1);
    mode_selector.mapStateToIndices(22, set_cam_shoot_offsetmodeflag, 1);
    mode_selector.mapStateToIndices(23, set_cam_x_offsetmodeflag, 1);
    mode_selector.mapStateToIndices(24, cam_setmodeflag, 2);
    mode_selector.mapStateToIndices(25, ladar_setmodeflag, 1);
}
float R3Controller::round_to_one_decimal(float number)
{
    return round(number * 10.0f) / 10.0f;
}
void R3Controller::process_data()
{

    HAL_UART_Receive_IT(crsf_port->huart_, crsf_port->rxBuffer_, RX_BUFFER_SIZE);
    update_flag();

    calc_data();
    set_accle();
    uint8_t flagValues[5] = {trigger_on_, sal_flag_, sar_flag_, l_flag_, r_flag_};
    currentStateflag = mode_selector.getState(flagValues);
    // debug_dis = round_to_one_decimal(crsf_port->right_V_map) * max_delta_rpm + 1000.0f;
    // send_datas.debug_dis = debug_dis;

    send_2_r2();
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
        auto_yunball_and_loadball();
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
        hand_set_lifter();
        break;
    case 12:
        hand_set_clawpos();
        break;
    case 13:
        reset_yunball_shooter();
        break;
    case 14:
        set_center_point();
        break;
    case 15:
        auto_shoot_2_center_point();
        break;

    case 17:
        lock_by_cam();
        break;
    case 18:
        hand_shoot();
        break;
    case 19:
        add_locate_point();
        break;
    case 20:
        calc_center_point();
        break;
    case 21:
        set_ladar_shoot_offset();
        break;
    case 22:
        set_cam_shoot_offset();
        break;
    case 23:
        set_cam_x_offset();
        break;

    case 24:
        cam_set();
        break;
    case 25:
        ladar_set();
        break;

    default:
        send_datas.status_flag = 40;
        break;
    }
    check_ef();
    send_crsf_datas();
}

void R3Controller::check_ef()
{
    if ((!check_error->u8_off_line) && (!check_error->m3508_off_line))
    {
        chassis_ef = 0; // 底盘正常
    }
    if (check_error->u8_off_line && (!check_error->m3508_off_line))
    {
        chassis_ef = 1;
    }
    if (check_error->m3508_off_line && (!check_error->u8_off_line))
    {
        chassis_ef = 2;
    }
    if (check_error->u8_off_line && check_error->m3508_off_line)
    {
        chassis_ef = 3;
    }

    if ((!check_error->shooter_motor_offline))
    {
        shooter_ef = 0;
    }
    if (check_error->shooter_motor_offline)
    {
        shooter_ef = 1;
    }

    if ((!check_error->yunball_motor_offline))
    {
        yunball_ef = 0;
    }
    if (check_error->yunball_motor_offline)
    {
        yunball_ef = 1;
    }

    if ((!check_error->position_offline) && (!check_error->ladar_offline))
    {
        pos_ef = 0;
    }
    if (check_error->position_offline && (!check_error->ladar_offline))
    {
        pos_ef = 1;
    }
    if ((!check_error->position_offline) && (check_error->ladar_offline))
    {
        pos_ef = 2;
    }
    if ((check_error->position_offline) && (check_error->ladar_offline))
    {
        pos_ef = 3;
    }
}

void R3Controller::send_crsf_datas()
{
    // send_datas.status_flag = currentStateflag;
    send_datas.position_yaw_rad = position_imu_->get_yaw_rad();
    send_datas.mid360_yaw_rad = ros_imu->get_yaw_rad();
    send_datas.position_x = position_imu_->get_world_pos_x();
    send_datas.position_y = position_imu_->get_world_pos_y();

    if ((send_datas.last_status_flag != send_datas.status_flag) && (!send_datas.toggle_lock))
    {
        send_datas.toggle_lock = true;
        send_datas.has_toggle = true;
        send_datas.real_status_flag = send_datas.status_flag;
    }

    if (send_datas.toggle_lock)
    {
        send_datas.status_cnt++;
        if (send_datas.status_cnt > 10)
        {
            send_datas.toggle_lock = false;
            send_datas.has_toggle = false;
            send_datas.status_cnt = 0;
        }
    }
    if (!send_datas.has_toggle)
    {
        send_datas.real_status_flag = send_datas.status_flag;
    }

    if (send_cnt == 0)
    {
        crsf_port->sendAttitude(send_datas.position_yaw_rad, send_datas.mid360_yaw_rad, target_accle * 0.5f);
    }

    else if (send_cnt == 1)
    {
        crsf_port->sendBattery(chassis_ef, shooter_ef, yunball_ef, pos_ef); // 1=10,1=10
    }
    else if (send_cnt == 2)
    {

        int integer_part = (int)send_datas.dis_2_target;
        float fractional_part = send_datas.dis_2_target - integer_part;
        crsf_port->sendGps(send_datas.position_x, send_datas.position_y, fractional_part * 10000.0f, integer_part * 100, send_datas.debug_dis, send_datas.real_status_flag);
    }
    send_cnt++;

    if (send_cnt >= 3)
    {
        send_cnt = 0;
    }

    send_datas.last_status_flag = send_datas.status_flag;
}

void R3Controller::set_accle()
{
    target_accle = 2.0f + max_delta_acc * crsf_port->roll_map;
}

void R3Controller::remote_move()
{

    camera_ops->camera_off();
    Vector2D tvel_(crsf_port->left_V_mapcurve * max_x_speed, crsf_port->left_H_mapcurve * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);

    set_RobotW(-crsf_port->right_H_mapcurve * max_yaw_speed, 0);
}

void R3Controller::remote_move_revert()
{
    Vector2D tvel_(-crsf_port->left_H_mapcurve * max_x_speed, crsf_port->left_V_mapcurve * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);
    set_RobotW(-crsf_port->right_H_mapcurve * max_yaw_speed, 0);
}

void R3Controller::remote_move_robot()
{

    Vector2D tvel_(-crsf_port->left_H_mapcurve * max_x_speed, crsf_port->left_V_mapcurve * max_y_speed);
    set_RobotVel(tvel_, 0);
    set_RobotW(-crsf_port->right_H_mapcurve * max_yaw_speed, 0);
}

void R3Controller::all_stop()
{
    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);
    set_RobotW(0.0f, 0);
}

void R3Controller::calc_data()
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
    dis_2_robot = dis.magnitude();
    nor_dir = dis.normalize();
    heading_2_robot = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;
}

void R3Controller::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    robot_point.x = -floatData[0] + 0.0f;
    robot_point.y = -floatData[1] + -6.7888f;
    r2_ec.check_tick();
}

void R3Controller::send_2_r2()
{
    float send_data[6] = {position_imu_->get_heading(), -(position_imu_->get_world_pos_y() - (-6.7888f)), -(position_imu_->get_world_pos_x() - 0.0f), position_imu_->get_yaw_speed(), 0.0f, 0.0f};

    sendFloatData(1, send_data, 6);
}

void R3Controller::all_auto_yunball()
{
    send_datas.status_flag = 2;

    auto_yunball_ptr->start_yunball();

    crsf_port->reset_trigger_flag(); // 动作执行完后重置扳机flag
}

void R3Controller::auto_reload_ball()
{

    send_datas.status_flag = 1;

    auto_yunball_ptr->start_putball();

    crsf_port->reset_trigger_flag(); // 动作执行完后重置扳机flag
}

void R3Controller::lock_on_center_point()
{
    if (!auto_yunball_ptr->if_enable_shoot())
    {
        auto_yunball_ptr->in_or_out(true);
    }
    camera_ops->camera_off();
    send_datas.status_flag = 3;
    yaw_TurnTo(heading_2_center, 0);
    Vector2D tvel_(crsf_port->left_V_mapcurve * max_x_speed, crsf_port->left_H_mapcurve * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);
    send_datas.dis_2_target = dis_2_center;
}

void R3Controller::lock_by_cam()
{
    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);
    if (!auto_yunball_ptr->if_enable_shoot())
    {
        auto_yunball_ptr->in_or_out(true);
    }
    camera_ops->camera_on();
    if (camera_ops->check())
    {
        if (camera_ops->camera_ready == true)
        {
            send_datas.status_flag = 30;

            set_RobotW(0.0f, 0);
        }
        else
        {
            send_datas.status_flag = 17;
        }
    }
    else
    {
        send_datas.status_flag = 31;
    }
}

void R3Controller::lock_on_r2()
{
    if (!auto_yunball_ptr->if_enable_shoot())
    {
        auto_yunball_ptr->in_or_out(true);
    }
   


    if(r2_ec.get_ec_code()==0)
    {
        yaw_TurnTo(heading_2_robot, 0);
        
        send_datas.dis_2_target = dis_2_robot;
        send_datas.status_flag = 4;
    }

    Vector2D tvel_(crsf_port->left_V_mapcurve * max_x_speed, crsf_port->left_H_mapcurve * max_y_speed);
    set_worldVel_accle(tvel_, target_accle);
}

void R3Controller::shoot_2_center_point()
{
    send_datas.status_flag = 5;
    if (!auto_yunball_ptr->if_enable_shoot())
    {
        auto_yunball_ptr->in_or_out(true);
    }
    if (auto_yunball_ptr->if_is_finish() && auto_yunball_ptr->if_enable_shoot())
    {
        all_stop();
        if (auto_shooter->set_auto_byFitter(Auto, dis_2_center) == Lift)
        {
            crsf_port->reset_trigger_flag();
        }
    }
}

void R3Controller::shoot_2_r2()
{
    send_datas.status_flag = 6;
    if (!auto_yunball_ptr->if_enable_shoot())
    {
        auto_yunball_ptr->in_or_out(true);
    }

    if (auto_yunball_ptr->if_is_finish() && auto_yunball_ptr->if_enable_shoot())
    {
        all_stop();
        if (auto_shooter->set_autoPass_byFitter(Auto, dis_2_robot) == Lift)
        {
            crsf_port->reset_trigger_flag();
        }
    }
}

void R3Controller::attack_move_mode()
{
    send_datas.status_flag = 0;

    remote_move();
}

void R3Controller::wait_mode_move()
{
    send_datas.status_flag = 8;
    remote_move_robot();
}

void R3Controller::reset_sw_motor()
{
    send_datas.status_flag = 10;
    reset_swerve();
    crsf_port->reset_trigger_flag();
}

void R3Controller::reset_all_imu()
{
    send_datas.status_flag = 9;
    ros_imu->imu_rst();
    position_imu_->imu_rst();
    crsf_port->reset_trigger_flag();
}

void R3Controller::hand_set_clawpos()
{

    send_datas.status_flag = 12;
    // auto_yunball_ptr->control_put_motor(crsf_port->right_H_map);
    //  remote_move();
    remote_move();

    if (sar_flag_ == 1)
    {
        auto_yunball_ptr->control_claw(true);
    }
    else if (sar_flag_ == 0)
    {
        auto_yunball_ptr->control_claw(false);
    }

    if (sal_flag_ == 0)
    {
        auto_yunball_ptr->init_yunball();
    }
    else
    {
        auto_yunball_ptr->in_or_out(false);
    }
}

void R3Controller::auto_yunball_and_loadball()
{
    send_datas.status_flag = 7;
    auto_yunball_ptr->yun_and_put();
    crsf_port->reset_trigger_flag();
}

void R3Controller::add_camera(CameraOperation *camera_ops_ptr)
{
    camera_ops = camera_ops_ptr;
}

void R3Controller::auto_shoot_2_center_point()
{
    send_datas.status_flag = 15;
    if (auto_yunball_ptr->if_enable_shoot())
    {
        if (auto_shooter->set_auto_byCameraFitter(Auto, camera_ops->camera_Y / 100.0f) == Lift)
        {

            crsf_port->reset_trigger_flag();
            camera_ops->camera_off();
        }
    }
    else
    {
        auto_yunball_ptr->in_or_out(true);
    }
}

void R3Controller::auto_shoot_2_r2()
{
    crsf_port->reset_trigger_flag();
}

void R3Controller::reset_yunball_shooter()
{
    send_datas.status_flag = 13;
    auto_yunball_ptr->reload_motor();
    crsf_port->reset_trigger_flag();
}

void R3Controller::set_center_point()
{
    send_datas.status_flag = 14;
    center_point.x = get_world_x();
    center_point.y = get_world_y();
    crsf_port->reset_trigger_flag();
}

void R3Controller::hand_set_lifter()
{
    send_datas.status_flag = 11;
    auto_shooter->set_hand(crsf_port->right_H_map * 500.0f);
}

void R3Controller::hand_shoot()
{
    crsf_port->reset_trigger_flag();
}

void R3Controller::add_locate_point()
{
    send_datas.status_flag = 19;
    basketCalibrator.addMeasurement(-get_world_x(), get_world_y(), position_imu_->get_heading());
    crsf_port->reset_trigger_flag();
}

void R3Controller::calc_center_point()
{
    if (basketCalibrator.isReadyToCalculate())
    {
        Vector2D center = basketCalibrator.calculateCenter();
        if (center.x != -50.0f && center.y != -50.0f)
        {
            send_datas.status_flag = 20;
            center_point.x = -center.x;
            center_point.y = center.y;
        }
        basketCalibrator.reset();
    }
    crsf_port->reset_trigger_flag();
}

void R3Controller::cam_set()
{
    send_datas.status_flag = 24;
    send_datas.debug_dis = abs(-crsf_port->right_H_map * 20.0f);
}

void R3Controller::ladar_set()
{
    send_datas.status_flag = 25;
    send_datas.debug_dis = abs(-crsf_port->right_H_map * 100.0f);
}

void R3Controller::set_ladar_shoot_offset()
{
    send_datas.status_flag = 21;
    auto_shooter->offset = -crsf_port->right_H_map * 100.0f;
    crsf_port->reset_trigger_flag();
}

void R3Controller::set_cam_shoot_offset()
{
    send_datas.status_flag = 22;
    auto_shooter->camera_offset = -crsf_port->right_H_map * 100.0f;
    crsf_port->reset_trigger_flag();
}

void R3Controller::set_cam_x_offset()
{
    send_datas.status_flag = 23;
    camera_ops->offest_x = -crsf_port->right_H_map * 20.0f;
    crsf_port->reset_trigger_flag();
}
