#include "vesc.h"

float vesc::get_rpm()
{
    return now_rpm;
}

void vesc::set_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm * gear_ratio;
    target_erpm = (int32_t)(target_rpm * motor_polse);

    test_rpm = (float)target_erpm;
    vesc_mode = vesc_erpm;
}

void vesc::send_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm * gear_ratio;
    target_erpm = (int32_t)(target_rpm * motor_polse);
    vesc_mode = vesc_erpm;
}

void vesc::set_current(float target_c_)
{
    target_current = target_c_;
    vesc_mode = vesc_current;
}

void vesc::can_update(uint8_t can_RxData[8])
{
    int16_t current = (int16_t)((can_RxData[4] << 8) | can_RxData[5]);

    int32_t erpm = (int32_t)((can_RxData[0] << 24) | (can_RxData[1] << 16) | (can_RxData[2] << 8) | can_RxData[3]);

    rcurrent = (float)current * 0.1f; // A
    now_rpm = (float)erpm / (float)motor_polse;

    filted_rpm = rpm_filter.update(now_rpm);
}

void vesc::process_data()
{

    switch (vesc_mode)
    {
    case vesc_current:
        current_mode();

        break;
    case vesc_erpm:
        erpm_mode();
        break;

    case vesc_rpm_increpid:
        rpm_increpid_mode();
        break;
    }
}

void vesc::current_mode()
{
    extid = (CAN_CMD_SET_CURRENT << 8) | can_id;
    uint8_t vesc_tx_buf[8] = {0};

    send_current = (int32_t)(target_current);

    vesc_tx_buf[0] = (send_current >> 24) & 0xFF;
    vesc_tx_buf[1] = (send_current >> 16) & 0xFF;
    vesc_tx_buf[2] = (send_current >> 8) & 0xFF;
    vesc_tx_buf[3] = send_current & 0xFF;
    CAN_Send(extid, true, vesc_tx_buf);
}

void vesc::erpm_mode()
{
    if (target_erpm != 0)
    {
        senderpm = target_erpm;

        extid = (CAN_CMD_SET_ERPM << 8) | can_id;

        uint8_t vesc_tx_buf[8] = {0};

        vesc_tx_buf[0] = (senderpm >> 24) & 0xFF;
        vesc_tx_buf[1] = (senderpm >> 16) & 0xFF;
        vesc_tx_buf[2] = (senderpm >> 8) & 0xFF;
        vesc_tx_buf[3] = senderpm & 0xFF;

        CAN_Send(extid, true, vesc_tx_buf);
    }
    else
    {
        extid = (CAN_CMD_SET_BRAKE << 8) | can_id;

        uint8_t vesc_tx_buf[8] = {0};
        brake = 6000;
        vesc_tx_buf[0] = (brake >> 24) & 0xFF;
        vesc_tx_buf[1] = (brake >> 16) & 0xFF;
        vesc_tx_buf[2] = (brake >> 8) & 0xFF;
        vesc_tx_buf[3] = brake & 0xFF;

        CAN_Send(extid, true, vesc_tx_buf);
    }
}

void vesc::rpm_increpid_mode()
{
    rpm_control.increPID_setarget(target_rpm);
    target_current = rpm_control.increPID_Compute(now_rpm) + target_ff_current;

    if (debug_mode)
    {

        float send_datas[3] = {target_rpm, now_rpm, filted_rpm};

        sendFloatData(1, send_datas, 3);
    }

    current_mode();
}

vesc::vesc(uint8_t can_id_, CAN_HandleTypeDef *hcan_, uint8_t motor_polse_, float gear_ratio_, float kp_, float ki_, float kd_, float r_) : CanDevice(VESC, hcan_, can_id_), motor_polse(motor_polse_), gear_ratio(gear_ratio_)
{
    // extid = (CAN_CMD_SET_ERPM << 8) | can_id;

    rpm_filter.setWindowSize(2);
}

void vesc::start_debug()
{
    debug_mode = true;
}

void vesc::set_ff_current(float target_c_)
{
    target_ff_current = target_c_;
}