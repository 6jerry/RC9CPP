#include "vesc.h"

float vesc::get_rpm()
{
<<<<<<< HEAD
    return now_rpm;
=======
    return now_rpm / gear_ratio;
>>>>>>> 40b0e7df49798d9cb74baf2e9eea4ee4dc4618a2
}

void vesc::set_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm * gear_ratio;
<<<<<<< HEAD
    target_erpm = (int32_t)(target_rpm * motor_polse);
=======
>>>>>>> 40b0e7df49798d9cb74baf2e9eea4ee4dc4618a2
}

void vesc::can_update(uint8_t can_RxData[8])
{
    int16_t current = (int16_t)((can_RxData[4] << 8) | can_RxData[5]);

    int32_t erpm = (int32_t)((can_RxData[0] << 24) | (can_RxData[1] << 16) | (can_RxData[2] << 8) | can_RxData[3]);

    rcurrent = (float)current * 0.1f; // A
    now_rpm = (float)erpm / (float)motor_polse;
}

void vesc::process_data()
{
<<<<<<< HEAD

    if (target_erpm != 0)
    {
        if (if_invert)
        {
            senderpm = -target_erpm;
        }
        else
        {
            senderpm = target_erpm;
        }

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

vesc::vesc(uint8_t can_id_, CAN_HandleTypeDef *hcan_, bool invert, uint8_t motor_polse_, float gear_ratio_, float kp_, float ki_, float kd_, float r_) : CanDevice(VESC, hcan_, can_id_), rpm_control(kp_, ki_, kd_, 25000.0f, 1000.0f, 20.0f, 400.0f), motor_polse(motor_polse_), gear_ratio(gear_ratio_), if_invert(invert)
{
    // extid = (CAN_CMD_SET_ERPM << 8) | can_id;
=======
    rpm_control.increPID_setarget(target_rpm);
    //target_current = (int32_t)rpm_control.increPID_Compute(now_rpm);
    uint8_t vesc_tx_buf[8] = {0};

    vesc_tx_buf[0] = (target_current >> 24) & 0xFF;
    vesc_tx_buf[1] = (target_current >> 16) & 0xFF;
    vesc_tx_buf[2] = (target_current >> 8) & 0xFF;
    vesc_tx_buf[3] = target_current & 0xFF;

    CAN_Send(extid, true, vesc_tx_buf);
}

vesc::vesc(uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t motor_polse_, float gear_ratio_, float kp_, float ki_, float kd_, float r_) : CanDevice(VESC, hcan_, can_id), rpm_control(kp_, ki_, kd_, r_, 25000.0f, 5.0f), motor_polse(motor_polse_), gear_ratio(gear_ratio_)
{
    extid = (CAN_CMD_SET_CURRENT << 8) | can_id;
>>>>>>> 40b0e7df49798d9cb74baf2e9eea4ee4dc4618a2
}