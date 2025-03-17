#include "RC9Protocol.hpp"

void RC9Protocol::send_data()
{

    sendBuffer_[0] = FRAME_HEAD_0_RC9;
    sendBuffer_[1] = FRAME_HEAD_1_RC9;
    sendBuffer_[2] = tx_frame_mat.frame_id;
    sendBuffer_[3] = tx_frame_mat.data_length;

    for (int q = 0; q < tx_frame_mat.data_length; q++)
    {
        sendBuffer_[4 + q] = tx_frame_mat.data.buff_msg[q];
    }

    // 发送时仍然启用 CRC 校验
    tx_frame_mat.check_code.crc_code = CRC16_Table(tx_frame_mat.data.buff_msg, tx_frame_mat.data_length);
    sendBuffer_[4 + tx_frame_mat.data_length] = tx_frame_mat.check_code.crc_buff[0];
    sendBuffer_[5 + tx_frame_mat.data_length] = tx_frame_mat.check_code.crc_buff[1];
    sendBuffer_[6 + tx_frame_mat.data_length] = FRAME_END_0_RC9;
    sendBuffer_[7 + tx_frame_mat.data_length] = FRAME_END_1_RC9;

    Serial.write(sendBuffer_, tx_frame_mat.data_length + 8);
}

void RC9Protocol::pack_data(XboxControllerNotificationParser *xboxNotif){
    tx_frame_mat.data.buff_msg[0] = xboxNotif->btnY;
    tx_frame_mat.data.buff_msg[1] = xboxNotif->btnB;
    tx_frame_mat.data.buff_msg[2] = xboxNotif->btnA;
    tx_frame_mat.data.buff_msg[3] = xboxNotif->btnX;
    tx_frame_mat.data.buff_msg[4] = xboxNotif->btnShare;
    tx_frame_mat.data.buff_msg[5] = xboxNotif->btnStart;
    tx_frame_mat.data.buff_msg[6] = xboxNotif->btnSelect;
    tx_frame_mat.data.buff_msg[7] = xboxNotif->btnXbox;
    tx_frame_mat.data.buff_msg[8] = xboxNotif->btnLB;
    tx_frame_mat.data.buff_msg[9] = xboxNotif->btnRB;
    tx_frame_mat.data.buff_msg[10] = xboxNotif->btnLS;
    tx_frame_mat.data.buff_msg[11] = xboxNotif->btnRS;
    tx_frame_mat.data.buff_msg[12] = xboxNotif->btnDirUp;
    tx_frame_mat.data.buff_msg[13] = xboxNotif->btnDirLeft;
    tx_frame_mat.data.buff_msg[14] = xboxNotif->btnDirRight;
    tx_frame_mat.data.buff_msg[15] = xboxNotif->btnDirDown;
    tx_frame_mat.data.buff_msg[16] = (uint8_t)(xboxNotif->joyLHori >> 8) & 0xFF;
    tx_frame_mat.data.buff_msg[17] = (uint8_t)(xboxNotif->joyLHori & 0xFF);
    tx_frame_mat.data.buff_msg[18] = (uint8_t)(xboxNotif->joyLVert >> 8);
    tx_frame_mat.data.buff_msg[19] = (uint8_t)(xboxNotif->joyLVert & 0xFF);
    tx_frame_mat.data.buff_msg[20] = (uint8_t)(xboxNotif->joyRHori >> 8);
    tx_frame_mat.data.buff_msg[21] = (uint8_t)(xboxNotif->joyRHori & 0xFF);
    tx_frame_mat.data.buff_msg[22] = (uint8_t)(xboxNotif->joyRVert >> 8);
    tx_frame_mat.data.buff_msg[23] = (uint8_t)(xboxNotif->joyRVert & 0xFF);
    tx_frame_mat.data.buff_msg[24] = (uint8_t)(xboxNotif->trigLT >> 8);
    tx_frame_mat.data.buff_msg[25] = (uint8_t)(xboxNotif->trigLT & 0xFF);
    tx_frame_mat.data.buff_msg[26] = (uint8_t)(xboxNotif->trigRT >> 8);
    tx_frame_mat.data.buff_msg[27] = (uint8_t)(xboxNotif->trigRT & 0xFF);
    tx_frame_mat.data_length = 28;
}

void RC9Protocol::send_zero(){
    tx_frame_mat.data.buff_msg[0] = 0;
    tx_frame_mat.data.buff_msg[1] = 0;
    tx_frame_mat.data.buff_msg[2] = 0;
    tx_frame_mat.data.buff_msg[3] = 0;
    tx_frame_mat.data.buff_msg[4] = 0;
    tx_frame_mat.data.buff_msg[5] = 0;
    tx_frame_mat.data.buff_msg[6] = 0;
    tx_frame_mat.data.buff_msg[7] = 0;
    tx_frame_mat.data.buff_msg[8] = 0;
    tx_frame_mat.data.buff_msg[9] = 0;
    tx_frame_mat.data.buff_msg[10] = 0;
    tx_frame_mat.data.buff_msg[11] = 0;
    tx_frame_mat.data.buff_msg[12] = 0;
    tx_frame_mat.data.buff_msg[13] = 0;
    tx_frame_mat.data.buff_msg[14] = 0;
    tx_frame_mat.data.buff_msg[15] = 0;
    tx_frame_mat.data.buff_msg[16] = 0x80;
    tx_frame_mat.data.buff_msg[17] = 0x00;
    tx_frame_mat.data.buff_msg[18] = 0x80;
    tx_frame_mat.data.buff_msg[19] = 0x00;
    tx_frame_mat.data.buff_msg[20] = 0x80;
    tx_frame_mat.data.buff_msg[21] = 0x00;
    tx_frame_mat.data.buff_msg[22] = 0x80;
    tx_frame_mat.data.buff_msg[23] = 0x00;
    tx_frame_mat.data.buff_msg[25] = 0;
    tx_frame_mat.data.buff_msg[24] = 0;
    tx_frame_mat.data.buff_msg[26] = 0;
    tx_frame_mat.data.buff_msg[27] = 0;
    tx_frame_mat.data_length = 28;
    send_data();
}

