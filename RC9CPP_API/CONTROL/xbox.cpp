/*******************************************************************************
 * @file xbox.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @brief xbox remote control.
 * @version 1.0
 * @date 2024-10-26
 *
 * @copyright Copyright (c) 2024-10-26 6Jerry
 *
 * @license MIT
 *
 * @disclaimer This software is provided "as is", without warranty of any kind, express or implied,
 *             including but not limited to the warranties of merchantability, fitness for a
 *             particular purpose and noninfringement. In no event shall the authors be liable for any
 *             claim, damages or other liability, whether in an action of contract, tort or otherwise,
 *             arising from, out of or in connection with the software or the use or other dealings
 *             in the software.
 ******************************************************************************/
#include "xbox.h"

void xbox::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    if (byteCount == 28)
    {
        // 解析按键数据 (bool 值)
        xbox_msgs.btnY = byteData[0];
        xbox_msgs.btnB = byteData[1];
        xbox_msgs.btnA = byteData[2];
        xbox_msgs.btnX = byteData[3];
        xbox_msgs.btnShare = byteData[4];
        xbox_msgs.btnStart = byteData[5];
        xbox_msgs.btnSelect = byteData[6];
        xbox_msgs.btnXbox = byteData[7];
        xbox_msgs.btnLB = byteData[8];
        xbox_msgs.btnRB = byteData[9];
        xbox_msgs.btnLS = byteData[10];
        xbox_msgs.btnRS = byteData[11];
        xbox_msgs.btnDirUp = byteData[12];
        xbox_msgs.btnDirLeft = byteData[13];
        xbox_msgs.btnDirRight = byteData[14];
        xbox_msgs.btnDirDown = byteData[15];

        // 解析霍尔传感器值（16位数据，高8位和低8位拼接）
        xbox_msgs.joyLHori = ((uint16_t)byteData[16] << 8) | byteData[17];
        xbox_msgs.joyLVert = ((uint16_t)byteData[18] << 8) | byteData[19];
        xbox_msgs.joyRHori = ((uint16_t)byteData[20] << 8) | byteData[21];
        xbox_msgs.joyRVert = ((uint16_t)byteData[22] << 8) | byteData[23];
        xbox_msgs.trigLT = ((uint16_t)byteData[24] << 8) | byteData[25];
        xbox_msgs.trigRT = ((uint16_t)byteData[26] << 8) | byteData[27];
    }
}
void xbox::joymap_compute()
{
    if (xbox_msgs.joyLHori > 31000 && xbox_msgs.joyLHori < 35000)
    {
        xbox_msgs.joyLHori_map = 0.0f;
    }
    if (xbox_msgs.joyLHori <= 31000)
    {
        xbox_msgs.joyLHori_map = (31000.0f - (float)xbox_msgs.joyLHori) / 31000.0f;
    }
    if (xbox_msgs.joyLHori >= 35000)
    {
        xbox_msgs.joyLHori_map = (35000.0f - (float)xbox_msgs.joyLHori) / 30535.0f;
    }

    if (xbox_msgs.joyLVert > 31000 && xbox_msgs.joyLVert < 35000)
    {
        xbox_msgs.joyLVert_map = 0.0f;
    }
    if (xbox_msgs.joyLVert <= 31000)
    {
        xbox_msgs.joyLVert_map = (31000.0f - (float)xbox_msgs.joyLVert) / 31000.0f;
    }
    if (xbox_msgs.joyLVert >= 35000)
    {
        xbox_msgs.joyLVert_map = (35000.0f - (float)xbox_msgs.joyLVert) / 30535.0f;
    }

    if (xbox_msgs.joyRHori > 31000 && xbox_msgs.joyRHori < 35000)
    {
        xbox_msgs.joyRHori_map = 0.0f;
    }
    if (xbox_msgs.joyRHori <= 31000)
    {
        xbox_msgs.joyRHori_map = (31000.0f - (float)xbox_msgs.joyRHori) / 31000.0f;
    }
    if (xbox_msgs.joyRHori >= 35000)
    {
        xbox_msgs.joyRHori_map = (35000.0f - (float)xbox_msgs.joyRHori) / 30535.0f;
    }
    if (xbox_msgs.joyRVert > 31000 && xbox_msgs.joyRVert < 35000)
    {
        xbox_msgs.joyRVert_map = 0.0f;
    }
    if (xbox_msgs.joyRVert <= 31000)
    {
        xbox_msgs.joyRVert_map = (31000.0f - (float)xbox_msgs.joyRVert) / 31000.0f;
    }
    if (xbox_msgs.joyRVert >= 35000)
    {
        xbox_msgs.joyRVert_map = (35000.0f - (float)xbox_msgs.joyRVert) / 30535.0f;
    }
    xbox_msgs.trigRT_map = (float)xbox_msgs.trigRT / 1023.0f;
    xbox_msgs.trigLT_map = (float)xbox_msgs.trigLT / 1023.0f;
}

void xbox::handleButton(ButtonConfig &config)
{
    if (*(config.currentState) && !(*(config.lastState)))
    {
        switch (config.actionType)
        {
        case ButtonActionType::Toggle:
            if (config.toggleState)
            {
                *config.toggleState = (*config.toggleState + 1) % (config.maxState + 1);
            }
            break;
        case ButtonActionType::Increment:
            if (config.toggleState && *config.toggleState < config.maxState)
            {
                (*config.toggleState)++;
            }
            break;
        case ButtonActionType::Decrement:
            if (config.toggleState && *config.toggleState > 0)
            {
                (*config.toggleState)--;
            }
            break;
        case ButtonActionType::Custom:
            if (config.customAction)
            {
                (this->*config.customAction)(); // 调用成员函数
            }
            break;
        }
    }
    *config.lastState = *config.currentState;
}
