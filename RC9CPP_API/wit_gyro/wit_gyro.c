#include "wit_gyro.h"

/* 全局变量定义 */
uint8_t GYR_Buffer[1];
unsigned char ucTemp[1];
float fAngle[3], fAcc[3], fGyro[3];
volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

/* 静态函数声明 */
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void Delayms(uint16_t ucMs);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);

/* 初始化函数 */
void GYR_Init(void) {
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
}

/* 数据更新函数 */
void GYR_Updata(void) {
    if (s_cDataUpdate) {
        for (int i = 0; i < 3; i++) {
            fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
            fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
            fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
        }

        if (s_cDataUpdate & ANGLE_UPDATE) {
            s_cDataUpdate &= ~ANGLE_UPDATE;
        }
        if (s_cDataUpdate & ACC_UPDATE) {
            s_cDataUpdate &= ~ACC_UPDATE;
        }
        if (s_cDataUpdate & GYRO_UPDATE) {
            s_cDataUpdate &= ~GYRO_UPDATE;
        }
    }
}

/* 命令数据处理函数 */
void CopeCmdData(unsigned char ucData) {
    static unsigned char s_ucData[50], s_ucRxCnt = 0;

    s_ucData[s_ucRxCnt++] = ucData;
    if (s_ucRxCnt < 3) return; // 少于三个数据直接返回
    if (s_ucRxCnt >= 50) s_ucRxCnt = 0;
    if (s_ucRxCnt >= 3) {
        if ((s_ucData[1] == '\r') && (s_ucData[2] == '\n')) {
            s_cCmd = s_ucData[0];
            memset(s_ucData, 0, 50);
            s_ucRxCnt = 0;
        } else {
            s_ucData[0] = s_ucData[1];
            s_ucData[1] = s_ucData[2];
            s_ucRxCnt = 2;
        }
    }
}

/* 静态函数实现 */
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
    // HAL_UART_Transmit(&huart2, p_data, uiSize, 100);
}

static void Delayms(uint16_t ucMs) {
    HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    for (int i = 0; i < uiRegNum; i++) {
        switch (uiReg) {
            case AZ:
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HZ:
                s_cDataUpdate |= MAG_UPDATE;
                break;
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}