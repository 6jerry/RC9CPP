#include "main.h"
#include "usart.h"

#include "gyro.h"
#include <string.h>
#include "stdio.h"
#include "wit_c_sdk.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
#define to_rad 0.0174533		


unsigned char ucTemp[1];
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
uint8_t GYR_Buffer[1];
float fAngle[3];
float rad;
int i;

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
//	HAL_UART_Transmit(&huart2,p_data,uiSize,100);
}

static void Delayms(uint16_t ucMs)
{
	HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
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

void GYR_Init(void)
{
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);

}

void GYR_Updata(void)
{

		if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}

			if(s_cDataUpdate & ANGLE_UPDATE)
			{
//				printf("angle:%.3f\r\n",fAngle[2]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}

		}
	
}



void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}

}



