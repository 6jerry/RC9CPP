#ifndef __GYRO_H
#define __GYRO_H

#include "main.h"
extern uint8_t GYR_Buffer[1];//


extern unsigned char ucTemp[1];
extern float fAngle[3];
void GYR_Updata(void);
void GYR_Init(void);



#endif
