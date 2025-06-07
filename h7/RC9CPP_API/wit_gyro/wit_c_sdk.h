#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN    2
#define WIT_PROTOCOL_I2C    3

#define REGSIZE 0x90

#define SAVE 		0x00
#define CALSW 		0x01
#define RSW 		0x02
#define RRATE		0x03
#define BAUD 		0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b`
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define MAGRANGX	0x1c
#define MAGRANGY	0x1d
#define MAGRANGZ	0x1e
#define BANDWIDTH	0x1f
#define GYRORANGE	0x20
#define ACCRANGE	0x21
#define SLEEP       0x22
#define ORIENT		0x23
#define AXIS6       0x24
#define FILTK       0x25
#define GPSBAUD		0x26
#define READADDR	0x27
#define BWSCALE		0x28
#define MOVETHR		0x28
#define MOVESTA		0x29
#define ACCFILT		0x2A
#define GYROFILT	0x2b
#define MAGFILT		0x2c
#define POWONSEND	0x2d
#define VERSION		0x2e
#define CCBW			  0x2f
#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight       0x4d
#define GPSYAW          0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
#define q0					0x51
#define q1					0x52
#define q2					0x53
#define q3					0x54
#define SVNUM				0x55
#define PDOP				0x56
#define HDOP				0x57
#define VDOP				0x58
#define DELAYT			0x59
#define XMIN            0x5a
#define XMAX            0x5b
#define BATVAL          0x5c
#define ALARMPIN        0x5d
#define YMIN            0x5e
#define YMAX            0x5f
#define GYROZSCALE		0x60
#define GYROCALITHR     0x61
#define ALARMLEVEL      0x62
#define GYROCALTIME		0x63
#define REFROLL			0x64
#define REFPITCH		0x65
#define REFYAW			0x66
#define GPSTYPE     0x67
#define TRIGTIME    0x68
#define KEY         0x69
#define WERROR      0x6a
#define TIMEZONE    0x6b
#define CALICNT     0x6c
#define WZCNT       0x6d
#define WZTIME      0x6e
#define WZSTATIC    0x6f
#define ACCSENSOR 	0x70
#define GYROSENSOR 	0x71
#define MAGSENSOR 	0x72
#define PRESSENSOR 	0x73
#define MODDELAY    0x74

#define ANGLEAXIS   0x75
#define XRSCALE			0x76    
#define YRSCALE			0x77
#define ZRSCALE			0x78

#define XREFROLL		0x79    
#define YREFPITCH		0x7a
#define ZREFYAW			0x7b

#define ANGXOFFSET		0x7c    
#define ANGYOFFSET		0x7d
#define ANGZOFFSET		0x7e

#define NUMBERID1    0x7f
#define NUMBERID2    0x80
#define NUMBERID3    0x81
#define NUMBERID4    0x82
#define NUMBERID5    0x83
#define NUMBERID6    0x84

#define XA85PSCALE       0x85
#define XA85NSCALE       0x86
#define YA85PSCALE       0x87
#define YA85NSCALE       0x88
#define XA30PSCALE       0x89
#define XA30NSCALE       0x8a
#define YA30PSCALE       0x8b
#define YA30NSCALE       0x8c

#define CHIPIDL     0x8D
#define CHIPIDH     0x8E
#define REGINITFLAG       REGSIZE-1 


/* AXIS6 */
#define ALGRITHM9 0
#define ALGRITHM6 1

/************CALSW**************/
#define NORMAL          0x00
#define CALGYROACC      0x01
#define CALMAG          0x02
#define CALALTITUDE     0x03
#define CALANGLEZ       0x04
#define CALACCL         0x05
#define CALACCR         0x06
#define CALMAGMM        0x07
#define CALREFANGLE		0x08
#define CALMAG2STEP		0x09
//#define CALACCX       0x09
//#define ACC45PRX      0x0A
//#define ACC45NRX      0x0B
//#define CALACCY       0x0C
//#define ACC45PRY      0x0D
//#define ACC45NRY     	0x0E
//#define CALREFANGLER  0x0F
//#define CALACCINIT    0x10
//#define CALREFANGLEINIT 0x11
#define CALHEXAHEDRON		0x12

/************OUTPUTHEAD**************/
#define WIT_TIME        0x50
#define WIT_ACC         0x51
#define WIT_GYRO        0x52
#define WIT_ANGLE       0x53
#define WIT_MAGNETIC    0x54
#define WIT_DPORT       0x55
#define WIT_PRESS       0x56
#define WIT_GPS         0x57
#define WIT_VELOCITY    0x58
#define WIT_QUATER      0x59
#define WIT_GSA         0x5A
#define WIT_REGVALUE    0x5F

/************RSW**************/
#define RSW_TIME 	0x01
#define RSW_ACC		0x02
#define RSW_GYRO	0x04
#define RSW_ANGLE	0x08
#define RSW_MAG		0x10
#define RSW_PORT	0x20
#define RSW_PRESS	0x40
#define RSW_GPS		0x80
#define RSW_V       0x100
#define RSW_Q       0x200
#define	RSW_GSA		0x400
#define RSW_MASK	0xfff

/**RRATE*****/
#define RRATE_NONE	0x0d
#define RRATE_02HZ	0x01
#define RRATE_05HZ	0x02
#define RRATE_1HZ 	0x03
#define RRATE_2HZ 	0x04
#define RRATE_5HZ 	0x05
#define RRATE_10HZ	0x06
#define RRATE_20HZ	0x07
#define RRATE_50HZ	0x08
#define RRATE_100HZ	0x09
#define RRATE_125HZ	0x0a	//only WT931
#define RRATE_200HZ	0x0b
#define RRATE_ONCE 	0x0c

/* BAUD */
#define WIT_BAUD_4800	  1
#define WIT_BAUD_9600	  2
#define WIT_BAUD_19200	  3
#define WIT_BAUD_38400	  4
#define WIT_BAUD_57600	  5
#define WIT_BAUD_115200	  6
#define WIT_BAUD_230400	  7
#define WIT_BAUD_460800	  8
#define WIT_BAUD_921600	  9

/*CAN BAUD*/
#define CAN_BAUD_1000000	 	0
#define CAN_BAUD_800000	  		1
#define CAN_BAUD_500000	  		2
#define CAN_BAUD_400000	  		3
#define CAN_BAUD_250000	  		4
#define CAN_BAUD_200000   		5
#define CAN_BAUD_125000	  		6
#define CAN_BAUD_100000  		7
#define CAN_BAUD_80000	  		8
#define CAN_BAUD_50000  		9
#define CAN_BAUD_40000	  		10
#define CAN_BAUD_20000  		11
#define CAN_BAUD_10000	  		12
#define CAN_BAUD_5000	  		13
#define CAN_BAUD_3000	  		14

/* KEY */
#define KEY_UNLOCK	0xB588

/* SAVE */
#define SAVE_PARAM	0x00
#define SAVE_SWRST	0xFF

/* ORIENT */
#define ORIENT_HERIZONE	0
#define ORIENT_VERTICLE	1

/* BANDWIDTH */
#define BANDWIDTH_256HZ	0
#define BANDWIDTH_184HZ	1
#define BANDWIDTH_94HZ	2
#define BANDWIDTH_44HZ	3
#define BANDWIDTH_21HZ	4
#define BANDWIDTH_10HZ	5
#define BANDWIDTH_5HZ	6

/* serial function */
typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
int32_t WitSerialWriteRegister(SerialWrite write_func);
void WitSerialDataIn(uint8_t ucData);

/* iic function */

/*
    i2c write function example

    int32_t WitI2cWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
    {
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        for(uint32_t i = 0; i < uiLen; i++)
        {
            i2c_send(*p_ucVal++); 
            if(i2c_wait_ask() != SUCCESS)return 0;
        }
        i2c_stop();
        return 1;
    }
*/
typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
/*
    i2c read function example

    int32_t WitI2cRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
    {
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        
        i2c_start();
        i2c_send(ucAddr+1);
        for(uint32_t i = 0; i < uiLen; i++)
        {
            if(i+1 == uiLen)*p_ucVal++ = i2c_read(0);  //last byte no ask
            else *p_ucVal++ = i2c_read(1);  //  ask
        }
        i2c_stop();
        return 1;
    }
*/
typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);

/* can function */
typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
int32_t WitCanWriteRegister(CanWrite write_func);

/* Delayms function */
typedef void (*DelaymsCb)(uint16_t ucMs);
int32_t WitDelayMsRegister(DelaymsCb delayms_func);


void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
int32_t WitRegisterCallBack(RegUpdateCb update_func);
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);
void WitDeInit(void);



/**
  ******************************************************************************
  * @file    wit_c_sdk.h
  * @author  Wit
  * @version V1.0
  * @date    05-May-2022
  * @brief   This file provides all Configure sensor function.
  ******************************************************************************
  * @attention
  *
  *        http://wit-motion.cn/
  *
  ******************************************************************************
  */
int32_t WitStartAccCali(void);
int32_t WitStopAccCali(void);
int32_t WitStartMagCali(void);
int32_t WitStopMagCali(void);
int32_t WitSetUartBaud(int32_t uiBaudIndex);
int32_t WitSetBandwidth(int32_t uiBaudWidth);
int32_t WitSetOutputRate(int32_t uiRate);
int32_t WitSetContent(int32_t uiRsw);
int32_t WitSetCanBaud(int32_t uiBaudIndex);

char CheckRange(short sTemp,short sMin,short sMax);

extern int16_t sReg[REGSIZE];

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */
