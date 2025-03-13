
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once
// 使用 extern "C" 包裹 C 语言的头文件
#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

#include "Serial_device.h"
#include "math.h"
#include "stdint.h"

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

/**********************原代码自定义结构体与宏定义**********************/
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern char  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记	

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define false 0
#define true 1

#define HEADER 0xAA							/* 起始符 */
#define device_address 0x00     /* 设备地址 */
#define chunk_offset 0x00       /* 偏移地址命令 */
#define PACK_GET_DISTANCE 0x02 	/* 获取测量数据命令 */
#define PACK_RESET_SYSTEM 0x0D 	/* 复位命令 */
#define PACK_STOP 0x0F 				  /* 停止测量数据传输命令 */
#define PACK_ACK 0x10           /* 应答码命令 */
#define PACK_VERSION 0x14       /* 获取传感器信息命令 */

typedef struct {
	int16_t distance;  						/* 距离数据：测量目标距离单位 mm */
	uint16_t noise;		 						/* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
	uint32_t peak;								/* 接收强度信息：测量目标反射回的光强度 */
	uint8_t confidence;						/* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
	uint32_t intg;     						/* 积分次数：当前传感器测量的积分次数 */
	int16_t reftof;   						/* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
}LidarPointTypedef;

struct AckResultData{
	uint8_t ack_cmd_id;						/* 答复的命令 id */
	uint8_t result; 							/* 1表示成功,0表示失败 */
};

struct LiManuConfig
{
	uint32_t version; 						/* 软件版本号 */
	uint32_t hardware_version; 		/* 硬件版本号 */
	uint32_t manufacture_date; 		/* 生产日期 */
	uint32_t manufacture_time; 		/* 生产时间 */
	uint32_t id1; 								/* 设备 id1 */
	uint32_t id2; 								/* 设备 id2 */
	uint32_t id3; 								/* 设备 id3 */
	uint8_t sn[8]; 								/* sn */
	uint16_t pitch_angle[4]; 			/* 角度信息 */
	uint16_t blind_area[2]; 			/* 盲区信息 */
	uint32_t frequence; 					/* 数据点频 */
};

/********************************************************************/
class stp_23l: public SerialDevice{
    private:
        char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
        uint16_t point1 ;
        LidarPointTypedef Pack_Data[12];/* 雷达接收的数据储存在这个变量之中（直出数据） */
        LidarPointTypedef Pack_sum;     /* 输出结果储存（后解析用） */
        /*******************中断回调函数使用******************/
        uint8_t state = 0;			//状态位	
	    uint8_t crc = 0;				//校验和
	    uint8_t cnt = 0;				//用于一帧12个点的计数
	    uint8_t PACK_FLAG = 0;  //命令标志位
	    uint8_t data_len  = 0;  //数据长度
	    uint32_t timestamp = 0; //时间戳
	    uint8_t state_flag = 1; //转入数据接收标志位
		uint16_t realtime_distance = 25; //死区大概为25(mm)
        /*****************************************************/
    public:
    u16 receive_cnt;//计算成功接收数据帧次数
    u8 confidence;
    u16 distance,noise,reftof; //直接获取distance即测距距离
    u32 peak,intg;
    u8 dis;
    stp_23l(UART_HandleTypeDef *huart); // 构造函数
    void Preprocessing_Data(u8 Uart_Receive_buf); //预处理数据函数
    void Post_Processing_Data(void); //后处理数据函数（在预处理最后判断是否接收一数据帧时调用）
    void handleReceiveData(uint8_t byte);
};









#endif