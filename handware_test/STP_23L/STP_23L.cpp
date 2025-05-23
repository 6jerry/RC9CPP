
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

#include "STP_23L.h"
#include <cstdint>


stp_23l::stp_23l(UART_HandleTypeDef *huart) : SerialDevice(huart){}

void stp_23l::Preprocessing_Data(u8 Uart_Receive_buf){
    u8 temp_data=Uart_Receive_buf;	
    if(state< 4) 																					 /* 起始符验证 前4个数据均为0xAA */
    {                                          
        if(temp_data == HEADER) state ++;
        else state = 0;
    }
    else if(state<10&&state>3)
    {
        switch(state)
        {
                case 4:   
                    if(temp_data == device_address)              /* 设备地址验证 */
                    {							
                                    state ++;
                                    crc = crc + temp_data;									
                                    break;        
                    } 
                    else state = 0,crc = 0;
                case 5:   
                    if(temp_data == PACK_GET_DISTANCE)					 /* 获取测量数据命令 */
                    {  
                                    PACK_FLAG = PACK_GET_DISTANCE;
                                    state ++;	
                                    crc = crc + temp_data;	
                                    break;									
                    }		 

                    else if(temp_data == PACK_RESET_SYSTEM) 		 /* 复位命令 */
                    {
                                    PACK_FLAG = PACK_RESET_SYSTEM;
                                    state ++; 
                                    crc = crc + temp_data;	
                                    break;	
                    }
                    else if(temp_data == PACK_STOP)							 /* 停止测量数据传输命令 */
                    { 
                                    PACK_FLAG = PACK_STOP;
                                    state ++; 
                                    crc = crc + temp_data;	
                                    break;
                    }
                    else if(temp_data == PACK_ACK)							 /* 应答码命令 */
                    {  
                                    PACK_FLAG = PACK_ACK;
                                    state ++;
                                    crc = crc + temp_data;	
                                    break;
                    }			 				 
                    else if(temp_data == PACK_VERSION)					 /* 获取传感器信息命令 */
                    { 
                                    PACK_FLAG = PACK_VERSION,
                                    state ++,
                                    crc = crc + temp_data;	   	     
                                    break;
                    }
                    else state = 0,crc = 0;
                case 6: if(temp_data == chunk_offset)          /* 偏移地址 */
                        {  
                            state ++;
                            crc = crc + temp_data;
                            break; 	  
                        }	
                        else state = 0,crc = 0;
                case 7: if(temp_data == chunk_offset)
                        {  
                            state ++;
                            crc = crc + temp_data;
                            break;
                        }
                        else state = 0,crc = 0;
                case 8: 
                        data_len = (u16)temp_data;								 /* 数据长度低八位 */
                        state ++; 
                        crc = crc + temp_data;
                        break;																			 
                case 9: 
                        data_len = data_len + ((u16)temp_data<<8); 			 /* 数据长度高八位 */
                        state ++;
                        crc = crc + temp_data;
                        break; 
                default: break;
            }
        }
    else if(state == 10 ) state_flag = 0;                    /*由switch跳出来时state为10，但temp_data仍为距离长度高八位数据，需跳过一次中断*/
    
    if(PACK_FLAG == PACK_GET_DISTANCE&&state_flag == 0)      /* 获取一帧数据并校验 */
    {
        if(state>9)
        {
            if(state<190)
            {
                    static uint8_t state_num;
                    state_num = (state-10)%15;
                    switch(state_num)
                    {
                            case 0: 
                                Pack_Data[cnt].distance = (uint16_t)temp_data ;				 /* 距离数据低八位 */
                                crc = crc + temp_data;
                                state++;
                                break;        
                            case 1: 
                                Pack_Data[cnt].distance = ((u16)temp_data<<8) + Pack_Data[cnt].distance;	 /* 距离数据 */
                                realtime_distance = Pack_Data[cnt].distance; /*实时距离数据*/
                                    if(realtime_distance < 15){  //死区限制
                                        realtime_distance = 0;
                                    }
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 2:
                                Pack_Data[cnt].noise = (u16)temp_data;				 /* 环境噪音低八位 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 3:
                                Pack_Data[cnt].noise = ((u16)temp_data<<8) + Pack_Data[cnt].noise;				 /* 环境噪音 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 4:
                                Pack_Data[cnt].peak = (u32)temp_data;				 										 /* 接受强度信息低八位 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 5:
                                Pack_Data[cnt].peak = ((u32)temp_data<<8) + Pack_Data[cnt].peak;
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 6:
                                Pack_Data[cnt].peak = ((u32)temp_data<<16) + Pack_Data[cnt].peak;	
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 7:
                                Pack_Data[cnt].peak = ((u32)temp_data<<24) + Pack_Data[cnt].peak;				    /* 接受强度信息 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 8:
                                Pack_Data[cnt].confidence = temp_data;				 /* 置信度 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 9:
                                Pack_Data[cnt].intg = (u32)temp_data;															/* 积分次数低八位 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 10:
                                Pack_Data[cnt].intg = ((u32)temp_data<<8) + Pack_Data[cnt].intg;
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 11:
                                Pack_Data[cnt].intg = ((u32)temp_data<<16) + Pack_Data[cnt].intg;
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 12:
                                Pack_Data[cnt].intg = ((u32)temp_data<<24) + Pack_Data[cnt].intg;				  	 /* 积分次数 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 13:
                                Pack_Data[cnt].reftof = (int16_t)temp_data;				 								 /* 温度表征值低八位 */
                                crc = crc + temp_data;
                                state++;
                                break; 
                            case 14:
                                Pack_Data[cnt].reftof = ((int16_t)temp_data<<8) +Pack_Data[cnt].reftof;			/* 温度表征值 */
                                crc = crc + temp_data;
                                state++;
                                cnt++;							 /* 进入下一个测量点 */
                                break; 
                            default: break;
                    }
                }
                /* 时间戳 */
                if(state == 190) timestamp = temp_data,state++,crc = crc + temp_data;
                else if(state == 191) timestamp = ((u32)temp_data<<8) + timestamp,state++,crc = crc + temp_data; 
                else if(state == 192) timestamp = ((u32)temp_data<<16) + timestamp,state++,crc = crc + temp_data;
                else if(state == 193) timestamp = ((u32)temp_data<<24) + timestamp,state++,crc = crc + temp_data; 
                else if(state==194)
                {
                            if(temp_data == crc)   /* 校验成功 */
                            {
                                Post_Processing_Data();  	 /* 数据处理函数，完成一帧之后可进行数据处理 */
                                receive_cnt++;	 	 /* 输出接收到正确数据的次数 */
                            }
                            distance = Pack_Data[0].distance;
                            crc = 0;
                            state = 0;
                            state_flag = 1;
                            cnt = 0; 							 /* 复位*/
                }
                
            }
    }
}

/*数据处理函数，完成一帧之后可进行数据处理*/
void stp_23l::Post_Processing_Data(void){
    /* 计算距离 */
		static u8 cnt = 0;
		u8 i;
		static u16 count = 0;
		static u32 sum = 0;
		for(i=0;i<12;i++)									/* 12个点取平均 */
		{
				if(Pack_Data[i].distance != 0)  /* 去除0的点 */
				{
						count++;
						Pack_sum.distance += Pack_Data[i].distance;
						Pack_sum.noise += Pack_Data[i].noise;
						Pack_sum.peak += Pack_Data[i].peak;
						Pack_sum.confidence += Pack_Data[i].confidence;
						Pack_sum.intg += Pack_Data[i].intg;
						Pack_sum.reftof += Pack_Data[i].reftof;
				}
		}
		if(count !=0)
		{

					distance = Pack_sum.distance/count;
					noise = Pack_sum.noise/count;
					peak = Pack_sum.peak/count;
					confidence = Pack_sum.confidence/count;
					intg = Pack_sum.intg/count;
					reftof = Pack_sum.reftof/count;
					Pack_sum.distance = 0;
					Pack_sum.noise = 0;
					Pack_sum.peak = 0;
					Pack_sum.confidence = 0;
					Pack_sum.intg = 0;
					Pack_sum.reftof = 0;
					count = 0;
		}
        if(distance < 15){  //死区限制
            distance = 0;
        }
}

void stp_23l::handleReceiveData(uint8_t byte){
    Preprocessing_Data(byte);
}