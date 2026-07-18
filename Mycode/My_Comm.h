#ifndef MY_COMM_H
#define MY_COMM_H

#include "stm32f4xx.h"                  
#include "string.h"
#include "Patrol_Task.h"

#define ANO_VISION_FRAME   0xF3    // 视觉数据使用F3灵活格式帧
#define ANO_GROUND_FRAME   0xF4    // 地面站数据使用F4灵活格式帧

//地面站数据结构体
typedef struct
{
    Cargo freight[36];
    u8 mission;
}Ground_Data_TypeDef;

//视觉数据结构体
typedef struct
{
    u8 number;
}Vision_Track_Data;

extern Ground_Data_TypeDef ground_data;
extern Vision_Track_Data vision_data;

void My_Comm_Init(void);
void My_Comm_ReceiveByte(u8 data);
void ANO_Send_Custom(u8 *data_buf, u8 data_len, u8 frame, void (*DrvUartSendBuf)(unsigned char *, u8));

#endif
