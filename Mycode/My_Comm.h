#ifndef MY_COMM_H
#define MY_COMM_H

#include "stm32f4xx.h"                  
#include "string.h"

#define ANO_VISION_FRAME   0xF3    // 视觉数据使用F3灵活格式帧
#define ANO_GROUND_FRAME   0xF4    // 地面站数据使用F4灵活格式帧

//地面站数据结构体
typedef struct
{
    u8 NoFly_Zone1[4];
    u8 NoFly_Zone2[4];
    u8 NoFly_Zone3[4];
    u8 update_flag;
}Ground_Data_TypeDef;

//视觉数据结构体
typedef struct
{
    u8 animal_number; // 动物代号
    uint8_t animal_count;        // 该方格内该种动物的数量
}Vision_Track_Data;

extern Ground_Data_TypeDef ground_data;
extern Vision_Track_Data vision_data;

void My_Comm_Init(void);
void My_Comm_ReceiveByte(u8 data);
void ANO_Send_Custom_F4(u8 *data_buf, u8 data_len);

#endif
