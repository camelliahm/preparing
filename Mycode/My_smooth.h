#ifndef __MY_SMOOTH_H
#define __MY_SMOOTH_H

#include "stm32f4xx.h"
#include "math.h"

/*平滑模式*/
typedef enum
{
    SMOOTH_NONE        = 0x00,    //无平滑
    SMOOTH_LINEAR_DEC  = 0x01,    //线性减速
    SMOOTH_CONST_ACCEL  = 0x02,   // 恒定加速度减速（抛物线）

}SmoothMode_t;

typedef struct 
{
    float smooth_x;       // 当前平滑后的X目标
    float smooth_y;       // 当前平滑后的Y目标
    float final_x;        // 最终目标X
    float final_y;        // 最终目标Y
    float smooth_speed;   // 平滑移动速度 (cm/s)
    uint8_t smoothing_active; // 平滑过程激活标志
} SmoothTarget;

extern SmoothTarget smooth_target; // 声明全局平滑目标结构体

void Set_Smooth_Target(float new_x, float new_y, float speed_cm_s, uint8_t active);
void Update_Smooth_Target(u8 mode);
uint8_t Is_Target_Reached(void);

#endif
