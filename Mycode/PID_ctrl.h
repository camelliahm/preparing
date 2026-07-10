#ifndef __DRONE_PID_H
#define __DRONE_PID_H

#include "stm32f4xx.h"          
#include "PID_Mod.h"            

/* UWB 位置信息结构体 */
typedef struct
{
    s16 uwb_target_x;          /* X 轴目标位置（厘米） */
    s16 uwb_target_y;          /* Y 轴目标位置（厘米） */
    s16 uwb_target_z;          /* Z 轴目标高度（厘米） */
    float target_yaw_deg;   /* 目标航向角（度） */
    s16 now_x;                 /* 当前 X 轴位置（厘米） */
    s16 now_y;                 /* 当前 Y 轴位置（厘米） */
    s16 now_z;                 /* 当前 Z 轴高度（厘米） */
    s16 target_init_done;      /* 目标初始化完成标志，非零表示完成 */
    u8 axis_mask;                /* 轴控制掩码 */
    u8  pid_enable_flag;    /* PID 控制使能标志，非零使能 */
    u8  is_moving_x;        /* X 方向正在运动标志 */
    u8  is_moving_y;        /* Y 方向正在运动标志 */
    u8   yaw_lock;          /* 航向锁定标志，非零时启用航向控制 */
} UWB_PosInfo;

/* ========== 轴选择掩码（按位组合） ========== */
typedef enum {
    DRONE_AXIS_X    = 0x01,   // 水平X轴
    DRONE_AXIS_Y    = 0x02,   // 水平Y轴
    DRONE_AXIS_Z    = 0x04,   // 高度轴
    DRONE_AXIS_YAW  = 0x08,   // 偏航轴
    DRONE_AXIS_XY   = 0x03,   // X+Y
    DRONE_AXIS_ALL  = 0x0F    // 全部轴
} DroneAxisMask_t;

/* 全局 PID 对象 */
extern PID_TypeDef pid_x;      /* X 轴位置 PID 控制器 */
extern PID_TypeDef pid_y;      /* Y 轴位置 PID 控制器 */
extern PID_TypeDef pid_z;      /* Z 轴高度 PID 控制器 */
extern PID_TypeDef pid_yaw;    /* 航向角 PID 控制器 */

/* 飞控输出数组：vel_x, vel_y, vel_z, yaw_dps */
extern s16 PID_ctrl[4];        /* PID 控制输出，依次为体坐标系 X/Y 速度、Z 速度、偏航角速度 */

/* UWB 信息 */
extern UWB_PosInfo uwb_pos_info; /* UWB 定位数据实例 */

/* IMU 姿态角（弧度） */
extern float imu_angle;        /* 无人机偏航角（弧度） */
extern float my_imu_pit_rad;   /* 俯仰角（弧度），目前未使用 */
extern float my_imu_rol_rad;   /* 滚转角（弧度），目前未使用 */

/* 函数声明 */
void Get_UWB_Pos(void);        /* 获取并处理 UWB 位置数据 */
void Get_Gen_Dis(void);        /* 获取并处理高度（激光/光流）数据 */
void Drone_PID_Init(void);     /* 初始化所有 PID 控制器和相关变量 */
void UWB_Position_PID(void);   /* UWB 位置 PID 主控制循环 */
void Drone_Control_Update(uint8_t axis_mask, uint8_t smooth_mode); /* 更新无人机控制输出，按轴选择和是否平滑移动 */
void Take_Off(u32 target_height); /* 起飞控制，设定目标高度并启动 Z 轴 PID */

#endif /* __DRONE_PID_H */
