#ifndef __DRONE_PID_H
#define __DRONE_PID_H

#include "stm32f4xx.h"          
#include "PID_Mod.h"            

#define USE_LEGACY_CONTROL          0       //新旧PID控制开关
#define DAMPING_CONTROL             1       //阻尼控制开关
#define ADJUST_DYNAMIC_PARAMS       1       //运动状态动态调整参数开关

/* 常量定义 */
#define DEG_TO_RAD      (PI / 180.0f) /* 角度转弧度因子 */
#define DAMP_GAIN 0.8                /* 阻尼控制增益 */
#define MAX_SPEED 20                  /* 最大允许输出速度 */

/* X轴PID 参数 */
#define X_KP  0.60f                  /* X 轴比例系数 */
#define X_KI  0.0f                 /* X 轴积分系数 */
#define X_KD  0.2f                  /* X 轴微分系数 */
#define X_MAX_OUT 15              /* X 轴最大输出幅值 */
#define X_INT_LIM 8              /* X 轴积分限幅 */
#define X_DEADBAND 2.0f              /* X 轴死区阈值 */

/* Y轴PID参数 */
#define Y_KP  0.60f                  /* Y 轴比例系数 */
#define Y_KI  0.0f                 /* Y 轴积分系数 */
#define Y_KD  0.2f                  /* Y 轴微分系数 */
#define Y_MAX_OUT 15              /* Y 轴最大输出幅值 */
#define Y_INT_LIM 8              /* Y 轴积分限幅 */
#define Y_DEADBAND 2.0f              /* Y 轴死区阈值 */

/* Z轴PID参数 */
#define Z_KP   0.8f                   /* Z 轴比例系数 */
#define Z_KI   0.002f                 /* Z 轴积分系数 */
#define Z_KD   0.03f                  /* Z 轴微分系数 */
#define Z_MAX_OUT  40              /* Z 轴最大输出幅值 */
#define Z_INT_LIM  10              /* Z 轴积分限幅 */
#define Z_DEADBAND 2.0f               /* Z 轴死区阈值 */

/* 航向PID参数 */
#define YAW_KP 0.5f                   /* 航向比例系数 */
#define YAW_KD 0.5f                   /* 航向微分系数（无 I 项） */
#define YAW_MAX_OUT 3.0f              /* 航向最大输出角速度 */
#define YAW_DEADBAND 3.0f             /* 航向死区阈值（度） */

/* 动态调整幅度 */
#define DYN_KD_BOOST  1.5f            /* 运动时微分增益放大倍数 */
#define DYN_KI_REDUCE 0.3f            /* 运动时积分增益缩小倍数 */

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
