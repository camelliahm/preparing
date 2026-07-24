#include "Drv_Uart.h"         
#include "Ano_DT_LX.h"       
#include "Drv_AnoOf.h"       
#include "stdlib.h"           
#include "string.h"          
#include "math.h"             
#include "Ano_Math.h"         
#include "UWB_location.h"      
#include "PID_ctrl.h"          
#include "LX_FC_Fun.h"        
#include "LX_FC_EXT_Sensor.h"
#include "My_smooth.h"  

#define PI 3.14159265f                /* 圆周率 */

// 顺序：vel_x, vel_y, vel_z, yaw_dps
s16 PID_ctrl[4] = {0};               /* 飞控输出数组，体坐标系 X/Y 速度、Z 速度、偏航角速度 */

// PID 对象
PID_TypeDef pid_x, pid_y, pid_z, pid_yaw; /* 实例化 X/Y/Z/航向 PID 控制器 */

// UWB 位置信息
UWB_PosInfo uwb_pos_info = {         /* 初始化控制信息结构体 */
    .uwb_target_x = 0,               /* 目标 X 清零 */
    .uwb_target_y = 0,               /* 目标 Y 清零 */
    .uwb_target_z = 0,               /* 目标 Z 清零 */
    .target_yaw_deg = 0.0f,          /* 目标航向角清零 */
    .now_x = 0,                      /* 当前 X 清零 */
    .now_y = 0,                      /* 当前 Y 清零 */
    .now_z = 0,                      /* 当前 Z 清零 */
    .target_init_done = 0,           /* 目标未初始化 */
    .pid_enable_flag = 0,            /* PID 未使能 */
    .is_moving_x = 0,                /* X 轴未运动 */
    .is_moving_y = 0,                /* Y 轴未运动 */
    .yaw_lock = 0                     /* 航向未锁定 */

};

extern float imu_angle, my_imu_pit_rad, my_imu_rol_rad; /* 引用外部的 IMU 姿态角 */
extern UWB_info UWB;                 /* 引用外部的 UWB 原始信息结构体 */

// 跳变保护用静态变量
static s16 last_x = 0, last_y = 0, last_z = 0; /* 上一帧有效坐标，用于滤除跳变 */
static u16 valid_frame_count = 0;    /* UWB 有效帧计数器，用于初始化稳定判断 */

// 辅助宏：取绝对值，用于 s16 类型
static s16 abs16(s16 x) { return x > 0 ? x : -x; } /* 静态内联函数，计算 s16 绝对值 */

// 限幅宏：将 val 限制在 min 与 max 之间
#define CLAMP(val, min, max)  ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

/**
 * @brief 获取 UWB 位置数据，并进行跳变保护和目标初始化
 *        初始化阶段需要连续 300 帧有效数据后锁定目标位置和航向
 * @author camelliahm
 */
void Get_UWB_Pos(void)
{
    s16 raw_x = UWB.pose_x_cm;       /* 从 UWB 信息中读取原始 X 坐标（厘米） */
    s16 raw_y = UWB.pose_y_cm;       /* 从 UWB 信息中读取原始 Y 坐标（厘米） */

    if (uwb_pos_info.target_init_done == 0)  /* 尚未完成目标初始化 */
    {
        if (UWB.uwb_work_sta == 1)   /* UWB 工作正常 */
        {
            valid_frame_count++;     /* 有效帧计数加1 */
            if (valid_frame_count >= 300) /* 连续收到 300 帧有效数据 */
            {
                uwb_pos_info.uwb_target_x = raw_x; /* 锁定当前 X 坐标作为目标 */
                uwb_pos_info.uwb_target_y = raw_y; /* 锁定当前 Y 坐标作为目标 */
                last_x = raw_x;      /* 记录当前有效坐标，供后续跳变保护使用 */
                last_y = raw_y;
                uwb_pos_info.target_init_done = 1; /* 标记初始化完成 */
                uwb_pos_info.target_yaw_deg = imu_angle * 57.2958f; /* 记录当前航向作为目标航向 (弧度转度) */
                uwb_pos_info.yaw_lock = 1;        /* 开启航向锁定 */
            }
        }
        else
        {
            valid_frame_count = 0;   /* UWB 不工作时重置计数 */
        }
        uwb_pos_info.now_x = raw_x;  /* 更新当前位置为原始值 */
        uwb_pos_info.now_y = raw_y;
    }
    else  /* 初始化已完成，进行跳变保护 */
    {
        if (abs16(raw_x - last_x) > 15) raw_x = last_x; /* X 坐标跳变超过 15cm 则丢弃，使用上一帧值 */
        if (abs16(raw_y - last_y) > 15) raw_y = last_y; /* Y 坐标跳变保护 */
        uwb_pos_info.now_x = raw_x;  /* 更新滤波后的当前位置 */
        uwb_pos_info.now_y = raw_y;
        last_x = raw_x;              /* 保存本次值作为下一次基准 */
        last_y = raw_y;
    }
}

/**
 * @brief 获取高度数据（光流/激光测距），并进行跳变保护
 *       跳变保护阈值设置为 10cm，超过则丢弃当前读数，使用上一帧高度
 * @author camelliahm
 */
void Get_Gen_Dis(void)
{
    s16 raw_z = ano_of.of_alt_cm;    /* 从光流传感器读取当前高度（厘米） */
    if (abs16(raw_z - last_z) > 10) raw_z = last_z; /* 高度跳变超过 10cm 则丢弃 */
    uwb_pos_info.now_z = raw_z;      /* 更新当前高度 */
    last_z = raw_z;                  /* 记录本次高度作为基准 */
}

/**
 * @brief  计算归一化角度误差（度），结果在 [-180, 180] 范围内
 * @param  current_deg  当前角度（度）
 * @param  target_deg   目标角度（度）
 * @author camelliahm
 * @return 角度误差（度），正值表示需要顺时针旋转（取决于你的坐标系正方向）
 *         如果返回 0，表示已对齐（或无需控制）
 */
static float Angle_Error_Normalize(float current_deg, float target_deg)
{
    float error = target_deg - current_deg;

    // 归一化到 [-180, 180]
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;
}

/**
 * @brief  世界坐标系速度指令 → 机体坐标系速度指令
 * @param  yaw_rad  当前偏航角（弧度）
 * @param  wx, wy   世界坐标系下的 X/Y 控制量
 * @param  bx, by   输出机体坐标系下的 X/Y 控制量（指针传回）
 * @author camelliahm
 */
static void WorldToBody(float yaw_rad, float wx, float wy, float *bx, float *by)
{
    float c = cosf(yaw_rad);
    float s = sinf(yaw_rad);
    *bx =  c * wx - s * wy;
    *by =  s * wx + c * wy;
}

#if DAMPING_CONTROL

/**
 * @brief 阻尼模式下的主动位置保持控制
 * 
 * 本质是一个以起飞点为原点、仅依赖光流速度积分的简易 P + D 位置环：
 *   - 目标值恒为 0（即起飞原点），
 *   - 当前位置由光流速度积分得到，
 *   - P 项（回中力） = 比例增益 × 位移（带 ±3 cm 死区，防止小偏差抖动），
 *   - D 项（速度阻尼） = 微分增益 × 光流速度（刹车，防止回中过冲）。
 * 
 * 该函数在 PID 未使能（如 UWB 未锁定）或起飞阶段时被调用，
 * 用于抵抗外部扰动、抑制漂移，并为后续切入完整位置环提供平滑过渡。
 * 
 * @author camelliahm
 */
static void Damping_Control(void)
{
    static float filt_out_x = 0, filt_out_y = 0; /* 滤波后的速度值，增加阻尼控制的平滑性 */
    static float integral_x = 0, integral_y = 0; /* 积分项，用于计算相对位移 */

    // 1. 获取光流速度 (惯导融合后，噪声小)
    float spd_x = ano_of.of2_dx_fix;   /* 获取光流 X 方向速度 */
    float spd_y = ano_of.of2_dy_fix;   /* 获取光流 Y 方向速度 */

    // 2. 速度积分，计算相对位移 (单位: cm)
    // 0.02f 是控制周期 (50Hz)，表示每20ms积分一次
    integral_x += spd_x * 0.02f; /* 积分计算相对位移 */
    integral_y += spd_y * 0.02f; /* 积分计算相对位移 */

    // 3. 积分限幅，防止积分饱和 (限制回中力最大作用范围)
    integral_x = CLAMP(integral_x, -20, 20); /* 积分限幅 ±20cm */
    integral_y = CLAMP(integral_y, -20, 20); /* 积分限幅 ±20cm */

    // 4. 积分死区：位移很小时不产生回中力，避免在原点附近往复震荡
    float out_x = 0.0f, out_y = 0.0f;
    if (fabsf(integral_x) > 3.0f)
    {
        out_x = -0.6f * integral_x; /* 当位移超过 3cm 时，产生回中力 */
    }
    if (fabsf(integral_y) > 3.0f)
    {
        out_y = -0.6f * integral_y; /* 当位移超过 3cm 时，产生回中力 */
    }

    // 5. 叠加被动速度阻尼，提供“刹车”效果
    out_x -= 1.2f * spd_x;
    out_y -= 1.2f * spd_y;

    // 6. 对最终输出进行滤波和限幅，确保控制量平滑且不超限
    filt_out_x = 0.3f * out_x + 0.7f * filt_out_x;
    filt_out_y = 0.3f * out_y + 0.7f * filt_out_y;

    filt_out_x = CLAMP(filt_out_x, -12.0f, 12.0f);      /* X 阻尼输出限幅 ±12 */
    filt_out_y = CLAMP(filt_out_y, -12.0f, 12.0f);      /* Y 阻尼输出限幅 ±12 */

    // 7. 输出并清理其他轴
    PID_ctrl[0] = (s16)filt_out_x;            /* 将阻尼量写入 X 速度输出 */
    PID_ctrl[1] = (s16)filt_out_y;            /* 将阻尼量写入 Y 速度输出 */
    //PID_ctrl[2] = 0;                      /* Z 输出置零 */
    PID_ctrl[3] = 0;                      /* 航向输出置零 */
    pid_x.Iout = pid_y.Iout = pid_z.Iout = 0; /* 清除各 PID 积分项，避免切换时跳变 */
}

#endif

#if ADJUST_DYNAMIC_PARAMS

/**
 * @brief 根据运动状态动态调整 PID 参数
 *        当某一轴正在运动时，增大该轴微分增益以增加阻尼，减小积分增益防止积分饱和
 * @author camelliahm
 */
static void Adjust_Dynamic_Params(void)
{
    if (uwb_pos_info.is_moving_y) 
    {                           /* Y 轴正在运动 */
        pid_x.Kd = X_KD * DYN_KD_BOOST;         /* 增大 X 轴 PID 的微分项 */
        pid_x.Ki = X_KI * DYN_KI_REDUCE;        /* 减小 X 轴 PID 的积分项 */
    } 
    else 
    {
        pid_x.Kd = X_KD;                        /* 恢复默认微分 */
        pid_x.Ki = X_KI;                        /* 恢复默认积分 */
    }

    if (uwb_pos_info.is_moving_x) 
    {                           /* X 轴正在运动 */
        pid_y.Kd = Y_KD * DYN_KD_BOOST;         /* 增大 Y 轴 PID 的微分项 */
        pid_y.Ki = Y_KI * DYN_KI_REDUCE;        /* 减小 Y 轴 PID 的积分项 */
    } 
    else 
    {
        pid_y.Kd = Y_KD;                        /* 恢复默认微分 */
        pid_y.Ki = Y_KI;                        /* 恢复默认积分 */
    }
}

#endif

/**
 * @brief 初始化所有 PID 控制器及相关变量
 *        配置死区、滤波、积分限幅等高级特性
 * @author camelliahm
 */
void Drone_PID_Init(void)
{
    // 初始化 X 轴 PID：启用积分限幅、变积分、微分滤波、输出滤波
    PID_Init(&pid_x, X_MAX_OUT, X_INT_LIM, X_DEADBAND,
             X_KP, X_KI, X_KD,
             9.0f, 3.0f, 0.3f, 0.3f,            /* 变积分参数(A,B)、微分滤波器因子、输出滤波器因子 */
              DerivativeFilter | Fallback_Diff);

    // 初始化 Y 轴 PID：配置与 X 轴相同
    PID_Init(&pid_y, Y_MAX_OUT, Y_INT_LIM, Y_DEADBAND,
             Y_KP, Y_KI, Y_KD,
             9.0f, 3.0f, 0.3f, 0.3f,
              DerivativeFilter| Fallback_Diff);

    // 初始化 Z 轴 PID：死区、限幅与 X/Y 不同，使用高度控制参数
    PID_Init(&pid_z, Z_MAX_OUT, Z_INT_LIM, Z_DEADBAND,
             Z_KP, Z_KI, Z_KD,
             15.0f, 5.0f, 0.2f, 0.2f,
             Integral_Limit | ChangingIntegralRate | DerivativeFilter | OutputFilter | Fallback_Diff);

    // 初始化航向 PID：只有 PD 控制，无积分项，死区与限幅为角度值
    PID_Init(&pid_yaw, YAW_MAX_OUT, 0, YAW_DEADBAND,
             YAW_KP, 0, YAW_KD,
             0, 0, 0.3f, 0.3f,
             OutputFilter);                     /* 仅使能输出滤波 */

    memset(&uwb_pos_info, 0, sizeof(uwb_pos_info)); /* 清空 UWB 信息结构体 */
    valid_frame_count = 0;                       /* 复位有效帧计数器 */
    last_x = last_y = last_z = 0;                /* 复位历史坐标 */
    uwb_pos_info.pid_enable_flag = 0;                         /* 默认不使能 PID */
    uwb_pos_info.yaw_lock = 0;                                /* 默认不锁定航向 */
}

/**
 * @brief 起飞控制函数，设定目标高度并立刻开始 Z 轴控制
 * @author camelliahm
 * @param target_height 目标高度（厘米）
 */
void Take_Off(u32 target_height)
{
    uwb_pos_info.uwb_target_z = target_height;   /* 更新目标高度 */
    Get_Gen_Dis();                               /* 获取当前高度 */
    pid_z.Target = target_height;                /* 设置 Z 轴 PID 目标 */
    PID_ctrl[2] = (s16)PID_Calculate(&pid_z, uwb_pos_info.now_z, target_height); /* 计算并输出 Z 轴控制量 */
}

#if USE_LEGACY_CONTROL

/**
 * @brief UWB 位置 PID 主控制循环
 *        读取传感器数据，根据使能状态执行阻尼或位置闭环控制
 *       包含坐标系转换、动态参数调整、异常数据处理等功能
 * @author camelliahm
 */
void UWB_Position_PID(void)
{
    Get_UWB_Pos();                               /* 获取并处理 UWB 数据 */
    Get_Gen_Dis();                               /* 获取并处理高度数据 */

    if (uwb_pos_info.pid_enable_flag == 0)  /* PID 未使能 */
    {                 
        Damping_Control();                       /* 进入阻尼模式 */
        return;
    }

    if (uwb_pos_info.target_init_done == 0) 
    {    /* 目标尚未初始化完成 */
        PID_ctrl[0] = PID_ctrl[1] = PID_ctrl[2] = PID_ctrl[3] = 0; /* 输出全零 */
        return;
    }

    // 根据当前速度状态调整 PID 参数
    Adjust_Dynamic_Params();

    // 航向外环控制（锁定时计算航向误差）
    if (uwb_pos_info.yaw_lock) 
    {
        float yaw_err = uwb_pos_info.target_yaw_deg - imu_angle * 57.2958f; /* 计算航向偏差（度） */
        while (yaw_err > 180) yaw_err -= 360;    /* 将偏差归一化到 -180~180 度 */
        while (yaw_err < -180) yaw_err += 360;
        PID_ctrl[3] = - (s16)PID_Calculate(&pid_yaw, 0, yaw_err); /* 航向 PID 计算并输出角速度 */
    } 
    else 
    {
        PID_ctrl[3] = 0;                         /* 不锁定时航向输出为零 */
    }

    // Z 轴高度控制
    pid_z.Target = uwb_pos_info.uwb_target_z;    /* 设定 Z 轴目标高度 */
    PID_ctrl[2] = (s16)PID_Calculate(&pid_z, uwb_pos_info.now_z, uwb_pos_info.uwb_target_z); /* 计算高度控制输出 */

    // XY 轴位置控制（世界坐标系下计算）
    float out_x_w = PID_Calculate(&pid_x, uwb_pos_info.now_x, uwb_pos_info.uwb_target_x); /* X 轴世界坐标输出 */
    float out_y_w = PID_Calculate(&pid_y, uwb_pos_info.now_y, uwb_pos_info.uwb_target_y); /* Y 轴世界坐标输出 */

    // 坐标系旋转：将世界坐标系下的输出转换到机体坐标系
    float yaw = imu_angle;                       /* 获取当前偏航角（弧度） */
    float c = cosf(yaw), s = sinf(yaw);          /* 计算旋转矩阵系数 */
    float out_x_body =  c * out_x_w + s * out_y_w; /* 旋转得到机体 X 方向输出 */
    float out_y_body = -s * out_x_w + c * out_y_w; /* 旋转得到机体 Y 方向输出 */

    out_x_body = CLAMP(out_x_body, -MAX_SPEED, MAX_SPEED); /* 限幅 X 输出 */
    out_y_body = CLAMP(out_y_body, -MAX_SPEED, MAX_SPEED); /* 限幅 Y 输出 */

    if (UWB.uwb_work_sta == 1 && UWB.uwb_link_sta == 1) /* UWB 工作正常且链路正常 */
    { 
        PID_ctrl[0] = (s16)out_x_body;           /* 写入 X 速度指令 */
        PID_ctrl[1] = (s16)out_y_body;           /* 写入 Y 速度指令 */
    } else {                                     /* UWB 异常时停止 XY 输出并清积分 */
        PID_ctrl[0] = PID_ctrl[1] = 0;
        pid_x.Iout = pid_y.Iout = 0;
    }
}

#else

/**
 * @brief  无人机多轴位置/姿态控制更新
 * @param  axis_mask  按位掩码，选择需要控制的轴 (DroneAxisMask_t 组合)
 * @param  smooth_mode 平滑模式选择
 * @note   调用前需确保 uwb_pos_info 中的 now_x/now_y/now_z 已经是滤波后的值，
 *         uwb_target_x/y/z 已经设定好。
 *         该函数仅负责 PID 计算、坐标系旋转和限幅。
 * @author camelliehm
 */
void Drone_Control_Update(uint8_t axis_mask, uint8_t smooth_mode)
{
    Get_UWB_Pos();                               /* 获取并处理 UWB 数据 */
    Get_Gen_Dis();                               /* 获取并处理高度数据 */

    if (uwb_pos_info.now_z < 108 || uwb_pos_info.now_z > 115)
    {
        Damping_Control();                       /* 高度在 108~115cm 时启用阻尼控制 */
        return;
    }

    Adjust_Dynamic_Params();

    float yaw_rad = imu_angle - 1.5708f;  // - π/2
    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);

    /* ---------- 水平轴 (X+Y) ---------- */
    if (axis_mask & (DRONE_AXIS_X | DRONE_AXIS_Y))
    {
        if (smooth_target.smoothing_active)
        {
            Update_Smooth_Target(smooth_mode);
            uwb_pos_info.uwb_target_x = (s16)smooth_target.smooth_x;
            uwb_pos_info.uwb_target_y = (s16)smooth_target.smooth_y;
        }
        float out_x_world = (axis_mask & DRONE_AXIS_X) ? 
            PID_Calculate(&pid_x, uwb_pos_info.now_x, uwb_pos_info.uwb_target_x) : 0.0f;
        float out_y_world = (axis_mask & DRONE_AXIS_Y) ? 
            PID_Calculate(&pid_y, uwb_pos_info.now_y, uwb_pos_info.uwb_target_y) : 0.0f;

        // 世界 → 机体
        float out_x_body;
        float out_y_body;

        WorldToBody(yaw_rad, out_x_world, out_y_world, &out_x_body, &out_y_body);

        // 限幅
        out_x_body = CLAMP(out_x_body, -X_MAX_OUT, X_MAX_OUT);
        out_y_body = CLAMP(out_y_body, -Y_MAX_OUT, Y_MAX_OUT);

        PID_ctrl[0] = (s16)out_x_body;
        PID_ctrl[1] = (s16)out_y_body;
    }
    else
    {
        PID_ctrl[0] = 0;
        PID_ctrl[1] = 0;
        pid_x.Iout = 0;
        pid_y.Iout = 0;
    }

    /* ---------- 高度轴 ---------- */
    if (axis_mask & DRONE_AXIS_Z)
    {
        PID_ctrl[2] = (s16)PID_Calculate(&pid_z, uwb_pos_info.now_z, uwb_pos_info.uwb_target_z);
        PID_ctrl[2] = CLAMP(PID_ctrl[2], -Z_MAX_OUT, Z_MAX_OUT);
    }
    else
    {
        PID_ctrl[2] = 0;
        pid_z.Iout = 0;
    }

    /* ---------- 偏航轴 ---------- */
    if (axis_mask & DRONE_AXIS_YAW)
    {
        if (uwb_pos_info.yaw_lock)
        {
            float current_yaw_deg = imu_angle * 57.2958f;
            float target_yaw_deg  = uwb_pos_info.target_yaw_deg;     // 读取锁定的目标角度
            float yaw_err = Angle_Error_Normalize(current_yaw_deg, target_yaw_deg);
            PID_ctrl[3] = -(s16)PID_Calculate(&pid_yaw, 0, yaw_err);
            PID_ctrl[3] = CLAMP(PID_ctrl[3], -YAW_MAX_OUT, YAW_MAX_OUT);
        }
        else
        {
            PID_ctrl[3] = 0;
        }
    }
    else
    {
        PID_ctrl[3] = 0;
        pid_yaw.Iout = 0;
    }
}

#endif
