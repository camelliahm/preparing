#include "My_smooth.h"

SmoothTarget smooth_target = {0}; // 初始化平滑目标结构体

/* 恒定加速度减速参数 */
#define MAX_ACCEL          80.0f
#define MIN_CRUISE_SPEED   15.0f
/* 线性减速参数 */
#define LINEAR_SLOW_DOWN   30.0f
#define LINEAR_MIN_SPEED   10.0f

/**
 * @brief  设置最终目标点，并启动平滑移动，首次调用前设定为当前坐标
 * @param  new_x        最终目标X坐标 (cm)
 * @param  new_y        最终目标Y坐标 (cm)
 * @param  speed_cm_s   平滑移动速度 (cm/s)，例如 40.0f
 * @author camelliahm
 */
void Set_Smooth_Target(float new_x, float new_y, float speed_cm_s, uint8_t active)
{
    smooth_target.final_x = new_x;
    smooth_target.final_y = new_y;
    smooth_target.smooth_speed = speed_cm_s;
    smooth_target.smoothing_active = active;
}

/**
 * @brief  每控制周期调用一次，更新平滑目标值
 * @note   应在 MyTask 的全轴 PID 分支中、设置 uwb_target 之前调用
 * @author camelliahm
 */
void Update_Smooth_Target(u8 mode)
{
    if (!smooth_target.smoothing_active) 
    {
        smooth_target.smooth_x = smooth_target.final_x;
        smooth_target.smooth_y = smooth_target.final_y;
        return;
    }

    // 每周期步长 = 速度 (cm/s) × 控制周期 (s)
    const float dt = 0.02f;
    float current_speed = smooth_target.smooth_speed;

    float diff_x = smooth_target.final_x - smooth_target.smooth_x;
    float diff_y = smooth_target.final_y - smooth_target.smooth_y;
    float max_remaining = fmaxf(fabsf(diff_x), fabsf(diff_y));

    /* ----- 根据掩码选择减速算法 ----- */
    if (mode & SMOOTH_LINEAR_DEC) {
        // 线性减速
        if (max_remaining < LINEAR_SLOW_DOWN)
        {
            float ratio = max_remaining / LINEAR_SLOW_DOWN;
            current_speed = LINEAR_MIN_SPEED + 
            (smooth_target.smooth_speed - LINEAR_MIN_SPEED) * ratio;
        }
    }
    else if (mode & SMOOTH_CONST_ACCEL)
    {
        // 恒定加速度减速（抛物线速度曲线）
        float v_stop = sqrtf(2.0f * MAX_ACCEL * max_remaining);
        if (v_stop < MIN_CRUISE_SPEED) v_stop = MIN_CRUISE_SPEED;
        if (v_stop > smooth_target.smooth_speed) v_stop = smooth_target.smooth_speed;
        current_speed = v_stop;
    }

    float step = dt * current_speed;

    /* X 轴平滑 */
    if (fabsf(diff_x) >= step)
    {
        smooth_target.smooth_x += (diff_x > 0) ? step : -step; 
    }
    else
    {
        smooth_target.smooth_x = smooth_target.final_x;
    }

    /* Y 轴平滑 */
    if (fabsf(diff_y) >= step)
    {
        smooth_target.smooth_y += (diff_y > 0) ? step : -step;
    }
    else
    {
        smooth_target.smooth_y = smooth_target.final_y;
    }

    /* 检查是否已到达最终目标 */
    if (smooth_target.smooth_x == smooth_target.final_x && smooth_target.smooth_y == smooth_target.final_y)
    {
        smooth_target.smoothing_active = 0;
    }
}

/**
 * @brief  查询是否已到达最终目标
 * @return 1: 已到达, 0: 移动中或未激活
 * @author camelliahm
 */
uint8_t Is_Target_Reached(void)
{
    return !smooth_target.smoothing_active;
}
