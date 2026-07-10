#include "UWB_location.h"
#include "Ano_Math.h"
#include "stm32f4xx.h"
#include "string.h"
#include "math.h"
#include "Drv_Sys.h"


// 定义一维卡尔曼滤波器结构体变量，用于对UWB坐标进行滤波处理
KalmanFilter1D kalman_x;
KalmanFilter1D kalman_y;
// 定义全局UWB状态结构体变量，存储UWB的坐标、状态等核心数据
UWB_info UWB;
// 浮点型数组，存储UWB解析后的原始XY坐标（单位：mm）
float UWB_Data[2];
// 浮点型数组，存储上电时的初始坐标，用于坐标归零/原点校准
float First_UWB_Data[2];
// 静态浮点型数组，存储上一帧的坐标数据，用于判断坐标是否异常跳变
static float UWB_data_old[2];
// 静态字节变量，初始坐标校准标志位，前5帧数据完成自动归零
static u8 init_flag = 0;

/**
 * @brief  UWB模块初始化函数
 * @note   该函数在系统初始化阶段调用，负责将UWB状态结构体内存清零，并初始化坐标数据
 * @author Camellia
 * @param  无
 * @retval 无
 */
void UWB_Init(void)
{
    // 将UWB状态结构体内存清零，初始化所有参数
    memset(&UWB, 0, sizeof(UWB_info));
    // 初始化原始坐标为0
    UWB_Data[0] = 0.0f;
    UWB_Data[1] = 0.0f;
    // 初始化初始坐标为0
    First_UWB_Data[0] = 0.0f;
    First_UWB_Data[1] = 0.0f;
    // 初始化上一帧坐标为0
    UWB_data_old[0] = 0.0f;
    UWB_data_old[1] = 0.0f;
    // 初始化校准标志位为0
    init_flag = 0;

    Kalman1D_Init(&kalman_x, 5.0f, 0.3f);
    Kalman1D_Init(&kalman_y, 5.0f, 0.3f);
}

/**
 * @brief  Modbus协议CRC16校验函数
 * @note   该函数采用常见的CRC16算法，适用于Modbus RTU协议的数据校验；输入数据和长度，输出16位CRC校验值
 * @author Camellia
 * @param  data: 待校验的数据首地址
 * @param  len:  待校验的数据长度
 * @retval 计算得到的CRC16校验值
 */
u16 CRC16_Check(u8 *data, u16 len)
{
    u16 crc = 0xFFFF;  // CRC16初始值固定为0xFFFF
    u16 i;             // 外层循环变量，遍历数据字节
    u8 j;              // 内层循环变量，遍历字节的每一位
    // 遍历所有待校验数据
    for(i = 0; i < len; i++)
    {
        crc ^= data[i];  // 当前字节与CRC值异或运算
        // 对每个字节的8位依次处理
        for(j = 0; j < 8; j++)
        {
            // 判断最低位是否为1，执行移位/多项式异或
            crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;  // 返回最终计算的CRC校验值
}

/**
 * @brief  计算UWB坐标偏移量函数
 * @note   计算当前滤波后坐标相对于上电初始坐标的偏移量，单位为厘米
 * @author Camellia
 * @param  无
 * @retval 无
 */
void UWB_CalculateOffset(void)
{
    // 前5帧未完成初始坐标采集时，偏移量保持为0
    if (init_flag >= 5)
    {
        // 计算X轴偏移量（mm转cm）
        UWB.uwb_Offset_Pos_x_cm = (s16)((UWB_Data[0] - First_UWB_Data[0]) / 10.0f);
        // 计算Y轴偏移量（mm转cm）
        UWB.uwb_Offset_Pos_y_cm = (s16)((UWB_Data[1] - First_UWB_Data[1]) / 10.0f);
    }
    else
    {
        UWB.uwb_Offset_Pos_x_cm = 0;
        UWB.uwb_Offset_Pos_y_cm = 0;
    }
}

/**
 * @brief  一维卡尔曼滤波器初始化
 * @author Camellia
 * @param  kf : 滤波器实例指针
 * @param  Q  : 过程噪声系数（建议 0.01~0.1 平滑，3.0~5.0 快速响应）
 * @param  R  : 观测噪声系数（建议 0.1~1.0 信任测量，3.0 更平滑）
 */
void Kalman1D_Init(KalmanFilter1D *kf, float Q, float R)
{
    kf->x = 0.0f;
    kf->v = 0.0f;
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 1.0f;
    kf->Q = Q;
    kf->R = R;
}

/**
 * @brief  一维卡尔曼滤波器更新（恒定速度模型）
 * @author Camellia
 * @param  kf      : 滤波器实例指针
 * @param  meas    : 测量值（位置，单位与状态一致）
 * @param  dt      : 时间间隔（单位：秒）
 * @param  out_pos : 输出滤波后位置（可 NULL）
 * @param  out_vel : 输出滤波后速度（可 NULL）
 */
void Kalman1D_Update(KalmanFilter1D *kf, float meas, float dt, float *out_pos, float *out_vel)
{
    // 防止非法时间间隔
    if (dt <= 0.0f) dt = 0.01f;
    if (dt > 0.5f) dt = 0.5f;

    // 1. 预测步
    float x_pre = kf->x + kf->v * dt;
    float v_pre = kf->v;

    // 预测协方差矩阵更新
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    kf->P[0][0] += 2.0f * dt * kf->P[0][1] + dt2 * kf->P[1][1] + kf->Q * dt4 / 4.0f;
    kf->P[0][1] += dt * kf->P[1][1] + kf->Q * dt3 / 2.0f;
    kf->P[1][0] = kf->P[0][1];   // 对称性
    kf->P[1][1] += kf->Q * dt2;

    // 2. 更新步
    float S = kf->P[0][0] + kf->R;
    float K0 = kf->P[0][0] / S;   // 位置修正增益
    float K1 = kf->P[1][0] / S;   // 速度修正增益

    float err = meas - x_pre;
    kf->x = x_pre + K0 * err;
    kf->v = v_pre + K1 * err;

    // 更新协方差矩阵
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    kf->P[0][0] = (1.0f - K0) * P00;
    kf->P[0][1] = (1.0f - K0) * P01;
    kf->P[1][0] = kf->P[0][1];
    kf->P[1][1] = kf->P[1][1] - K1 * P01;

    // 输出结果
    if (out_pos) *out_pos = kf->x;
    if (out_vel) *out_vel = kf->v;
}


/**
 * @brief  UWB数据接收与解析核心函数（采用Modbus可变帧思路）
 * @note   串口每接收1字节数据，调用1次该函数；接收完整帧后自动解析
 * @author Camellia
 * @param  data: 串口接收到的单字节数据
 * @retval 无
 */
void UWB_Getdata(uint8_t data)
{
    // 沿用的状态机变量定义
    static u8 rxstate = 0;
    static u8 buf[100];
    static u8 cnt = 0;
    static u8 payload_len = 0;
    static uint32_t last_valid_tick = 0;

    // =====================  Modbus 可变长度状态机解析逻辑 =====================
    // 状态0：匹配设备地址 0x01
    if (rxstate == 0 && data == 0x01)
    {
        rxstate = 1;
        buf[0] = data;
    }
    // 状态1：匹配功能码 0x03
    else if (rxstate == 1 && data == 0x03)
    {
        rxstate = 2;
        buf[1] = data;
    }
    // 状态2：读取数据长度
    else if (rxstate == 2)
    {
        payload_len = data;
        // 合法长度判断
        if (payload_len >= 4 && payload_len <= 90)
        {
            rxstate = 3;
            buf[2] = data;
            cnt = 3;
        }
        else
        {
            rxstate = 0;
        }
    }
    // 状态3：接收数据段
    else if (rxstate == 3)
    {
        buf[cnt++] = data;
        // 帧总长度 = 头部3字节 + 数据长度 + CRC2字节
        if (cnt >= (payload_len + 5))
        {
            // ===================== CRC 校验 =====================
            uint16_t calc_crc = CRC16_Check(buf, cnt - 2);
            uint16_t recv_crc = (buf[cnt - 1] << 8) | buf[cnt - 2];

            if (calc_crc == recv_crc)
            {
                // 匹配数据头标识 0xAC 0xDA
                if (buf[3] == 0xAC && buf[4] == 0xDA)
                {
                    u16 output_mask = (buf[5] << 8) | buf[6];
                    u16 read_ptr = 9;

                    // 跳过掩码数据
                    if (output_mask & 0x01)
                    {
                        read_ptr += 32;
                    }

                    // 提取定位状态
                    u16 loc_flag = (buf[read_ptr] << 8) | buf[read_ptr + 1];
                    read_ptr += 2;

                    if (loc_flag != 0)
                    {
                        // 1. 提取原始坐标 (cm单位)
                        int16_t raw_x_cm = (int16_t)((buf[read_ptr] << 8) | buf[read_ptr + 1]);
                        int16_t raw_y_cm = (int16_t)((buf[read_ptr + 2] << 8) | buf[read_ptr + 3]);
                        UWB.uwb_x_cm = raw_x_cm;
                        UWB.uwb_y_cm = raw_y_cm;

                        UWB.uwb_x_cm = (s16)UWB.uwb_x_cm;
                        UWB.uwb_y_cm = (s16)UWB.uwb_y_cm;

                        // 2. 单位转换 cm → mm
                        float raw_x_mm = (float)raw_x_cm * 10.0f;
                        float raw_y_mm = (float)raw_y_cm * 10.0f;

                        u32 now = GetSysRunTimeMs();
                        float dt;
                        if (now >= last_valid_tick)
                        {
                            dt = 0.05f; // 默认50ms
                        }
                        else
                        {
                            dt = (now - last_valid_tick) / 1000.0f; // 转换为秒
                            if (dt <= 0.0f || dt > 0.5f) dt = 0.05f; // 防止异常时间间隔
                        }
                        last_valid_tick = now;

                        // 3. 二维卡尔曼滤波处理
                        Kalman1D_Update(&kalman_x, raw_x_mm, 0.01 , &UWB_Data[0], NULL);
                        Kalman1D_Update(&kalman_y, raw_y_mm, 0.01 , &UWB_Data[1], NULL);

                        //4. 前5帧自动归零
                        if (init_flag < 5)
                       {
                            First_UWB_Data[0] = UWB_Data[0];
                            First_UWB_Data[1] = UWB_Data[1];
                            init_flag++;
                        }
                        // 5. 计算正确的坐标偏移量（单位：cm）= 当前绝对坐标 - 上电初始坐标
                        UWB.uwb_Offset_Pos_x_cm = (s16)((UWB_Data[0] - First_UWB_Data[0]) / 10.0f);
                        UWB.uwb_Offset_Pos_y_cm = (s16)((UWB_Data[1] - First_UWB_Data[1]) / 10.0f);


                        // 6. 赋值给全局UWB结构体
                        UWB.pose_x_mm = (s16)UWB_Data[0];
                        UWB.pose_y_mm = (s16)UWB_Data[1];
                        UWB.pose_x_cm = UWB.pose_x_mm / 10;
                        UWB.pose_y_cm = UWB.pose_y_mm / 10;

                        // 7. 更新通信状态
                        UWB.uwb_link_sta = 1;
                        UWB.uwb_time = 0;
                        UWB.uwb_update_cnt++;
                        UWB.dubug = 1;

                        // 8. 坐标跳变判断
                        if (ABS(UWB_Data[0] - UWB_data_old[0]) > JUMP_LIMIT_MM
                            || ABS(UWB_Data[1] - UWB_data_old[1]) > JUMP_LIMIT_MM)
                        {
                            UWB.uwb_work_sta = 0;
                        }
                        else
                        {
                            UWB.uwb_work_sta = 1;
                        }

                        // 9. 保存上一帧数据
                        UWB_data_old[0] = UWB_Data[0];
                        UWB_data_old[1] = UWB_Data[1];
                    }
                    else
                    {
                        UWB.uwb_work_sta = 0;
                    }
                }
            }
            // 校验失败，重置状态机
            rxstate = 0;
        }
    }
    // 异常数据，重置状态机
    else
    {
        rxstate = 0;
    }
}


/**
 * @brief  UWB通信超时检测函数
 * @note   调度器1ms调用一次，用于判断UWB是否断开连接
 * @author Camellia
 * @param  无
 * @retval 无
 */
void UWB_Timeout_Process(void)
{
    // 超时计时小于500ms，计时自增
    if(UWB.uwb_time < 500)
    {
        UWB.uwb_time++;
    }
    // 超过500ms未收到数据，判定UWB断开
    else
    {
        UWB.uwb_link_sta = 0;  // 连接状态：断开
        UWB.uwb_work_sta = 0;   // 工作状态：异常
    }
}
