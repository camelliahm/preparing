#ifndef __UWB_LOCATION_H
#define __UWB_LOCATION_H
#include "stm32f4xx.h"

// 定义UWB数据结构体（存储定位结果+状态）
typedef struct
{
	u8 uwb_update_cnt;       // UWB数据更新次数（判断数据是否刷新）
	u8 uwb_link_sta;         // 连接状态：0=未连接 1=已连接
	u8 uwb_work_sta;         // 工作状态：0=数据异常 1=正常定位
	u8 dubug ;
	s16 pose_x_mm;           // 解析后X坐标（原始单位：毫米）
	s16 pose_y_mm;           // 解析后Y坐标（原始单位：毫米）
	s16 pose_x_cm;           // 解析后X坐标（转换单位：厘米）
	s16 pose_y_cm;           // 解析后Y坐标（转换单位：厘米）
	s16 uwb_x_cm;			 // UWB模块X坐标（厘米）
	s16 uwb_y_cm;			 // UWB模块Y坐标（厘米）
	s16 uwb_Offset_Pos_x_cm; // UWB偏移量X（厘米）
	s16 uwb_Offset_Pos_y_cm; // UWB偏移量Y（厘米）
	u16 uwb_time;            // 超时计时（判断数据丢失）
}UWB_info;

// 二维卡尔曼滤波结构体（恒定速度模型）
typedef struct
{
    float x;  // X轴位置
    float v;  // 速度
    float P[2][2];  // 估计协方差矩阵
    float Q;       // 过程噪声系数
    float R;       // 观测噪声系数
} KalmanFilter1D;

// 协议固定宏定义
#define UWB_MODBUS_ID       0x01
#define UWB_FUNC_CODE       0x03
#define UWB_TAG_FLAG        0xACDA
#define JUMP_LIMIT_MM       500
#define FRAME_LEN_51        51
#define UWB_DATA_LEN        46  // 51字节帧的数据段长度

// 声明全局变量（外部文件可调用）
extern UWB_info UWB;
extern KalmanFilter1D kalman_x;
extern KalmanFilter1D kalman_y;
extern float UWB_Data[2];
extern float First_UWB_Data[2];

// 声明函数（主函数/串口中断/调度器调用）
void UWB_Init(void);// 初始化函数，配置UWB状态结构体和卡尔曼滤波器
void UWB_Getdata(uint8_t data);// 数据接收与解析核心函数，串口每接收1字节调用1次
void UWB_Timeout_Process(void);// 超时检测函数，调度器1ms调用一次，判断UWB是否断开连接
void Kalman1D_Init(KalmanFilter1D *kf, float q, float r);// 一维卡尔曼滤波器初始化函数，配置初始状态和噪声参数
void Kalman1D_Update(KalmanFilter1D *kf, float meas, float dt, float *out_pos, float *out_vel);// 一维卡尔曼滤波器更新函数，处理每次新的UWB测量值，返回滤波后的坐标值
void UWB_CalculateOffset(void);// 计算坐标偏移量函数，根据当前坐标和初始坐标计算偏移量，单位转换为厘米

#endif
