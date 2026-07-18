#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "UWB_location.h"
#include "PID_ctrl.h"
#include "PID_Mod.h"
#include "math.h"
#include "Ano_Math.h"
#include "Drv_AnoOf.h"
#include "My_smooth.h"
#include "Patrol_Task.h"
#include "My_Comm.h"

// 静态变量记录当前所处阶段：0=官方起飞阶段，1=高度环阶段
static uint8_t alt_stage = 0;
static uint16_t stable_count = 0;   // 高度稳定计数
static Cargo patrol_path[20];      // 定向盘点子路径数组
static uint8_t patrol_path_len;    // 定向盘点路径长度、
static u8 recognized_id; // 识别到的目标货物编号
static u8 Fly = 0;
static u8 flag = 0;

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
    static u32 time_dly_cnt_ms;

    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
        //判断第6通道拨杆位置 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            //还没有执行
            if (one_key_takeoff_f == 0)
            {
                //标记已经执行
                one_key_takeoff_f =
                    //执行一键起飞
                    OneKey_Takeoff(130); //参数单位：厘米； 0：默认上位机设置的高度。
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_takeoff_f = 0;
        }
        //
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //还没有执行
            if (one_key_land_f == 0)
            {
                //标记已经执行
                one_key_land_f =
                    //执行一键降落
                    OneKey_Land();
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
        //判断第6通道拨杆位置 1700<CH_6<2000
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //还没有执行
            if (one_key_mission_f == 0)
            {
                //标记已经执行
                one_key_mission_f = 1;
                //开始流程
                mission_step = 1;
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_mission_f = 0;
        }
        //
        if (one_key_mission_f == 1)
        {
            switch (mission_step)
            {
                case 0:
                {
                    time_dly_cnt_ms = 0;
                } 
                break;
                case 1:
                {
                    mission_step += LX_Change_Mode(2);
                }
                break;
                case 2:
                {
                    if(time_dly_cnt_ms<5000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
                }
                break;
                case 3:
                {
                    mission_step += FC_Unlock();
                }
                break;
                case 4:
                {
                    Take_Off(110);
                    //OneKey_Takeoff(110);
                    if (ano_of.of_alt_cm > 108 && ano_of.of_alt_cm < 115)
                    {
                        // mission_step += 1;
                        uwb_pos_info.axis_mask = DRONE_AXIS_ALL; // 全轴控制

                        if (time_dly_cnt_ms<100000)//！！！！！！！！！！！！！！超大延时（仅调试使用）
                        {
                            time_dly_cnt_ms+=20;//ms
                        }
                        else
                        {
                            time_dly_cnt_ms = 0;
                            if (ground_data.mission == 1)
                            {
                                mission_step = 5;
                            }
                            else if(ground_data.mission == 2)
                            {
                                mission_step = 6;
                            }
                        }
                    }
                }
                break;
                case 5:
                {
                    if (Patrol_Task(Goods_list, 36, 5000))
                    {
                        mission_step = 8;
                    }
                }
                break;
                
                case 6:
                {
                    if (DirectedPath_map(vision_data.number, &recognized_id))
                    {
                        patrol_path_len = DirectedPath_Build(recognized_id, patrol_path);
                        mission_step = 7;
                    }
                }
                break;
                case 7:
                {
                    if (Patrol_Task(patrol_path, patrol_path_len, 5000))
                    {
                        mission_step = 8;
                    }
                }
                break;
                case 8:
                {
                    mission_step += OneKey_Land();
                }
                break;
                case 9:
                {
                    mission_step += FC_Lock();
                }
                break;
                default:
                break;
            }
        }
        else
        {
            mission_step = 0;
        }
    }
    ////////////////////////////////////////////////////////////////////////
}

void MyTask(void)
{
    static u8 inited = 0;

    if (rc_in.fail_safe == 0)
    {
        /* ---------- 上电仅一次初始化 ---------- */
        if (inited == 0)
        {
            inited = 1;
            // PID 对象, 输出限幅, 积分限幅, 死区, Kp, Ki, Kd, 变积分A, 变积分B, 输出滤波, 微分滤波, 功能按位组合
            PID_Init(&pid_x , 20 , 8 , 2.0f , 0.6f , 0.0025f , 0.06f , 9.0f , 3.0f , 0.3f , 0.3f , Integral_Limit | ChangingIntegralRate | DerivativeFilter | OutputFilter | Fallback_Diff | Derivative_On_Measurement);
            PID_Init(&pid_y , 20 , 8 , 2.0f , 0.6f , 0.0025f , 0.06f , 9.0f , 3.0f , 0.3f , 0.3f , Integral_Limit | ChangingIntegralRate | DerivativeFilter | OutputFilter | Fallback_Diff | Derivative_On_Measurement);
            PID_Init(&pid_yaw, 15.0f , 0 , 3 , 0.50f , 0 , 0.50f , 0 , 0 , 0.3f , 0.3f , OutputFilter);
            PID_Init(&pid_z,   40 , 10 , 2.0f , 0.80f , 0.002f , 0.03f , 15 , 5 , 0.2f , 0.2f , Integral_Limit | ChangingIntegralRate | DerivativeFilter | OutputFilter | Fallback_Diff);
        }

        /* ---------- 读取传感器（带跳变保护） ---------- */
        Get_UWB_Pos();
        Get_Gen_Dis();

        /* ---------- 根据通道6设定轴掩码 ---------- */
        uint16_t ch6 = rc_in.rc_ch.st_data.ch_[ch_6_aux2];

        // 仅Z轴PID（1300~1700）
        if (ch6 > 1300 && ch6 < 1700)
        {
            uwb_pos_info.axis_mask = DRONE_AXIS_Z;   // 只控制高度
        }
        // 全轴PID（1700~2200）
        else if (ch6 > 1700 && ch6 < 2200)
        {
            uwb_pos_info.axis_mask = DRONE_AXIS_ALL; // 全部轴
        }
        // 其他情况（通道值不在有效范围）
        else
        {
            // 降落
            uwb_pos_info.axis_mask = DRONE_AXIS_Z;   // 只控制高度
            uwb_pos_info.uwb_target_z = 0;
            return;
        }

        /* ---------- 起飞阶段处理（复用 alt_stage 状态机） ---------- */
        if (alt_stage == 0)
        {
            // 调用官方起飞函数
            //OneKey_Takeoff(110);
            Take_Off(110);
            // 检测高度是否稳定在目标附近
            if (ano_of.of_alt_cm > 108 && ano_of.of_alt_cm < 115)
            {
                stable_count++;
            }
            else
            {
                stable_count = 0;
            }

            if (stable_count > 25)   // 持续约0.5秒稳定
            {
                // 初始化 pid_z 状态
                pid_z.Iout     = 0.0f;
                pid_z.ITerm    = 0.0f;
                pid_z.Last_Err = 0.0f;
                pid_z.Target   = 110.0f;

                // 初始化平滑起点为当前 UWB 坐标
                smooth_target.smooth_x = uwb_pos_info.now_x;
                smooth_target.smooth_y = uwb_pos_info.now_y;
                Set_Smooth_Target(uwb_pos_info.uwb_target_x, uwb_pos_info.uwb_target_y, 0.0f, 0);

                // 锁定航向
                uwb_pos_info.yaw_lock = 1;
                uwb_pos_info.target_yaw_deg= imu_angle * 57.2958f;

                alt_stage = 1;
                stable_count = 0;
            }
        }
        else   // alt_stage == 1，正常控制阶段
        {
            // 设置高度目标（全轴和仅Z轴模式都使用）
            uwb_pos_info.uwb_target_z = 130;

            if (Fly == 0)
            {
                uwb_pos_info.uwb_target_x += 200;
                Fly = 1;
            }

            if (ABS(uwb_pos_info.uwb_target_x - uwb_pos_info.now_x) <= 5 && ABS(uwb_pos_info.uwb_target_y - uwb_pos_info.now_y) <= 5 && flag == 0)
            {
                uwb_pos_info.axis_mask = DRONE_AXIS_ALL;
                uwb_pos_info.target_yaw_deg = 90.0f;
                flag = 1;
            }

            if (my_abs(uwb_pos_info.target_yaw_deg - imu_angle * 57.2958f) <= 1 && flag == 1)
            {
                uwb_pos_info.axis_mask = DRONE_AXIS_ALL;
                uwb_pos_info.uwb_target_y -= 200;
                flag = 2;
            }

            // 调用统一控制更新函数
            //Drone_Control_Update(uwb_pos_info.axis_mask, SMOOTH_NONE);
        }
    }
}
