#include "grid_coord_mapping.h"
#include "My_Comm.h"

GridCoord target_coord[MAX_PATH_LENGTH] = {0};

uint8_t error_flag_record_full = 0;   // 全局错误标志
static uint8_t wait_vision_flag = 0;        // 0=正常飞行，1=等待视觉数据
static uint8_t wait_vision_counter = 0;
#define WAIT_VISION_TIMEOUT  100   // 等待100个周期

// 静态变量（记录执行状态）
static uint16_t path_exec_index = 0;
static int8_t last_visited_row = -1;
static int8_t last_visited_col = -1;

// 初始化执行（开始巡查前调用一次）
void path_exec_init(void)
{
    path_exec_index = 0;
    last_visited_row = -1;
    last_visited_col = -1;
}

/*
@author: camelliahm
@brief:  执行路径点到目标坐标的转换
@param:  src  源路径点数组（Point类型）
@param:  dst  目标坐标数组（GridCoord类型）
@param:  len  路径点数量
*/
void convert_full_path_to_coords(const Point *src, GridCoord *dst, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) 
    {
        dst[i] = map_point_to_phys(src[i]);
    }
}

/*
@author: camelliahm
@brief:  将路径坐标转换为目标坐标
*/
void path_coord_to_target_coord (void)
{
    convert_full_path_to_coords(full_path, target_coord, full_path_length);
}

u8 is_position_reached(int16_t target_x, int16_t target_y, uint16_t threshold_cm)
{
    int16_t cur_x = uwb_pos_info.now_x; // 获取当前 X 坐标
    int16_t cur_y = uwb_pos_info.now_y; // 获取当前 Y 坐标
    uint16_t dist = sqrt(pow(cur_x-target_x,2) + pow(cur_y-target_y,2));
    return (dist < threshold_cm);
}

/*
@author: camelliahm
@brief:  执行路径点的巡查任务（每次调用处理一个点）
@return: uint8_t  0=继续执行，1=已完成所有点
*/
uint8_t path_exec_step(void)
{
    // ========== 1. 等待视觉数据状态 ==========
    if (wait_vision_flag) 
    {
        wait_vision_counter++;

        // 检查视觉数据是否有效（动物代号>0 且 数量>0）
        if (vision_data.animal_number > 0 && vision_data.animal_count > 0)
        {
            // 收到有效数据，记录到动物档案（使用当前方格坐标）
            if (!animal_record_add(last_visited_row, last_visited_col,vision_data.animal_number, vision_data.animal_count)) 
            {
                error_flag_record_full = 1;   // 存储失败（记录已满）
            }

            u8 data_buf[4];
            data_buf[0] = last_visited_row;
            data_buf[1] = last_visited_col;
            data_buf[2] = vision_data.animal_number;
            data_buf[3] = vision_data.animal_count;
            ANO_Send_Custom_F4(data_buf, 4); // 发送数据到地面站

            // 清空缓存，防止污染下一个方格
            vision_data.animal_number = 0;
            vision_data.animal_count = 0;

            wait_vision_flag = 0;          // 退出等待状态
            path_exec_index++;             // 推进到下一个目标点
            return (path_exec_index >= full_path_length) ? 1 : 0;
        }

        // 超时未收到有效数据，放弃该方格（不记录）
        if (wait_vision_counter >= WAIT_VISION_TIMEOUT) 
        {
            wait_vision_flag = 0;
            vision_data.animal_number = 0;
            vision_data.animal_count = 0;
            path_exec_index++;
            return (path_exec_index >= full_path_length) ? 1 : 0;
        }

        // 继续等待
        return 0;
    }


    //  判断是否所有点都已发送完毕
    if (path_exec_index >= full_path_length) 
    {
        return 1; // 完成
    }

    // 获取当前要去的目标点（物理坐标）
    Point current_logic = full_path[path_exec_index];
    GridCoord target = target_coord[path_exec_index];

    uwb_pos_info.uwb_target_x = target.x_coord; // 更新全局目标坐标
    uwb_pos_info.uwb_target_y = target.y_coord;

    if (is_position_reached(uwb_pos_info.uwb_target_x, uwb_pos_info.uwb_target_y, 5))
    {
        //  判断是否进入了新方格（用于触发识别）
        if (current_logic.row != last_visited_row || current_logic.col != last_visited_col) 
        {
            // 更新记录
            last_visited_row = current_logic.row;
            last_visited_col = current_logic.col;

            // 清空视觉缓存，进入等待状态
            vision_data.animal_number = 0;
            vision_data.animal_count = 0;
            wait_vision_flag = 1;          // 进入等待视觉数据状态
            wait_vision_counter = 0;       // 重置等待计数器

            return 0; // 等待视觉数据，不推进路径点

        }
        else 
        {
            // 重复路过同一方格，直接跳过
            path_exec_index++; // 推进到下一个目标点
            return (path_exec_index >= full_path_length) ? 1 : 0;
        }

    }
    return 0;
}
