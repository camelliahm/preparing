#include "Patrol_Task.h"
#include "Ano_Math.h"
#include "PID_ctrl.h"
#include "UWB_location.h"
#include "Drv_Sys.h"
#include "My_Comm.h"
#include "Laser.h"
#include "Drv_Uart.h"

Cargo Goods;

Cargo Goods_list[36] = 
{
    { 75, 75, 150, 0, 0, {0}},//0

    // A面 (x较小，y递增)
    { 75, 250, 140, 0, 0, "A1"},//1
    { 75, 200, 140, 0, 0, "A2"},//2
    { 75, 150, 140, 0, 0, "A3"},//3
    { 75, 150, 100, 0, 0, "A6"},//4
    { 75, 200, 100, 0, 0, "A5"},//5
    { 75, 250, 100, 0, 0, "A4"},//6

    //拐角
    { 75, 350, 100, 0,  0, {0}},//7
    {225, 350, 100, 0, 0, {0}},//8
    {225, 350, 100,180, 0, {0}},//9

    // B面 (y较小，x递增)
    {225, 250, 100, 180, 0, "B6"},//10
    {225, 200, 100, 180, 0, "B5"},//11
    {225, 150, 100, 180, 0, "B4"},//12
    {225, 150, 140, 180, 0, "B1"},//13
    {225, 200, 140, 180, 0, "B2"},//14
    {225, 250, 140, 180, 0, "B3"},//15

    //拐角
    {225, 350, 140, 180, 0, {0}},//16
    {275, 350, 140, 180, 0, {0}},//17
    {275, 350, 140,  0, 0, {0}},//18

    // C面 (x较大，y递增)
    {275, 250, 140,   0, 0, "C1"},//19
    {275, 200, 140,   0, 0, "C2"},//20
    {275, 150, 140,   0, 0, "C3"},//21
    {275, 150, 100,   0, 0, "C6"},//22
    {275, 190, 100,   0, 0, "C5"},//23
    {275, 145, 100,   0, 0, "C4"},//24

    //拐角
    {275, 350, 100,   0, 0, {0}},//25
    {425, 350, 100,   0, 0, {0}},//26
    {425, 350, 100, 180, 0, {0}},//27

    // D面 (y较小，x递增)
    {425, 250, 100, 180, 0, "D4"},//28
    {425, 200, 100, 180, 0, "D5"},//29
    {425, 150, 100, 180, 0, "D6"},//30
    {425, 150, 140, 180, 0, "D3"},//31
    {425, 200, 140, 180, 0, "D2"},//32
    {425, 250, 140, 180, 0, "D1"},//33

    //降落
    {425, 325, 140, 180, 0, {0}},//34
    {425, 325,   0, 180, 0, {0}},//35
};

/** 
 *  @brief 初始化巡检任务，清零 Goods 结构体
 *  @author camelliahm
 */
void Patrol_Init (void)
{
    memset(&Goods, 0, sizeof(Cargo));
}

/** 
 * @brief 更新悬停时间
 * @author camelliahm
 */
u32 Hover_time_Update(void)
{
    static u32 Hover_time_count = 0;
    return ++Hover_time_count;
}

/**
 * @brief 判断是否到达指定位置
 * @param tx 目标x坐标
 * @param ty 目标y坐标
 * @param tz 目标z坐标
 * @param d 判断距离
 * @return 1表示已到达，0表示未到达
 * @author camelliahm
 */
static uint8_t Is_Reached(s16 tx, s16 ty, s16 tz, u16 d)
{
    s16 dx = ABS(uwb_pos_info.now_x - tx);
    s16 dy = ABS(uwb_pos_info.now_y - ty);
    s16 dz = ABS(uwb_pos_info.now_z - tz);
    return (dx <= d && dy <= d && dz <= d);
}

/**
 * @brief 巡检任务主函数
 * @param map 目标点数组
 * @param map_size 数组索引
 * @param hover_time 悬停时间
 * @author camelliahm
 */
uint8_t Patrol_Task( Cargo map[], u8 map_size, u32 hover_time)
{
    static u8  current_index = 0;
    static u8  state = 0;               // 0: 飞往目标点  1: 货物点悬停等待
    static u32 wait_start = 0;          // 悬停开始时间(ms)
    u8 data_buf[4];

    if (current_index >= map_size) return 1;                         // 遍历完成，不再动作
    
    const Cargo *p = &map[current_index];

    /* 设置目标坐标与航向 */
    uwb_pos_info.uwb_target_x = p->x;
    uwb_pos_info.uwb_target_y = p->y;
    uwb_pos_info.uwb_target_z = p->z;
    uwb_pos_info.target_yaw_deg = p->yaw;
    uwb_pos_info.yaw_lock = 1;

        if (state == 0)                     // 飞行阶段
        {
            if (Is_Reached(p->x, p->y, p->z, 5))   // 到达目标点(允许5cm误差)
            {
                if (p->coord[1] != 0)         // 货物点，需要悬停
                {
                    state = 1;
                    wait_start = GetSysRunTimeMs();
                }
                else                        // 过渡点或起降点，直接进入下一个
                {
                    current_index++;
                }
            }
        }
        else                                // 货物点悬停等待阶段
        {
            if (GetSysRunTimeMs() - wait_start >= hover_time)
            {
                state = 0;
                current_index++;            // 前往下一个点
            }
            else 
            {
                if (GetSysRunTimeMs() - wait_start == 1000)
                {
                    Laser_On();
                }
                else if(GetSysRunTimeMs() - wait_start == 1500)
                {
                    Laser_Off();
                }
                map[current_index].number = vision_data.number;
                data_buf[0] = map[current_index].coord[0];
                data_buf[1] = map[current_index].coord[1];
                data_buf[2] = map[current_index].number;
                ANO_Send_Custom(data_buf,3, ANO_GROUND_FRAME, DrvUart2SendBuf);
            }
        }
    return 0;
}

/**
 * @brief  根据目标货物编号找到位置
 * @param  target_id  视觉识别的目标货物编号 (1~24)
 * @return 子路径的实际长度 (0 表示未找到目标)
 */
uint8_t DirectedPath_map(uint8_t target_id, uint8_t *found_idx)
{
    static uint8_t search_idx = 0;   // 当前正在检查的 Goods_list 索引
    //static uint8_t found_idx  = 0;   // 找到后保存的索引
    static uint8_t found_flag = 0;   // 0:查找中, 1:已找到

    // 如果已经找到，直接返回
    if (found_flag) return 1;

    // 超出范围，未找到
    if (search_idx >= 36) {
        search_idx = 0;
        return 0;
    }

    // 检查当前索引的货物编号是否匹配
    if (Goods_list[search_idx].number == target_id) {
        *found_idx = search_idx;
        found_flag = 1;
        return 1;
    }

    search_idx++;
    return 0;
}

/**
 * @brief  添加一个路径点到子路径数组
 * @param  sub_path  子路径数组
 * @param  len       当前长度指针（函数内自增）
 * @param  x, y, z   目标坐标 (cm)
 * @param  yaw       目标航向角 (度)
 */
static void Add_Waypoint(Cargo *sub_path, uint8_t *len, s16 x, s16 y, s16 z, s16 yaw)
{
    sub_path[*len].x   = x;
    sub_path[*len].y   = y;
    sub_path[*len].z   = z;
    sub_path[*len].yaw = yaw;
    (*len)++;
}

/**
 * @brief  根据目标货物索引生成定向盘点子路径
 * @param  target_idx  目标货物在 Goods_list 中的索引 (货物点:1~6,10~15,19~24,28~33)
 * @param  sub_path    输出路径数组 (需预分配至少16个Cargo空间)
 * @return 子路径实际长度
 * @note   水平移动约束：仅在Y=350时允许X变化；仅在X∈{75,225,275,425}时允许Y变化。
 *         高度仅在到达目标正上方或降落时调整，不在走廊移动中无效升降。
 */
uint8_t DirectedPath_Build(uint8_t target_idx, Cargo sub_path[])
{
    uint8_t len = 0;
    const Cargo *target = &Goods_list[target_idx];
    
    // 起点(0): 75,75,150,0°
    sub_path[len++] = Goods_list[0];
    
    /* 根据目标面选择走廊路线 */
    if (target_idx <= 6) {
        // ===== A面 (X=75, 不需要走廊移动) =====
        // 同X，只变Y、Z、Yaw
        Add_Waypoint(sub_path, &len, 75, target->y, 150, 0);
    }
    else if (target_idx <= 15) {
        // ===== B面 (X=225, 需要通过Y=350走廊) =====
        Add_Waypoint(sub_path, &len, 75, 350, 150, 0);// 从起点(75,75)沿X=75北移到Y=350
        
        Add_Waypoint(sub_path, &len, 225, 350, 150, 0);// 沿Y=350东移到X=225
        
        Add_Waypoint(sub_path, &len, 225, target->y, 150, 0);// 降到目标Z，南移到目标Y
    }
    else if (target_idx <= 24) {
        // ===== C面 (X=275, 需要通过Y=350走廊) =====
        
        Add_Waypoint(sub_path, &len, 75, 350, 150, 0);// 从起点沿X=75北移到Y=350
        
        // Add_Waypoint(sub_path, &len, 225, 350, 150, 0);// 沿Y=350东移到X=225
        
        Add_Waypoint(sub_path, &len, 275, 350, 150, 0);// 沿Y=350东移到X=275

        Add_Waypoint(sub_path, &len, 275, target->y, 150, 0);// 沿X=275通道南移到目标Y
    }
    else {
        // ===== D面 (X=425, 需要通过Y=350走廊) =====
        
        Add_Waypoint(sub_path, &len, 75, 350, 150, 0);// 从起点沿X=75北移到Y=350
        
        Add_Waypoint(sub_path, &len, 425, 350, 150, 0);// 沿Y=350东移到X=425
    
        Add_Waypoint(sub_path, &len, 425, target->y, 150, 0);// 沿X=425通道南移到目标Y
    }
    
    /* 所有面共同：到达目标正上方 → 下降 → 调航向 → 识别 */
    Add_Waypoint(sub_path, &len, target->x, target->y, 150, 0);     // 目标正上方150cm
    Add_Waypoint(sub_path, &len, target->x, target->y, target->z, 0); // 降到目标Z
    Add_Waypoint(sub_path, &len, target->x, target->y, target->z, target->yaw); // 调航向
    // 识别点（保留完整货物信息）
    sub_path[len-1].number = target->number;
    strcpy(sub_path[len-1].coord, target->coord);
    
    /* 识别完成后，返回走廊并降落 */
    // 升到安全高度，退回走廊
    Add_Waypoint(sub_path, &len, target->x, target->y, 150, target->yaw); // 升到150cm
    Add_Waypoint(sub_path, &len, target->x, target->y, 150, 0); // 调整航向为0°，便于沿走廊移动
    // 沿安全路径返回北走廊Y=350（保持150cm）
    if (target->y != 350) {
        Add_Waypoint(sub_path, &len, target->x, 350, 150, 0); // 北移到Y=350
    }
    // 沿Y=350西移回到降落航线（如果需要的话，确保到达X=425）
    if (target->x != 425) {
        Add_Waypoint(sub_path, &len, 425, 350, 150, 0); // 沿走廊到X=425
    }
    // 降落悬停点 34: (425,325,140)
    Add_Waypoint(sub_path, &len, 425, 325, 140, 0);
    // 降落点 35: (425,325,0)
    Add_Waypoint(sub_path, &len, 425, 325, 0, 0);
    
    return len;
}
