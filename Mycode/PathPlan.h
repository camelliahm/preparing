#ifndef _PATHPLAN_H_
#define _PATHPLAN_H_
#include "stm32f4xx.h"                  // Device header

// 常量定义（适配9×7方格：A1-A9横向，B1-B7纵向）
#define ROWS 7       // 对应B1-B7（纵向7行）
#define COLS 9       // 对应A1-A9（横向9列）
#define MAX_CELLS (ROWS * COLS)
#define MAX_PATH_LENGTH 1000
#define BARRIER_COUNT 3  // 禁飞区为3个连续方格

// 坐标结构体（row对应B1-B7，col对应A1-A9）
typedef struct 
{
    int8_t row;  // 行：0→B1，1→B2，...，6→B7
    int8_t col;  // 列：0→A1，1→A2，...，8→A9
} Point;

// 全局变量 - 存储最终巡查路径
extern Point full_path[MAX_PATH_LENGTH];  // 路径点数组（按巡查顺序存储)
extern uint16_t full_path_length;

void init_grid(void);
uint8_t check_connectivity(void);
void set_barriers(void);
void solve_shortest_path(void);

#endif
