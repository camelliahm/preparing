#ifndef PATROL_TASK_H
#define PATROL_TASK_H

#include "stm32f4xx.h"
#include "string.h"

#define MAX_AMOUNT 100

typedef struct{

    s16 x, y, z;
    s16 yaw;
    u8 number;
    char coord[4];

} Cargo;

extern Cargo Goods;
extern Cargo Goods_list[36];

void Patrol_Init (void);
u32 Hover_time_Update(void);
static uint8_t Is_Reached(s16 tx, s16 ty, s16 tz, u16 d);
uint8_t Patrol_Task( Cargo map[], u8 map_size, u32 hover_time);
uint8_t DirectedPath_map(uint8_t target_id, uint8_t *found_idx);
uint8_t DirectedPath_Build(uint8_t target_idx, Cargo sub_path[]);


#endif
