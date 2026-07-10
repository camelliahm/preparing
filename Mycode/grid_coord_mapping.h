#ifndef _GRID_COORD_MAPPING_H_
#define _GRID_COORD_MAPPING_H_

#include "stm32f4xx.h"                  // Device header
#include "PathPlan.h"
#include "My_Comm.h"
#include "UWB_location.h"
#include "PID_ctrl.h" 
#include "animal_record.h"
#include <math.h>


#define GRID_STEP 25     //网格对应长度（厘米）
#define BASEX   0
#define BASEY   0

typedef struct
{
    s16 x_coord;
    s16 y_coord;
} GridCoord;

extern GridCoord grid_coord; 

static inline GridCoord map_point_to_phys(Point p)
{
    GridCoord result;
    result.x_coord = (p.row * 2 + 1) * GRID_STEP  + BASEX;
    result.y_coord = (((COLS - 1) - p.col) * 2 + 1) * GRID_STEP + BASEY;
    return result;
}

void convert_full_path_to_coords(const Point *src, GridCoord *dst, uint16_t len);
void path_coord_to_target_coord (void);
void path_exec_init(void);
extern uint8_t path_exec_step(void);

#endif
