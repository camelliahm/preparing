#ifndef LASER_H
#define LASER_H

#include "SysConfig.h"
#include "stm32f4xx.h"
#include "Drv_Sys.h"

void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);
void Laser_Blink(u32 time_ms);
void Laser_Update(void); 

#endif
