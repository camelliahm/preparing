#ifndef _ANIMAL_RECORD_H_
#define _ANIMAL_RECORD_H_

#include <stdint.h>
#include "stm32f4xx.h"

typedef struct {
    uint8_t row;          // 所在方格行 B1~B7 (0~6)
    uint8_t col;          // 所在方格列 A1~A9 (0~8)
    uint16_t animal_code; // 动物代号（视觉模块定义，例如 0x01=鹿，0x02=野猪...）
    uint8_t count;        // 该方格内该种动物的数量
} AnimalRecord;


void animal_record_init(void);
extern uint16_t animal_record_add(uint8_t row, uint8_t col, uint16_t animal_code, uint8_t count);
uint16_t animal_record_get_total(uint16_t animal_code);
const AnimalRecord* animal_record_get_all(void);
uint16_t animal_record_get_count(void);
void animal_record_clear(void);

#endif
