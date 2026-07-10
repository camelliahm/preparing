#include "animal_record.h"
#include <string.h>

#define MAX_RECORDS 100   // 最多存100条（63个方格足够了）

static AnimalRecord g_records[MAX_RECORDS];
static uint16_t g_record_count = 0;

/*
@author: camelliahm
@brief:  初始化动物记录（清空所有记录）
*/
void animal_record_init(void)
{
    g_record_count = 0;
    memset(g_records, 0, sizeof(g_records));
}

/*
@author: camelliahm
@brief:  添加动物记录
@param:  row          方格行（0~6）
@param:  col          方格列（0~8） 
@param:  animal_code  动物代号（视觉模块定义）
@param:  count        该方格内该种动物的数量
*/
u16 animal_record_add(uint8_t row, uint8_t col, uint16_t animal_code, uint8_t count)
{
    if (animal_code == 0 || count == 0) return 0;
    if (g_record_count >= MAX_RECORDS) return 0;

    g_records[g_record_count].row = row;
    g_records[g_record_count].col = col;
    g_records[g_record_count].animal_code = animal_code;
    g_records[g_record_count].count = count;
    g_record_count++;
    return 1;
}

// 按动物代号统计总数（线性遍历，数据量小，足够快）
/*
@author: camelliahm
@brief:  查询某种代号动物的总数量（遍历所有记录求和）
@param:  animal_code  动物代号（视觉模块定义）
@return: uint16_t     该动物代号的总数量
*/
uint16_t animal_record_get_total(uint16_t animal_code)
{
    uint16_t total = 0;
    for (uint16_t i = 0; i < g_record_count; i++) 
    {
        if (g_records[i].animal_code == animal_code) 
        {
            total += g_records[i].count;
        }
    }
    return total;
}

/*
@author: camelliahm
@brief:  获取所有动物记录的指针（只读）
@return: const AnimalRecord*  指向动物记录数组的指针
*/
const AnimalRecord* animal_record_get_all(void)
{
    return g_records;
}

/*
@author: camelliahm
@brief:  获取动物记录数量
@return: uint16_t  当前记录数量
*/
uint16_t animal_record_get_count(void)
{
    return g_record_count;
}

/*
@author: camelliahm
@brief:  清空所有动物记录
*/
void animal_record_clear(void)
{
    animal_record_init();
}
