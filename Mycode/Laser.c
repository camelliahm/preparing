#include "Laser.h"

static u8 laser_is_on = 0;
static u32 laser_off_time =0;


void Laser_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //开启gpio时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    //配置GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    //默认关闭激光
    GPIO_ResetBits(GPIOD , GPIO_Pin_14);

    laser_is_on = 0;

}

void Laser_On(void)
{
    GPIO_SetBits(GPIOD , GPIO_Pin_14);
    laser_is_on = 1;
}

void Laser_Off(void)
{
    GPIO_ResetBits(GPIOD , GPIO_Pin_14);
    laser_is_on = 0;
}

void Laser_Blink( u32 time_ms )
{
    Laser_On();
    laser_off_time = GetSysRunTimeMs()+ time_ms;
}

void Laser_Update(void)
{
    if (laser_is_on)
    {
        if (GetSysRunTimeMs() >= laser_off_time)
        {
            Laser_Off();
        }
    }
}
