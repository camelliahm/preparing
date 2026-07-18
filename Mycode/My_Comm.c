#include "My_Comm.h"
#include "Drv_Uart.h"                   

Ground_Data_TypeDef ground_data;
Vision_Track_Data vision_data;
static u8 rx_state = 0;
static u8 rx_buf[32];
static u8 rx_len = 0;
static u8 data_len = 0;
static u8 frame_type = 0; // 帧类型：0xF3=视觉  0xF4=投放

static void ANO_CalculateCheck(u8 *data, u8 len, u8 *sc, u8 *ac)
{
    u8 sumcheck = 0;
    u8 addcheck = 0;
    u8 i;

    for (i = 0; i < len; i++)
    {
        sumcheck += data[i];
        addcheck += sumcheck;
    }
    *sc = sumcheck;
    *ac = addcheck;
}

void My_Comm_Init(void)
{
    memset(&ground_data, 0, sizeof(Ground_Data_TypeDef));
    memset(&vision_data, 0, sizeof(Vision_Track_Data));
    rx_state = 0;
    rx_len = 0;
    data_len = 0;
    frame_type = 0;
}

void My_Comm_ReceiveByte(u8 data)
{
    u8 calc_sc, calc_ac;

    switch (rx_state)
    {
        case 0:
            if (data == 0xAA)
            {
                rx_buf[0] = data;
                rx_state = 1;
            }
            break;

        case 1:
            if (data == 0xFF)
            {
                rx_buf[1] = data;
                rx_state = 2;
            }
            else
                rx_state = 0;
            break;

        case 2:
            rx_buf[2] = data;
            if(data == ANO_VISION_FRAME)    // F3：视觉帧
            {
                frame_type = ANO_VISION_FRAME;
                rx_state = 3;
            }
            else if(data == ANO_GROUND_FRAME)    // F4：地面站帧
            {
                frame_type = ANO_GROUND_FRAME;
                rx_state = 3;
            }
            else
            {
                rx_state = 0;
            }
            break;

        case 3:
            data_len = data;
            rx_buf[3] = data;
            rx_len = 4;
            rx_state = 4;
            break;

        case 4:
            rx_buf[rx_len++] = data;
            if (rx_len >= data_len + 4)
                rx_state = 5;
            break;

        case 5:
            rx_buf[rx_len++] = data;
            rx_state = 6;
            break;

        case 6:
            rx_buf[rx_len++] = data;
            ANO_CalculateCheck(rx_buf, data_len + 4, &calc_sc, &calc_ac);

            if (calc_sc == rx_buf[data_len + 4] && calc_ac == rx_buf[data_len + 5])
            {
                if (frame_type == ANO_GROUND_FRAME)
                {

                }
                else if (frame_type == ANO_VISION_FRAME)
                {
                    vision_data.number = rx_buf[4];
                }
            }
            else
            {
                // 校验失败，数据无效
                if(frame_type == ANO_VISION_FRAME)
                {
                    memset(&vision_data, 0, sizeof(Vision_Track_Data));
                }
                else if(frame_type == ANO_GROUND_FRAME)
                {
                    memset(&ground_data, 0, sizeof(Ground_Data_TypeDef));
                }
            }

            rx_state = 0;
            rx_len = 0;
            data_len = 0;
            frame_type = 0;
            break;

        default:
            rx_state = 0;
            rx_len = 0;
            data_len = 0;
            frame_type = 0;
            break;
    }
}

/**
 * @brief  匿名协议 灵活帧 通用发送接口
 * @note   纯协议层，数据由外部传入，自动组帧+校验+串口1发送
 * @param  *data_buf：要发送的 数据区 指针
 * @param  data_len：数据区长度（0~20字节均可）
 * @param  frame:功能码
 * @param  (*DrvUartSendBuf)(unsigned char *, u8)：对应调用的发送的串口发送函数
 * @retval 无
 */
void ANO_Send_Custom(u8 *data_buf, u8 data_len, u8 frame, void (*DrvUartSendBuf)(unsigned char *, u8))
{
    u8 frame_buf[32];
    u8 frame_len = 0;
    u8 sumcheck, addcheck;

    // 组帧
    frame_buf[frame_len++] = 0xAA;  // 帧头0xAA
    frame_buf[frame_len++] = 0xFF;  // 目标地址0xFF
    frame_buf[frame_len++] = frame;  // 功能码0xF4
    frame_buf[frame_len++] = data_len;
    
    // 数据区
    for(u8 i = 0; i < data_len; i++)
    {
        frame_buf[frame_len++] = data_buf[i];
    }

    // 计算校验值
    ANO_CalculateCheck(frame_buf, frame_len, &sumcheck, &addcheck);

    // 添加校验值
    frame_buf[frame_len++] = sumcheck;
    frame_buf[frame_len++] = addcheck;

    // 发送
    DrvUartSendBuf(frame_buf, frame_len);
}
