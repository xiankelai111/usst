#include "JY931.h"

User_USART JY931_data;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

struct SAcc 	stcAcc;
struct SGyro 	stcGyro;
struct SAngle 	stcAngle;

uint8_t a =0;

//��ʼ������
void User_USART_Init(User_USART *Data)
{
    for (uint16_t i = 0; i < JY931_RXBUFFER_LEN; i++) Data->RxBuffer[i] = 0;
    Data->frame_head = 0x55;  // ȷ�����ֵ��ʵ������֡ͷһ��
    Data->Rx_flag = 0;
    Data->Rx_len = 0;

    HAL_UART_Receive_IT(&huart2, Data->RxBuffer, 1);  // ÿ�ν���һ���ֽڣ������ж�
}

void JY931_Process()
{
    if(JY931_data.Rx_len < JY931_RXBUFFER_LEN) return;    //���ݳ��Ȳ��ԡ�
    
    for(uint8_t i=0; i<4; i++)
    {
        if(JY931_data.RxBuffer[i*11] != JY931_data.frame_head)  continue;
        char msg[500];
        memset(msg, 0, sizeof(msg));
                // ����һ�����������ڴ洢Ҫ���͵��ַ���
        switch(JY931_data.RxBuffer[i*11+1])
        {

          
                case 0x53:    
                memcpy(&stcAngle, &JY931_data.RxBuffer[2 + i*11], 8);
                for(uint8_t j = 0; j < 3; j++)
                {
                    JY931_data.angle.angle[j] = (float)stcAngle.Angle[j] / 32768 * 180;  // �Ƕ�
                }
//                snprintf(msg, sizeof(msg), "%.2f %.2f %.2f  ", JY931_data.angle.angle[0], JY931_data.angle.angle[1], JY931_data.angle.angle[2]);
//                HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 1000);
                break;

        }
        
    }
    
    HAL_UART_Receive_DMA(&huart2, JY931_data.RxBuffer, JY931_RXBUFFER_LEN);
}

// ���ظ�����
float JY931_getPitch(void) {
    return JY931_data.angle.angle[0];
}

// ���غ����
float JY931_getRoll(void) {
    return JY931_data.angle.angle[1];
}

// ���غ����
float JY931_getYaw(void) {
    return JY931_data.angle.angle[2];
}
