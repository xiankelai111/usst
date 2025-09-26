#ifndef __KELLER_H
#define __KELLER_H

#include "stm32f4xx_hal.h"
#define KELLER_RXBUFFER_LEN 10

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

extern float KellerBusDat; // 声明外部变量
extern unsigned char KellerBusRecvBuf[KELLER_RXBUFFER_LEN]; // 声明外部变量


void KellerInit(void);
void KellerBusStart(void);
uint8_t KellerBusRead(void);
void KellerBusSendCmd(unsigned char _fuc);
void KellerBusTxCpltCallback(UART_HandleTypeDef *huart);
void KellerBusRxCpltCallback(UART_HandleTypeDef *huart);

#endif /* __KELLER_H */
