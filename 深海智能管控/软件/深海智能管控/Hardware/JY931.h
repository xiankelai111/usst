#ifndef _JY931_H
#define _JY931_H

#include "main.h"
#include "JY901.h"
#include "stdio.h"
#include "string.h"


#define JY931_RXBUFFER_LEN 33

typedef struct
{
	float angle[3];
}Angle;

typedef struct
{
	float a[3];
}Acc;

typedef struct
{
	float w[3];
}SGyro;

typedef struct User_USART
{
		uint8_t Rx_flag;												//������ɱ�־
		uint8_t Rx_len;													//���ճ���
		uint8_t frame_head;											//֡ͷ
		uint8_t RxBuffer[JY931_RXBUFFER_LEN];					//���ݴ洢
		Angle angle;
		Acc acc;
		SGyro w;
}User_USART;



extern User_USART JY931_data;

void JY931_Process(void);
void User_USART_Init(User_USART *Data);
void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen);

float JY931_getPitch(void);
float JY931_getRoll(void);
float JY931_getYaw(void);
	

#endif
