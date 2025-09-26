#include "Keller.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define KellerHandle huart1

// RS485 Enable pin (PB8)
#define RS485_Enable_Pin GPIO_PIN_8
#define RS485_Port GPIOB

#define KellerBusSend() HAL_GPIO_WritePin(RS485_Port, RS485_Enable_Pin, GPIO_PIN_SET)
#define KellerBusRev() HAL_GPIO_WritePin(RS485_Port, RS485_Enable_Pin, GPIO_PIN_RESET)

// Forward declaration of the crc16 function
static void crc16(unsigned char *CRC_H, unsigned char *CRC_L, const unsigned char buffer[], const int offset, const int bteCount);

float KellerBusDat = 0.0f;
unsigned char KellerBusRecvBuf[10] = {0,};
unsigned char KellerBusSendBuf[10] = {0,};
unsigned char KellerBusNowFucCmd = 0;

void KellerInit(void)
{
    // Ensure GPIOB clock is enabled
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Initialize GPIO for RS485 Enable pin
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RS485_Enable_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_Port, &GPIO_InitStruct);
}

void KellerBusStart(void)
{
    HAL_Delay(100);
    KellerBusSendCmd(48);
    HAL_Delay(100);
}

uint8_t KellerBusRead(void)
{
    static unsigned long int _lastHalTick_KellerBusRead = 0;
    static unsigned char _stepFlag_KellerBusRead = 0;

    if (_lastHalTick_KellerBusRead > HAL_GetTick())
        _lastHalTick_KellerBusRead = HAL_GetTick(); // Correct previous timer error

    switch (_stepFlag_KellerBusRead)
    {
    case 0:
        if (HAL_GetTick() - _lastHalTick_KellerBusRead >= 1000)
        {
            KellerBusSendCmd(73);
            _lastHalTick_KellerBusRead = HAL_GetTick();
            _stepFlag_KellerBusRead = 1;
        }
        break;
    case 1:
        if (KellerBusNowFucCmd == 0)
        {
            _stepFlag_KellerBusRead = 0;
            return 0;
        }
        else if (HAL_GetTick() - _lastHalTick_KellerBusRead >= 1000)
        {
            HAL_UART_AbortReceive(&KellerHandle);
            KellerBusSendCmd(48);
            _lastHalTick_KellerBusRead = HAL_GetTick();
            _stepFlag_KellerBusRead = 2;
        }
        break;
    case 2:
        if (KellerBusNowFucCmd == 0)
        {
            _stepFlag_KellerBusRead = 0;
        }
        else if (HAL_GetTick() - _lastHalTick_KellerBusRead >= 1000)
        {
            HAL_UART_AbortReceive(&KellerHandle);
            KellerBusSendCmd(48);
            _lastHalTick_KellerBusRead = HAL_GetTick();
            _stepFlag_KellerBusRead = 2;
        }
        break;
    default:
        _lastHalTick_KellerBusRead = HAL_GetTick();
        _stepFlag_KellerBusRead = 2;
        break;
    }
    return 1;
}

void KellerBusSendCmd(unsigned char _fuc)
{
    unsigned char i = 0;

    KellerBusSend();
    memset(KellerBusRecvBuf, 0, 10);
    memset(KellerBusSendBuf, 0, 10);

    KellerBusSendBuf[i++] = 0x01; // Assuming 0x01 is the address for KellerBusNoteAddr
    KellerBusSendBuf[i++] = _fuc;

    switch (_fuc)
    {
    case 48:
        break;
    case 73:
        KellerBusSendBuf[i++] = 1;
        break;
    }

    unsigned char CRC_H, CRC_L;
    crc16(&CRC_H, &CRC_L, KellerBusSendBuf, 0, i);

    KellerBusSendBuf[i++] = CRC_H;
    KellerBusSendBuf[i++] = CRC_L;

    KellerBusNowFucCmd = _fuc;

    HAL_UART_Transmit_IT(&KellerHandle, KellerBusSendBuf, i);
}

void KellerBusTxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == KellerHandle.Instance)
    {
        KellerBusRev();
        if (KellerBusNowFucCmd == 73)
        {
            HAL_UART_Receive_IT(&KellerHandle, KellerBusRecvBuf, 9);
        }
        else if (KellerBusNowFucCmd == 48)
        {
            HAL_UART_Receive_IT(&KellerHandle, KellerBusRecvBuf, 10);
        }
    }
}

void KellerBusRxCpltCallback(UART_HandleTypeDef *huart)
{
    unsigned char _ctof[4] = {0,};
    if (huart->Instance == KellerHandle.Instance)
    {
        if (KellerBusRecvBuf[1] == 73)
        {
            _ctof[0] = KellerBusRecvBuf[5];
            _ctof[1] = KellerBusRecvBuf[4];
            _ctof[2] = KellerBusRecvBuf[3];
            _ctof[3] = KellerBusRecvBuf[2];
            KellerBusDat = *(float*)_ctof;
        }
        KellerBusNowFucCmd = 0;
    }
}

static void crc16(unsigned char *CRC_H, unsigned char *CRC_L, const unsigned char buffer[], const int offset, const int bteCount)
{
    unsigned char i, n;
    unsigned char ex;
    unsigned short crc;
    unsigned short polynom = 0xA001;

    crc = 0xFFFF;
    for (i = 0; i < bteCount; i++)
    {
        crc = (unsigned short)(crc ^ buffer[offset + i]);

        for (n = 0; n < 8; n++)
        {
            if (crc % 2 == 1)
            {
                ex = 1;
            }
            else
            {
                ex = 0;
            }
            crc = (unsigned short)(crc / 2);
            if (ex == 1)
            {
                crc = (unsigned short)(crc ^ polynom);
            }
        }
    }
    *CRC_H = (unsigned char)(crc >> 8);
    *CRC_L = (unsigned char)(crc & 0x00FF);
}
