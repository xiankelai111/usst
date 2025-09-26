#include "Switch.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

GPIO_TypeDef* mercury_ports[NUM_SWITCHES] = {
    GPIOA, GPIOA, GPIOC, GPIOB, 
    GPIOB, GPIOB, GPIOB, GPIOB
};

uint16_t mercury_pins[NUM_SWITCHES] = {
    GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_15, GPIO_PIN_0, 
    GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_9
};

int MercurySwitch_Check(void)
{
    int count = 0;

    // ͳ������ˮ�����صıպ�����
    for (int i = 0; i < NUM_SWITCHES; i++)
    {
        GPIO_PinState pin_state;

        // ��ȡˮ�����ص�״̬
        pin_state = HAL_GPIO_ReadPin(mercury_ports[i], mercury_pins[i]);

        if (pin_state == GPIO_PIN_SET)
        {
            count++;  // ���ӱպϿ��ص�����
        }
    }
    return count;
}

void CheckAndOutputMercurySwitchStatus(void)
{
    // ʹ�� MercurySwitch_Check ������ȡ�պ�����
    int closed_switch_count = MercurySwitch_Check();

    // ��ʽ������ַ�����ͨ���������
    char buffer[50];
//    snprintf(buffer, sizeof(buffer), "Closed Switch Count: %d\r\n", closed_switch_count);
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);  // ʹ�� UART3 ���з���
}
