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

    // 统计所有水银开关的闭合数量
    for (int i = 0; i < NUM_SWITCHES; i++)
    {
        GPIO_PinState pin_state;

        // 读取水银开关的状态
        pin_state = HAL_GPIO_ReadPin(mercury_ports[i], mercury_pins[i]);

        if (pin_state == GPIO_PIN_SET)
        {
            count++;  // 增加闭合开关的数量
        }
    }
    return count;
}

void CheckAndOutputMercurySwitchStatus(void)
{
    // 使用 MercurySwitch_Check 函数获取闭合数量
    int closed_switch_count = MercurySwitch_Check();

    // 格式化输出字符串并通过串口输出
    char buffer[50];
//    snprintf(buffer, sizeof(buffer), "Closed Switch Count: %d\r\n", closed_switch_count);
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);  // 使用 UART3 进行发送
}
