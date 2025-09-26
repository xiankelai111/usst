#include "main.h"
#include "motor.h"
#include <math.h> 
#include <stdio.h> 

extern float pitch;
extern float roll;

void Motor_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
}

void Motor_Run(uint32_t dir, uint32_t num, uint32_t speed_us, uint32_t motor)
{
    uint16_t dir_pin, pulse_pin, enable_pin;

    switch (motor)
    {
        case 1:
            dir_pin = GPIO_PIN_6;
            pulse_pin = GPIO_PIN_5;
            enable_pin = GPIO_PIN_7;
            break;
        case 2:
            dir_pin = GPIO_PIN_9;
            pulse_pin = GPIO_PIN_8;
            enable_pin = GPIO_PIN_10;
            break;
        case 3:
            dir_pin = GPIO_PIN_12;
            pulse_pin = GPIO_PIN_11;
            enable_pin = GPIO_PIN_13;
            break;
        default:
            return;
    }
    printf("Motor %u running, direction: %u, steps: %u, speed: %u us\r\n", motor, dir, num, speed_us);
    
		HAL_GPIO_WritePin(GPIOC, enable_pin, GPIO_PIN_SET); 
		
		if (dir == 1)
    {
        HAL_GPIO_WritePin(GPIOC, dir_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, dir_pin, GPIO_PIN_RESET);
    }

    for (uint32_t i = 0; i <= (num * 800); i++)
    {
        HAL_Delay(speed_us / 1000);  // Convert microsecond delay to millisecond
        HAL_GPIO_TogglePin(GPIOC, pulse_pin);
    }

    HAL_GPIO_WritePin(GPIOC, enable_pin, GPIO_PIN_RESET); // Stop the motor after running
    printf("Motor %u stopped\r\n", motor);
}

void AdjustPlane(float pitch, float roll)
{
    int steps_per_degree = 1;  // 根据你的电机和机械结构调整这个值
    int pitch_steps = (int)(pitch * steps_per_degree);
    int roll_steps = (int)(roll * steps_per_degree);
 
	  printf("Adjusting plane with pitch: %f, roll: %f\r\n", pitch, roll);
    
//    if (fabs(pitch) < 1.0 && fabs(roll) < 1.0)
//    {
//        return;
//    }
    
    if (pitch_steps != 0)
    {
        if (pitch_steps > 0)
        {
            Motor_Run(1, pitch_steps, 100, 1);  // 假设电机 1 控制 pitch
        }
        else
        {
            Motor_Run(0, -pitch_steps, 100, 1);
        }
    }

    if (roll_steps != 0)
    {
        if (roll_steps > 0)
        {
            Motor_Run(1, roll_steps, 300, 3);  // 假设电机 2 控制 roll
        }
        else
        {
            Motor_Run(0, -roll_steps, 300, 3);
        }
    }

    // 假设电机 3 是备用或用于其他调整
    // 你可以根据需要增加更多的控制逻辑
}

int Motor_AdjustComplete(void)
{
    // 这里你需要实现一个逻辑来判断调平是否完成。
    // 例如：传感器读数稳定在某个阈值以内时，返回1；否则返回0。
    
    
    if (fabs(pitch) < 1.0 && fabs(roll) < 1.0) // 设定阈值，例如0.1度以内
    {
        return 1;  // 调平完成
    }
    return 0;  // 调平未完成
}
