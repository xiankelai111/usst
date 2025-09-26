/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "gps.h" 
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern int closed_switch_count;
	extern volatile bool sensor_reading_started;
extern volatile bool pressure_sensor_triggered;
extern volatile bool gps_reading_triggered;

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MT3_EN_Pin GPIO_PIN_13
#define MT3_EN_GPIO_Port GPIOC
#define MT1_D_Pin GPIO_PIN_6
#define MT1_D_GPIO_Port GPIOC
#define MT1_EN_Pin GPIO_PIN_7
#define MT1_EN_GPIO_Port GPIOC
#define MT2_P_Pin GPIO_PIN_8
#define MT2_P_GPIO_Port GPIOC
#define MT2_D_Pin GPIO_PIN_9
#define MT2_D_GPIO_Port GPIOC
#define MT2_EN_Pin GPIO_PIN_10
#define MT2_EN_GPIO_Port GPIOC
#define MT3_P_Pin GPIO_PIN_11
#define MT3_P_GPIO_Port GPIOC
#define MT3_D_Pin GPIO_PIN_12
#define MT3_D_GPIO_Port GPIOC
#define RS485_EN_Pin GPIO_PIN_8
#define RS485_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
