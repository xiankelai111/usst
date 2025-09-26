/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "ds.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
    #include "DHT11.h"  
    #include "motor.h"
    #include "Keller.h" 
    #include "gps.h"
    #include "JY901.h"
    #include "JY931.h"
    #include "stm32f4xx_hal.h"
    #include <string.h>
    #include <stdio.h> 
    #include <stdbool.h>
    #include "Switch.h"
		
	int closed_switch_count = 0;  // 定义全局变量

/* USER CODE END Includes */
volatile uint8_t adc_dma_complete = 0;
volatile uint8_t dht11_1_updated = 0;
volatile uint8_t dht11_2_updated = 0;
float pitch, roll, yaw;
volatile uint32_t timer_ticks = 0;
extern volatile bool usart2_rx_complete;
uint8_t uart_buffer[512];
volatile int motor_in_progress = 0; // 电机调平是否正在进行
volatile int delay_completed = 0; 
volatile int motor_completed = 0;    // 电机调平是否完成的标志
volatile bool sensor_reading_started = false;
volatile bool pressure_sensor_triggered = false;
volatile bool gps_reading_triggered = false;
volatile bool system_enabled = false;


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    // ADC数据
    unsigned short adc_values[5];
    uint8_t humi1, temp1;
    uint8_t humi2, temp2;
    // IMU姿态数据
    float pitch;
    float roll;
    float yaw;
    // GPS数据（需要添加的字段）
    GPSData gps;  // 添加 GPS 数据结构体
} SystemSensorData;

/* USER CODE END PTD */
volatile SystemSensorData system_sensor_data;

// 定义全局传感器数据变量

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_Value[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void ProcessDHT11Data(void);
void ProcessADCData(void);
void ProcessIMUData(void);
void SystemClock_Config(void);
void integrate_and_fuse_data(void);
float ds_sensor_data[NUM_FEATURES];
void calculate_svm_prob(float *features, float *probabilities);
void OutputGpsData(void);
//void check_sensor_stability_and_restart_motor(void);
/* USER CODE BEGIN PFP */
 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void processGpsData(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t rx_buffer[50] = {0}; // 临时接收缓冲区
  MX_DMA_Init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    Motor_Init();
    KellerInit();
    KellerBusStart();
    User_USART_Init(&JY931_data);
		 Motor_AdjustComplete();
    GPS_Init();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_Value, 5);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
//		printf("ok1");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  extern volatile bool gps_data_ready;
extern volatile bool gps_data_processing;

 
    // 等待开启指令
    while (!system_enabled) {
        memset(rx_buffer,0,sizeof(rx_buffer));
        HAL_UART_Receive(&huart3, rx_buffer, sizeof(rx_buffer), 1000); // 接收数据

        // 检查是否接收到 "start" 指令
        if (strstr((char *)rx_buffer, "start") != NULL) {
            system_enabled = true; // 设置系统开启标志
            printf("System Enabled. Starting operations...\r\n");
        } else {
            printf("Waiting for 'start' command...\r\n");
        }

        HAL_Delay(500); // 控制串口监听频率
    }

while (1) {
	 if (timer_ticks >= 1 && motor_in_progress == 0)  //timer_ticks代表延时的长度，20分钟等于1200秒
        {
					HAL_Delay(100);
					if (usart2_rx_complete) {
											usart2_rx_complete = false;
											JY931_Process();
										}  

            motor_in_progress = 1;   // 标记调平开始
            delay_completed = 1;     // 标记延时已完成

                pitch = JY931_data.angle.angle[0];
                roll =  JY931_data.angle.angle[1];
                yaw =   JY931_data.angle.angle[2];                             
                           
            // 进入调平过程（反复启停电机）

            while (!Motor_AdjustComplete())
            {
								HAL_Delay(100);
                 if (usart2_rx_complete) {
											usart2_rx_complete = false;
											JY931_Process();
										}  
                pitch = JY931_data.angle.angle[0];
                roll =  JY931_data.angle.angle[1];
                yaw =   JY931_data.angle.angle[2];                
                printf("Pitch: %f, Roll: %f, Yaw: %f\r\n", pitch, roll, yaw); // 获取实时的传感器数据
                
                AdjustPlane(pitch, roll);            // 调整电机
                HAL_Delay(500);                      // 适当延时，模拟电机的反复启停
            }
            motor_completed = 1;  // 调平完成，标记完成
						sensor_reading_started = false;
						pressure_sensor_triggered = false;
						gps_reading_triggered = false;
        }
	if (motor_completed == 1)
         {			
    // 检查水银开关闭合数量
    closed_switch_count = MercurySwitch_Check();
    printf("Closed Switch Count: %d\r\n", closed_switch_count);

    // 只有在水银开关数量大于 1 时才处理 GPS 数据
    if (!sensor_reading_started && closed_switch_count > 0) {
        // 一旦水银开关数量大于0，縤t始读取数据
        sensor_reading_started = true;
    }
		
    if (sensor_reading_started) {
        // 处理 GPS 数据和传感器数据

        ProcessDHT11Data();
        ProcessADCData();
        ProcessIMUData();
        integrate_and_fuse_data();
//        check_sensor_stability_and_restart_motor();
    } else {
        printf("GPS processing halted due to insufficient switch count.\r\n");
    }

//    HAL_Delay(500); // 控制循环频率
				}
				memset(rx_buffer,0,sizeof(rx_buffer));
				HAL_UART_Receive(&huart3, rx_buffer, sizeof(rx_buffer), 500); // 接收数据

        // 检查是否接收到 "start" 指令
				if (strstr((char *)rx_buffer, "start") != NULL) 
				{
						motor_in_progress = 0;
						motor_completed = 0;
            printf("System ReStarting operations...\r\n");
        } 
				else if (strstr((char *)rx_buffer, "stop") != NULL) 
				{
          motor_completed = 0;  
					printf("System Stoped!\r\n");
        }

		processGpsData();
				 
		}

}////
/**
  * @brief System Clock Configuration
  * @retval None
  */

void ProcessDHT11Data(void)
{
    // 处理 DHT11 传感器1 数据
    if (dht11_1_updated)
    {
        dht11_1_updated = 0;
        if (DHT11_Read_Data(GPIOB, GPIO_PIN_12, (uint8_t *)&system_sensor_data.humi1, (uint8_t *)&system_sensor_data.temp1) != 0)
        {
            system_sensor_data.humi1 = 0;
            system_sensor_data.temp1 = 0;
//            HAL_UART_Transmit(&huart3, (uint8_t *)"Failed to read DHT11 on PIN 12\r\n", 32, 1000);
        }
//        else
//        {
//            sprintf(buffer, "%d%%  %d°C\r\n", system_sensor_data.humi1, system_sensor_data.temp1);
//            HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1000);
//        }
    }

    // 处理 DHT11 传感器2 数据
    if (dht11_2_updated)
    {
        dht11_2_updated = 0;
        if (DHT11_Read_Data(GPIOB, GPIO_PIN_13, (uint8_t *)&system_sensor_data.humi2, (uint8_t *)&system_sensor_data.temp2) != 0)
        {
            system_sensor_data.humi2 = 0;
            system_sensor_data.temp2 = 0;
//            HAL_UART_Transmit(&huart3, (uint8_t *)"Failed to read DHT11 on PIN 13\r\n", 32, 1000);
        }
//        else
//        {
//            sprintf(buffer, "%d%%  %d°C\r\n", system_sensor_data.humi2, system_sensor_data.temp2);
//            HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1000);
//        }
    }
}

void ProcessADCData(void)
{
    // 如果ADC转换完成
    if (adc_dma_complete)
    {
        adc_dma_complete = 0;

        // 将ADC数据存储到 sensor_data 中
        for (int i = 0; i < 5; i++)
        {
            system_sensor_data.adc_values[i] = adc_Value[i];
        }

       
    }
}

void ProcessIMUData(void)
{
    // 检查串口接收是否完成
    if (usart2_rx_complete)
    {
        usart2_rx_complete = false;

        // 处理IMU数据
        JY931_Process();

        // 更新姿态数据
        system_sensor_data.pitch = JY931_getPitch();
        system_sensor_data.roll = JY931_getRoll();
        system_sensor_data.yaw = JY931_getYaw();


    }
}

void OutputGpsData(void)
{
	if (motor_in_progress == 1) {
        return; // 直接返回，不执行后续代码
    }
    char buffer[100];

    // 格式化GPS数据为字符串
    snprintf(buffer, sizeof(buffer), 
             "GPS Position: Latitude: %s %c, Longitude: %s %c, Time: %s\r\n",
             system_sensor_data.gps.latitude, system_sensor_data.gps.lat_direction,
             system_sensor_data.gps.longitude, system_sensor_data.gps.lon_direction,
             system_sensor_data.gps.time);

    // 通过串口（例如 UART4）发送数据
    HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 1000);
}


#define REMOVAL_THRESHOLD 3300.0   // 被拆除的阈值
#define TEMPERATURE 16.0
#define EARTHQUAKE_THRESHOLD 8.0  // 用于区分地震的阈值
#define WATER_FLOW_THRESHOLD 1.0   // 用于区分水流的阈值


void integrate_and_fuse_data(void) {
    float features[NUM_FEATURES];
    uint32_t current_time = HAL_GetTick();
    // 将系统传感器数据转化为分类器可以处理的特征数组
    features[0] = system_sensor_data.humi1;    // DHT11 传感器1湿度
    features[1] = system_sensor_data.temp1;    // DHT11 传感器1温度
    features[2] = system_sensor_data.humi2;    // DHT11 传感器2湿度
    features[3] = system_sensor_data.temp2;    // DHT11 传感器2温度
    features[4] = system_sensor_data.pitch;    // IMU Pitch
    features[5] = system_sensor_data.roll;     // IMU Roll
    features[6] = system_sensor_data.yaw;      // IMU Yaw
    features[7] = system_sensor_data.adc_values[0];  // ADC 采集的第1路
    features[8] = system_sensor_data.adc_values[1];  // ADC 采集的第2路
    features[9] = system_sensor_data.adc_values[2];  // ADC 采集的第3路
    features[10] = system_sensor_data.adc_values[3]; // ADC 采集的第4路
    features[11] = system_sensor_data.adc_values[4]; // ADC 采集的第5路
	

    // 生成串口输出字符串
    char buffer[200];
    int len = snprintf(buffer, sizeof(buffer),
        "Sensor Data: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
        "%.2f, %.2f, %.2f, %.2f, %.2f\r\n",
        features[0], features[1], features[2], features[3], features[4], features[5], features[6],
        features[7], features[8], features[9], features[10], features[11]);

    // 通过串口发送数据
    printf("%s\n", buffer);  // 使用 printf 输出串口数据
		
		if ((double)features[7] < REMOVAL_THRESHOLD || (double)features[8] < REMOVAL_THRESHOLD) {
        printf("Warning: System has been removed!\r\n");
    }
		
		if ((double)features[1] > TEMPERATURE || (double)features[3] > TEMPERATURE) {
        printf("Warning: System temperature changes!\r\n");
    }
		    if ((double)features[1] > 40 || (double)features[3] > 40) {
        printf("报警: 耐压舱过温！\n");
    }

//    // 耐压舱姿态失稳报警
//    if ((double)features[4] < -5 || (double)features[4] > 5 || (double)features[5] < -5 || (double)features[5] > 5) {
//        printf("报警: 耐压舱姿态失稳！\n");
//    }

    // 耐压舱形变预警
if ((double)features[11] < 100) {
    printf("预警: 应变数据异常！\n");
} else if ((double)features[11] < 1500 || (double)features[11] > 2400) {
    printf("预警: 耐压舱形变！\n");
}

		
		 // 姿态数据变化判断逻辑
    static float last_pitch = 0.0;
    static float last_roll = 0.0;
    static float last_yaw = 0.0;
		
    // 计算当前姿态角度变化
    float pitch_change = fabs(features[4] - last_pitch);
    float roll_change = fabs(features[5] - last_roll);
    float yaw_change = fabs(features[6] - last_yaw);
		
		 // 更新上次的姿态角度
    last_pitch = features[4];
    last_roll = features[5];
    last_yaw = features[6];
		
		    // 判断是地震还是水流
    if ((double)pitch_change > EARTHQUAKE_THRESHOLD || (double)roll_change > EARTHQUAKE_THRESHOLD || (double)yaw_change > EARTHQUAKE_THRESHOLD) {
        // 如果姿态变化超过地震阈值，判断为地震
        printf("Detected: Earthquake! Pitch change: %.2f, Roll change: %.2f, Yaw change: %.2f\r\n", pitch_change, roll_change, yaw_change);
    } else if ((double)pitch_change > WATER_FLOW_THRESHOLD || (double)roll_change > WATER_FLOW_THRESHOLD || (double)yaw_change > WATER_FLOW_THRESHOLD) {
        // 如果变化在一定范围内，判断为水流
        printf("Detected: Water Flow! Pitch change: %.2f, Roll change: %.2f, Yaw change: %.2f\r\n", pitch_change, roll_change, yaw_change);
    }
}


//// 定义阈值和时间间隔
//#define MIN_CHANGE_THRESHOLD 0.5   // 姿态变化小于0.5度
//#define STABLE_PERIOD 10000         // 5秒，单位是毫秒
//#define MIN_DEPTH_CHANGE_THRESHOLD 0.001  // 深度计变化小于0.001

//// 全局变量用于记录上一次姿态和时间
//static float last_pitch = 0.0;
//static float last_roll = 0.0;
//static float last_yaw = 0.0;
//static uint32_t last_time = 0;  // 上次数据更新的时间

//extern char status;

//void check_sensor_stability_and_restart_motor(void) {
//    // 获取当前时间
//    uint32_t current_time = HAL_GetTick();

//    // 姿态数据的当前变化
//    float pitch_change = fabs(system_sensor_data.pitch - last_pitch);
//    float roll_change = fabs(system_sensor_data.roll - last_roll);
//    float yaw_change = fabs(system_sensor_data.yaw - last_yaw);

//    // 如果变化小于阈值，且时间超过设定的稳定期
//    if ((double)pitch_change < MIN_CHANGE_THRESHOLD && (double)roll_change < MIN_CHANGE_THRESHOLD && (double)yaw_change < MIN_CHANGE_THRESHOLD && 
//        (fabs(system_sensor_data.pitch) > 1 || fabs(system_sensor_data.roll) > 1)) {
//        if ((current_time - last_time > STABLE_PERIOD)) {
//            // 数据变化小，且时间超过设定的稳定期，重新启动调平
//            motor_in_progress = 0;  // 重置标志位
//            delay_completed = 1;    // 标记延时已完成
//        }
//    } else {
//        // 如果姿态有明显变化，更新记录的数据和时间
//        last_pitch = system_sensor_data.pitch;
//        last_roll = system_sensor_data.roll;
//        last_yaw = system_sensor_data.yaw; 
//        last_time = current_time;
//    }
//}

void SystemClock_Config(void)
	{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int c,FILE *stream)  //重写printf函数
{
		HAL_UART_Transmit(&huart3,(unsigned char *)&c,1,1000);
		return 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
