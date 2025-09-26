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
		
	int closed_switch_count = 0;  // ����ȫ�ֱ���

/* USER CODE END Includes */
volatile uint8_t adc_dma_complete = 0;
volatile uint8_t dht11_1_updated = 0;
volatile uint8_t dht11_2_updated = 0;
float pitch, roll, yaw;
volatile uint32_t timer_ticks = 0;
extern volatile bool usart2_rx_complete;
uint8_t uart_buffer[512];
volatile int motor_in_progress = 0; // �����ƽ�Ƿ����ڽ���
volatile int delay_completed = 0; 
volatile int motor_completed = 0;    // �����ƽ�Ƿ���ɵı�־
volatile bool sensor_reading_started = false;
volatile bool pressure_sensor_triggered = false;
volatile bool gps_reading_triggered = false;
volatile bool system_enabled = false;


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    // ADC����
    unsigned short adc_values[5];
    uint8_t humi1, temp1;
    uint8_t humi2, temp2;
    // IMU��̬����
    float pitch;
    float roll;
    float yaw;
    // GPS���ݣ���Ҫ��ӵ��ֶΣ�
    GPSData gps;  // ��� GPS ���ݽṹ��
} SystemSensorData;

/* USER CODE END PTD */
volatile SystemSensorData system_sensor_data;

// ����ȫ�ִ��������ݱ���

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
	uint8_t rx_buffer[50] = {0}; // ��ʱ���ջ�����
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

 
    // �ȴ�����ָ��
    while (!system_enabled) {
        memset(rx_buffer,0,sizeof(rx_buffer));
        HAL_UART_Receive(&huart3, rx_buffer, sizeof(rx_buffer), 1000); // ��������

        // ����Ƿ���յ� "start" ָ��
        if (strstr((char *)rx_buffer, "start") != NULL) {
            system_enabled = true; // ����ϵͳ������־
            printf("System Enabled. Starting operations...\r\n");
        } else {
            printf("Waiting for 'start' command...\r\n");
        }

        HAL_Delay(500); // ���ƴ��ڼ���Ƶ��
    }

while (1) {
	 if (timer_ticks >= 1 && motor_in_progress == 0)  //timer_ticks������ʱ�ĳ��ȣ�20���ӵ���1200��
        {
					HAL_Delay(100);
					if (usart2_rx_complete) {
											usart2_rx_complete = false;
											JY931_Process();
										}  

            motor_in_progress = 1;   // ��ǵ�ƽ��ʼ
            delay_completed = 1;     // �����ʱ�����

                pitch = JY931_data.angle.angle[0];
                roll =  JY931_data.angle.angle[1];
                yaw =   JY931_data.angle.angle[2];                             
                           
            // �����ƽ���̣�������ͣ�����

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
                printf("Pitch: %f, Roll: %f, Yaw: %f\r\n", pitch, roll, yaw); // ��ȡʵʱ�Ĵ���������
                
                AdjustPlane(pitch, roll);            // �������
                HAL_Delay(500);                      // �ʵ���ʱ��ģ�����ķ�����ͣ
            }
            motor_completed = 1;  // ��ƽ��ɣ�������
						sensor_reading_started = false;
						pressure_sensor_triggered = false;
						gps_reading_triggered = false;
        }
	if (motor_completed == 1)
         {			
    // ���ˮ�����رպ�����
    closed_switch_count = MercurySwitch_Check();
    printf("Closed Switch Count: %d\r\n", closed_switch_count);

    // ֻ����ˮ�������������� 1 ʱ�Ŵ��� GPS ����
    if (!sensor_reading_started && closed_switch_count > 0) {
        // һ��ˮ��������������0���itʼ��ȡ����
        sensor_reading_started = true;
    }
		
    if (sensor_reading_started) {
        // ���� GPS ���ݺʹ���������

        ProcessDHT11Data();
        ProcessADCData();
        ProcessIMUData();
        integrate_and_fuse_data();
//        check_sensor_stability_and_restart_motor();
    } else {
        printf("GPS processing halted due to insufficient switch count.\r\n");
    }

//    HAL_Delay(500); // ����ѭ��Ƶ��
				}
				memset(rx_buffer,0,sizeof(rx_buffer));
				HAL_UART_Receive(&huart3, rx_buffer, sizeof(rx_buffer), 500); // ��������

        // ����Ƿ���յ� "start" ָ��
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
    // ���� DHT11 ������1 ����
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
//            sprintf(buffer, "%d%%  %d��C\r\n", system_sensor_data.humi1, system_sensor_data.temp1);
//            HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1000);
//        }
    }

    // ���� DHT11 ������2 ����
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
//            sprintf(buffer, "%d%%  %d��C\r\n", system_sensor_data.humi2, system_sensor_data.temp2);
//            HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 1000);
//        }
    }
}

void ProcessADCData(void)
{
    // ���ADCת�����
    if (adc_dma_complete)
    {
        adc_dma_complete = 0;

        // ��ADC���ݴ洢�� sensor_data ��
        for (int i = 0; i < 5; i++)
        {
            system_sensor_data.adc_values[i] = adc_Value[i];
        }

       
    }
}

void ProcessIMUData(void)
{
    // ��鴮�ڽ����Ƿ����
    if (usart2_rx_complete)
    {
        usart2_rx_complete = false;

        // ����IMU����
        JY931_Process();

        // ������̬����
        system_sensor_data.pitch = JY931_getPitch();
        system_sensor_data.roll = JY931_getRoll();
        system_sensor_data.yaw = JY931_getYaw();


    }
}

void OutputGpsData(void)
{
	if (motor_in_progress == 1) {
        return; // ֱ�ӷ��أ���ִ�к�������
    }
    char buffer[100];

    // ��ʽ��GPS����Ϊ�ַ���
    snprintf(buffer, sizeof(buffer), 
             "GPS Position: Latitude: %s %c, Longitude: %s %c, Time: %s\r\n",
             system_sensor_data.gps.latitude, system_sensor_data.gps.lat_direction,
             system_sensor_data.gps.longitude, system_sensor_data.gps.lon_direction,
             system_sensor_data.gps.time);

    // ͨ�����ڣ����� UART4����������
    HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 1000);
}


#define REMOVAL_THRESHOLD 3300.0   // ���������ֵ
#define TEMPERATURE 16.0
#define EARTHQUAKE_THRESHOLD 8.0  // �������ֵ������ֵ
#define WATER_FLOW_THRESHOLD 1.0   // ��������ˮ������ֵ


void integrate_and_fuse_data(void) {
    float features[NUM_FEATURES];
    uint32_t current_time = HAL_GetTick();
    // ��ϵͳ����������ת��Ϊ���������Դ������������
    features[0] = system_sensor_data.humi1;    // DHT11 ������1ʪ��
    features[1] = system_sensor_data.temp1;    // DHT11 ������1�¶�
    features[2] = system_sensor_data.humi2;    // DHT11 ������2ʪ��
    features[3] = system_sensor_data.temp2;    // DHT11 ������2�¶�
    features[4] = system_sensor_data.pitch;    // IMU Pitch
    features[5] = system_sensor_data.roll;     // IMU Roll
    features[6] = system_sensor_data.yaw;      // IMU Yaw
    features[7] = system_sensor_data.adc_values[0];  // ADC �ɼ��ĵ�1·
    features[8] = system_sensor_data.adc_values[1];  // ADC �ɼ��ĵ�2·
    features[9] = system_sensor_data.adc_values[2];  // ADC �ɼ��ĵ�3·
    features[10] = system_sensor_data.adc_values[3]; // ADC �ɼ��ĵ�4·
    features[11] = system_sensor_data.adc_values[4]; // ADC �ɼ��ĵ�5·
	

    // ���ɴ�������ַ���
    char buffer[200];
    int len = snprintf(buffer, sizeof(buffer),
        "Sensor Data: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
        "%.2f, %.2f, %.2f, %.2f, %.2f\r\n",
        features[0], features[1], features[2], features[3], features[4], features[5], features[6],
        features[7], features[8], features[9], features[10], features[11]);

    // ͨ�����ڷ�������
    printf("%s\n", buffer);  // ʹ�� printf �����������
		
		if ((double)features[7] < REMOVAL_THRESHOLD || (double)features[8] < REMOVAL_THRESHOLD) {
        printf("Warning: System has been removed!\r\n");
    }
		
		if ((double)features[1] > TEMPERATURE || (double)features[3] > TEMPERATURE) {
        printf("Warning: System temperature changes!\r\n");
    }
		    if ((double)features[1] > 40 || (double)features[3] > 40) {
        printf("����: ��ѹ�չ��£�\n");
    }

//    // ��ѹ����̬ʧ�ȱ���
//    if ((double)features[4] < -5 || (double)features[4] > 5 || (double)features[5] < -5 || (double)features[5] > 5) {
//        printf("����: ��ѹ����̬ʧ�ȣ�\n");
//    }

    // ��ѹ���α�Ԥ��
if ((double)features[11] < 100) {
    printf("Ԥ��: Ӧ�������쳣��\n");
} else if ((double)features[11] < 1500 || (double)features[11] > 2400) {
    printf("Ԥ��: ��ѹ���α䣡\n");
}

		
		 // ��̬���ݱ仯�ж��߼�
    static float last_pitch = 0.0;
    static float last_roll = 0.0;
    static float last_yaw = 0.0;
		
    // ���㵱ǰ��̬�Ƕȱ仯
    float pitch_change = fabs(features[4] - last_pitch);
    float roll_change = fabs(features[5] - last_roll);
    float yaw_change = fabs(features[6] - last_yaw);
		
		 // �����ϴε���̬�Ƕ�
    last_pitch = features[4];
    last_roll = features[5];
    last_yaw = features[6];
		
		    // �ж��ǵ�����ˮ��
    if ((double)pitch_change > EARTHQUAKE_THRESHOLD || (double)roll_change > EARTHQUAKE_THRESHOLD || (double)yaw_change > EARTHQUAKE_THRESHOLD) {
        // �����̬�仯����������ֵ���ж�Ϊ����
        printf("Detected: Earthquake! Pitch change: %.2f, Roll change: %.2f, Yaw change: %.2f\r\n", pitch_change, roll_change, yaw_change);
    } else if ((double)pitch_change > WATER_FLOW_THRESHOLD || (double)roll_change > WATER_FLOW_THRESHOLD || (double)yaw_change > WATER_FLOW_THRESHOLD) {
        // ����仯��һ����Χ�ڣ��ж�Ϊˮ��
        printf("Detected: Water Flow! Pitch change: %.2f, Roll change: %.2f, Yaw change: %.2f\r\n", pitch_change, roll_change, yaw_change);
    }
}


//// ������ֵ��ʱ����
//#define MIN_CHANGE_THRESHOLD 0.5   // ��̬�仯С��0.5��
//#define STABLE_PERIOD 10000         // 5�룬��λ�Ǻ���
//#define MIN_DEPTH_CHANGE_THRESHOLD 0.001  // ��ȼƱ仯С��0.001

//// ȫ�ֱ������ڼ�¼��һ����̬��ʱ��
//static float last_pitch = 0.0;
//static float last_roll = 0.0;
//static float last_yaw = 0.0;
//static uint32_t last_time = 0;  // �ϴ����ݸ��µ�ʱ��

//extern char status;

//void check_sensor_stability_and_restart_motor(void) {
//    // ��ȡ��ǰʱ��
//    uint32_t current_time = HAL_GetTick();

//    // ��̬���ݵĵ�ǰ�仯
//    float pitch_change = fabs(system_sensor_data.pitch - last_pitch);
//    float roll_change = fabs(system_sensor_data.roll - last_roll);
//    float yaw_change = fabs(system_sensor_data.yaw - last_yaw);

//    // ����仯С����ֵ����ʱ�䳬���趨���ȶ���
//    if ((double)pitch_change < MIN_CHANGE_THRESHOLD && (double)roll_change < MIN_CHANGE_THRESHOLD && (double)yaw_change < MIN_CHANGE_THRESHOLD && 
//        (fabs(system_sensor_data.pitch) > 1 || fabs(system_sensor_data.roll) > 1)) {
//        if ((current_time - last_time > STABLE_PERIOD)) {
//            // ���ݱ仯С����ʱ�䳬���趨���ȶ��ڣ�����������ƽ
//            motor_in_progress = 0;  // ���ñ�־λ
//            delay_completed = 1;    // �����ʱ�����
//        }
//    } else {
//        // �����̬�����Ա仯�����¼�¼�����ݺ�ʱ��
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
int fputc(int c,FILE *stream)  //��дprintf����
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
