#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define UART4_MAX_RECV_LEN 512
#define USART_MAX_SEND_LEN 512
#define GPS_Buffer_Length 512

typedef struct {
    char GPS_Buffer[GPS_Buffer_Length];
    bool isGetData;
} Data;

typedef struct {
    int hour;
    int minute;
    int second;
} Time;

typedef struct {
    char time[11];          // ʱ���ֶ�
    char latitude[10];      // γ���ֶ�
    char lat_direction;     // γ�ȷ���N/S��
    char longitude[11];     // �����ֶ�
    char lon_direction;     // ���ȷ���E/W��
    char date[7];           // �����ֶ�
    char status;            // ״̬��A����Ч����V����Ч��
} GPSData;


typedef struct {
    uint16_t adc_values[5];
    uint8_t humi1, temp1;
    uint8_t humi2, temp2;
    uint8_t humi3, temp3;
    char gps_buffer[UART4_MAX_RECV_LEN];
    GPSData gps_data;
} SensorData;

extern Data Save_Data;
extern SensorData sensor_data;
extern uint8_t gps_buffer[UART4_MAX_RECV_LEN];
extern uint8_t UartRxData;
extern uint16_t point1;
extern GPSData gpsData;

void convertUtcToLocalTime(const Time *utcTime, int timeZoneOffset, Time *localTime);
void parseGpsBuffer(void);
void printfGpsBuffer(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void startReceivingData(void);
void GPS_Init(void);
void processGpsData(void);
#endif // GPS_H

