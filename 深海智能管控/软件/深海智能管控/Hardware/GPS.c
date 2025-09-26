#include "gps.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>  // ���� fabs() ����

extern int closed_switch_count;
void OutputGpsData(void);   // ���� OutputGpsData ����
void parseRMC(char *rmc_data);  // ���� parseRMC ����

// ȫ�ֱ�������
Data Save_Data;
GPSData gpsData;
SensorData sensor_data;
uint16_t point1 = 0;
uint8_t gps_buffer[UART4_MAX_RECV_LEN];
volatile bool gps_data_ready = false; // ��־λ��ָʾ����׼����

float last_latitude = 0.0f;
float last_longitude = 0.0f;
bool gps_moving = false;   // ��־λ����ʾ�豸�Ƿ��ƶ�

// ������ֵ
#define LARGE_MOVEMENT_THRESHOLD 0.01f   // ����仯��ֵ����Ϊ�豸�ڱ�����
int large_movement_count = 0;             // ���ڼ���������仯

void GPS_Init(void) {
    if (closed_switch_count > 0) {  
        HAL_Delay(200);
        HAL_UART_Receive_IT(&huart4, gps_buffer, UART4_MAX_RECV_LEN);
        printf("GPS single receive initiated.\n");
    } else {
        HAL_UART_AbortReceive(&huart4);  // ��������������ʱֹͣ����
        printf("GPS UART receive aborted due to insufficient switch count.\r\n");
    }
}



void parseGpsBuffer(void) {
    if (strlen(Save_Data.GPS_Buffer) == 0) {
        return;
    }

    char *nmea_sentence = strtok(Save_Data.GPS_Buffer, "\n");
    while (nmea_sentence != NULL) 
		{
        if (strstr(nmea_sentence, "GNRMC") != NULL) 
				{
            parseRMC(nmea_sentence);
        }
        nmea_sentence = strtok(NULL, "\n");
    }

    // ��ջ������������ظ�������ͬ�� GPS ����
    memset(Save_Data.GPS_Buffer, 0, sizeof(Save_Data.GPS_Buffer));
}
//void parseGpsBuffer() {
//    char buffer_copy[UART4_MAX_RECV_LEN];
//    strncpy(buffer_copy, Save_Data.GPS_Buffer, sizeof(buffer_copy) - 1);
//    buffer_copy[sizeof(buffer_copy) - 1] = '\0'; // ȷ���� '\0' ��β

//    char *token = strtok(buffer_copy, ",");
//    int count = 0;

//    char time[7] = {0};
//    char date[7] = {0};
//    char latitude[10] = {0};
//    char longitude[11] = {0};
//    char lat_direction = 0;
//    char lon_direction = 0;

//    while (token != NULL) {
//        count++;

//        switch (count) {
//            case 2:
//                strncpy(time, token, 6);
//                time[6] = '\0';
//                break;
//            case 10:
//                strncpy(date, token, 6);
//                date[6] = '\0';
//                break;
//            case 4:
//                if (sizeof(token) > 1) strncpy(latitude, token, sizeof(token) - 1);
//                latitude[sizeof(token) - 1] = '\0';
//                break;
//            case 5:
//                lat_direction = token[0];
//                break;
//            case 6:
//                if (sizeof(token) > 1) strncpy(longitude, token, sizeof(token) - 1);
//                longitude[sizeof(token) - 1] = '\0';
//                break;
//            case 7:
//                lon_direction = token[0];
//                break;
//            default:
//                break;
//        }
//        token = strtok(NULL, ",");
//    }

//    // Store parsed data into global variable
//    strcpy(gpsData.latitude, latitude);
//    strcpy(gpsData.longitude, longitude);
//    strcpy(gpsData.time, time);
//    strcpy(gpsData.date, date);
//    gpsData.lat_direction = lat_direction;
//    gpsData.lon_direction = lon_direction;
//}

char status = 'V';
void parseRMC(char *rmc_data) {
//    if (strlen(rmc_data) < 30) 
//		{
//        return;
//    }
    char time[11] = "000000.000";

    char lat[10] = "";
    char lat_dir = ' ';
    char lon[11] = "";
    char lon_dir = ' ';
    char date[7] = "000000";

    sscanf(rmc_data, "$GNRMC,%10[^,],%c,%9[^,],%c,%10[^,],%c,,,,%6[^,]",
           time, &status, lat, &lat_dir, lon, &lon_dir, date);

    char output[100];

    if (status == 'A') 
			{
        snprintf(output, sizeof(output),
                 "$GNRMC,%s,A,%s,%c,%s,%c,,,,%s,,,N,A*00\r\n",
                 time, lat, lat_dir, lon, lon_dir, date);
        printf("Valid RMC data: %s\n", output);

        // ����γ��ת��Ϊ������
        float current_latitude = atof(lat);
        float current_longitude = atof(lon);

        // ���㵱ǰ��γ�ȵı仯
        float lat_change = fabs(current_latitude - last_latitude);
        float lon_change = fabs(current_longitude - last_longitude);
        
        // Ĭ����Ϊ�豸�ڳ�����
        printf("GPS Status: ������ (On the Sea Surface)\r\n");
        gps_moving = false;

        // �ж��Ƿ��г����Ĵ���仯����Ϊ�豸�ڱ�����
        if (lat_change > LARGE_MOVEMENT_THRESHOLD || lon_change > LARGE_MOVEMENT_THRESHOLD) {
            large_movement_count++;
            if (large_movement_count >= 3) {  // �������仯��������3�Σ���Ϊ�豸������
                printf("GPS Status: ������ (Being Transported)\r\n");
                gps_moving = true;
            }
        } else {
            // ���û�д���仯�����ü�����
            large_movement_count = 0;
        }

        // ������һ�εľ�γ��
        last_latitude = current_latitude;
        last_longitude = current_longitude;
    } 
		else 
		{
        snprintf(output, sizeof(output),
                 "$GNRMC,%s,V,,,,,,,071024,,,N,V*00\r\n",
                 time);
        printf("Invalid RMC data: %s\n", output);
    }
}



bool gps_invalid_output_flag = false;  // ��־λ�����ڿ�����Ч���ݵ��ظ����
int gps_invalid_counter = 0;           // ������������ʵ����ȴ�ڻ���
const int GPS_COOLDOWN_PERIOD = 10;    // ��ȴ�ڳ��ȣ���λ��ѭ��������

void processGpsData(void) {
    // ���ˮ��������������0����δ����GPS��ȡ���򴥷�
    if (closed_switch_count > 0 && !gps_reading_triggered) 
		{
        gps_reading_triggered = true;  // ����GPS��ȡ990
    }

    // ����Ѿ�����GPS��ȡ����������GPS����
    if (gps_reading_triggered) {
        // ����־λ��ȷ��GPS����׼���ú��ٽ���
        if (gps_data_ready) 
				{
            gps_data_ready = false;  //00000000000000000001 00�����־λ
            parseGpsBuffer();        // ����GPS����

            // ���ݽ�������ж������Ƿ���Ч
            if (gpsData.status == 'V') 
						{
                if (!gps_invalid_output_flag || gps_invalid_counter >= GPS_COOLDOWN_PERIOD) 
								{
                    printf("Invalid RMC data received. GPS signal not locked.\n");
                    gps_invalid_output_flag = true;  // ������Ч���ݱ�־
                    gps_invalid_counter = 0;         // ���ü�����
                } 
								else 
								{
                    gps_invalid_counter++;  // ��ȴ�ڼ���
                }
            } 
						else if (gpsData.status == 'A') 
						{
                OutputGpsData();  // �����ЧGPS����
                gps_invalid_output_flag = false;  // �����Ч���ݱ�־
                gps_invalid_counter = 0;          // ���ü�����
            }
        }
    }
}





