#include "gps.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>  // 用于 fabs() 函数

extern int closed_switch_count;
void OutputGpsData(void);   // 声明 OutputGpsData 函数
void parseRMC(char *rmc_data);  // 声明 parseRMC 函数

// 全局变量定义
Data Save_Data;
GPSData gpsData;
SensorData sensor_data;
uint16_t point1 = 0;
uint8_t gps_buffer[UART4_MAX_RECV_LEN];
volatile bool gps_data_ready = false; // 标志位，指示数据准备好

float last_latitude = 0.0f;
float last_longitude = 0.0f;
bool gps_moving = false;   // 标志位，表示设备是否移动

// 定义阈值
#define LARGE_MOVEMENT_THRESHOLD 0.01f   // 大幅变化阈值，认为设备在被运输
int large_movement_count = 0;             // 用于检测持续大幅变化

void GPS_Init(void) {
    if (closed_switch_count > 0) {  
        HAL_Delay(200);
        HAL_UART_Receive_IT(&huart4, gps_buffer, UART4_MAX_RECV_LEN);
        printf("GPS single receive initiated.\n");
    } else {
        HAL_UART_AbortReceive(&huart4);  // 当开关数量不足时停止接收
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

    // 清空缓冲区，避免重复解析相同的 GPS 数据
    memset(Save_Data.GPS_Buffer, 0, sizeof(Save_Data.GPS_Buffer));
}
//void parseGpsBuffer() {
//    char buffer_copy[UART4_MAX_RECV_LEN];
//    strncpy(buffer_copy, Save_Data.GPS_Buffer, sizeof(buffer_copy) - 1);
//    buffer_copy[sizeof(buffer_copy) - 1] = '\0'; // 确保以 '\0' 结尾

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

        // 将经纬度转换为浮点数
        float current_latitude = atof(lat);
        float current_longitude = atof(lon);

        // 计算当前经纬度的变化
        float lat_change = fabs(current_latitude - last_latitude);
        float lon_change = fabs(current_longitude - last_longitude);
        
        // 默认认为设备在出海面
        printf("GPS Status: 出海面 (On the Sea Surface)\r\n");
        gps_moving = false;

        // 判断是否有持续的大幅变化，认为设备在被运输
        if (lat_change > LARGE_MOVEMENT_THRESHOLD || lon_change > LARGE_MOVEMENT_THRESHOLD) {
            large_movement_count++;
            if (large_movement_count >= 3) {  // 如果大幅变化持续超过3次，认为设备被运输
                printf("GPS Status: 被运输 (Being Transported)\r\n");
                gps_moving = true;
            }
        } else {
            // 如果没有大幅变化，重置计数器
            large_movement_count = 0;
        }

        // 更新上一次的经纬度
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



bool gps_invalid_output_flag = false;  // 标志位，用于控制无效数据的重复输出
int gps_invalid_counter = 0;           // 计数器，用于实现冷却期机制
const int GPS_COOLDOWN_PERIOD = 10;    // 冷却期长度（单位：循环次数）

void processGpsData(void) {
    // 如果水银开关数量大于0且尚未触发GPS读取，则触发
    if (closed_switch_count > 0 && !gps_reading_triggered) 
		{
        gps_reading_triggered = true;  // 触发GPS读取990
    }

    // 如果已经触发GPS读取，继续处理GPS数据
    if (gps_reading_triggered) {
        // 检查标志位，确保GPS数据准备好后再解析
        if (gps_data_ready) 
				{
            gps_data_ready = false;  //00000000000000000001 00清除标志位
            parseGpsBuffer();        // 解析GPS数据

            // 根据解析结果判断数据是否有效
            if (gpsData.status == 'V') 
						{
                if (!gps_invalid_output_flag || gps_invalid_counter >= GPS_COOLDOWN_PERIOD) 
								{
                    printf("Invalid RMC data received. GPS signal not locked.\n");
                    gps_invalid_output_flag = true;  // 设置无效数据标志
                    gps_invalid_counter = 0;         // 重置计数器
                } 
								else 
								{
                    gps_invalid_counter++;  // 冷却期计数
                }
            } 
						else if (gpsData.status == 'A') 
						{
                OutputGpsData();  // 输出有效GPS数据
                gps_invalid_output_flag = false;  // 清除无效数据标志
                gps_invalid_counter = 0;          // 重置计数器
            }
        }
    }
}





