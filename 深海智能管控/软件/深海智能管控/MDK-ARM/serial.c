//#include "serial.h"
//#include <string.h>  // 用于strlen等字符串操作
//#include "usart.h"   // 包含用于UART通信的头文件
//#include <stdio.h>

//// 保留实际的serial_getchar函数实现
//char serial_getchar(void) {
//    char c;
//    HAL_UART_Receive(&huart1, (uint8_t *)&c, 1, HAL_MAX_DELAY);  // 使用UART1作为示例
//    return c;
//}

//void serial_write(const char *str) {
//    // 假设串口是通过 HAL 库配置的
//    while (*str) {
//        // 调用 UART 发送函数，将数据发送出去
//        HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
//        str++;
//    }

//    // 添加调试信息，确认数据被写入串口
//    printf("Sent through UART: %s\n", str);
//}


//void serial_init(void) {
//    // 使用 HAL 库初始化串口的示例
//    // 假设使用 UART1 串口

//    // 可能需要配置具体的 UART 初始化参数
//    MX_USART1_UART_Init();  // 如果使用 STM32 的 HAL 库
//}

//void serial_readline(char *buffer, size_t max_length) {
//    size_t i = 0;
//    char c;

//    while (i < max_length - 1) {
//        c = serial_getchar();

//        if (c == '\n' || c == '\r') {
//            break;
//        }

//        buffer[i++] = c;
//    }

//    buffer[i] = '\0';  // 确保字符串以null字符结尾
//}
