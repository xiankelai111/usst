//#include "serial.h"
//#include <string.h>  // ����strlen���ַ�������
//#include "usart.h"   // ��������UARTͨ�ŵ�ͷ�ļ�
//#include <stdio.h>

//// ����ʵ�ʵ�serial_getchar����ʵ��
//char serial_getchar(void) {
//    char c;
//    HAL_UART_Receive(&huart1, (uint8_t *)&c, 1, HAL_MAX_DELAY);  // ʹ��UART1��Ϊʾ��
//    return c;
//}

//void serial_write(const char *str) {
//    // ���贮����ͨ�� HAL �����õ�
//    while (*str) {
//        // ���� UART ���ͺ����������ݷ��ͳ�ȥ
//        HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
//        str++;
//    }

//    // ��ӵ�����Ϣ��ȷ�����ݱ�д�봮��
//    printf("Sent through UART: %s\n", str);
//}


//void serial_init(void) {
//    // ʹ�� HAL ���ʼ�����ڵ�ʾ��
//    // ����ʹ�� UART1 ����

//    // ������Ҫ���þ���� UART ��ʼ������
//    MX_USART1_UART_Init();  // ���ʹ�� STM32 �� HAL ��
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

//    buffer[i] = '\0';  // ȷ���ַ�����null�ַ���β
//}
