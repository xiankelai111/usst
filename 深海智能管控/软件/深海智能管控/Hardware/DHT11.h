#ifndef _DHT11_H_
#define _DHT11_H_


void Delay_us(uint16_t delay);                   //ͨ��TIM3��ʱ��΢�뼶��ʱ
void Dht11_DATA_OUT(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                       //�������ݽ�����Ϊ���
void Dht11_DATA_IN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                        //�������ݽ�����Ϊ����
void DHT11_Rst(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                            //��λDHT11
uint8_t DHT11_Check(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                       //DHT11״̬����
uint8_t DHT11_Read_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                    //��DHT11һλ����
uint8_t DHT11_Read_Byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                   //��DHT11һ�ֽ�����
uint8_t DHT11_Read_Data(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* humi,uint8_t* temp);     //DHT11������ʾ


#endif 
