#ifndef _DHT11_H_
#define _DHT11_H_


void Delay_us(uint16_t delay);                   //通过TIM3定时器微秒级延时
void Dht11_DATA_OUT(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                       //设置数据交互口为输出
void Dht11_DATA_IN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                        //设置数据交互口为输入
void DHT11_Rst(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                            //复位DHT11
uint8_t DHT11_Check(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                       //DHT11状态反馈
uint8_t DHT11_Read_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                    //读DHT11一位数据
uint8_t DHT11_Read_Byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);                   //读DHT11一字节数据
uint8_t DHT11_Read_Data(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* humi,uint8_t* temp);     //DHT11数据显示


#endif 
