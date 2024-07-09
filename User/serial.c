/*
 * serial.c
 *
 *  Created on: 2024年7月2日
 *      Author: YOXI
 */

#include "debug.h"
#include "sht31_reg.h"
#include "serial.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define RXBUF_SIZE 1024 // DMA buffer size
soil_Data soil_data1 = { };
soil_Data soil_data3 = { };
soil_Data soil_data2 = { };
uint16_t Serial_RxPacket[17];
uint8_t Serial_RxData;
uint16_t buf_size = 0;
uint16_t TR = 0;
uint8_t intra_buf[18];
uint8_t Serial_RxFlag = 1;
uint8_t intra_buf_count;
u8 TxBuffer[] = " ";
u8 RxBuffer[RXBUF_SIZE] = { 0 };
SHT31_Data sht31_data;
static uint8_t RxState = 0;
#define BUFLEN 256      //数组缓存大小
char atstr[BUFLEN];
char atstr1[BUFLEN];
double temp = 0.00;

void Serial_SendByte(uint8_t Byte) {
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}
uint32_t Serial_Pow(uint32_t X, uint32_t Y) {
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}
void Serial_SendNumber(uint32_t Number, uint8_t Length) {
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}
void Serial_SendArray(uint8_t *Array, uint16_t Length) {
    uint16_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

void USARTy_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    USART_InitTypeDef USART_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //| RCC_APB1Periph_USART3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // | RCC_APB2Periph_GPIOB

    /* USART2 TX-->A.2   RX-->A.3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* USART3 TX-->B.10  RX-->B.11 */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

//    USART_Init(USART3, &USART_InitStructure);
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
//    USART_Cmd(USART3, ENABLE);
}

void USART4_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    USART_InitTypeDef USART_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //| RCC_APB1Periph_USART3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); // | RCC_APB2Periph_GPIOB

    /* USART2 TX-->A.2   RX-->A.3 */

    GPIO_PinRemapConfig(GPIO_FullRemap_USART4, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(UART4, &USART_InitStructure);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

//    USART_Init(USART3, &USART_InitStructure);
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

//    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(UART4, ENABLE);
//    USART_Cmd(USART3, ENABLE);
}

void DMA_INIT(void) {
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    // TX DMA 初始化
    DMA_DeInit(DMA2_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) (&UART6->DATAR); // DMA 外设基址，需指向对应的外设
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) TxBuffer; // DMA 内存基址，指向发送缓冲区的首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // 方向 : 外设 作为 终点，即 内存 ->  外设
    DMA_InitStructure.DMA_BufferSize = 0;         // 缓冲区大小,即要DMA发送的数据长度,目前没有数据可发
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址自增，禁用
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         // 内存地址自增，启用
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据位宽，8位(Byte)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 内存数据位宽，8位(Byte)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;             // 普通模式，发完结束，不循环发送
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;             // 优先级最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                    // M2P,禁用M2M
    DMA_Init(DMA2_Channel6, &DMA_InitStructure);

    // RX DMA 初始化，环形缓冲区自动接收
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) RxBuffer;              // 接收缓冲区
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // 方向 : 外设 作为 源，即 内存 <- 外设
    DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;          // 缓冲区长度为 RXBUF_SIZE
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;              // 循环模式，构成环形缓冲区
    DMA_Init(DMA2_Channel7, &DMA_InitStructure);
}

void USART6_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    //开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* USART6 TX-->C0  RX-->C1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX，输入上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;                    // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 数据位 8
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 停止位 1
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无校验
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //使能 RX 和 TX

    USART_Init(UART6, &USART_InitStructure);
    DMA_Cmd(DMA2_Channel7, ENABLE);                                  //开启接收 DMA
    USART_Cmd(UART6, ENABLE);                                        //开启UART
//    DMA_INIT();
//    USART_DMACmd(UART6,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);
}

/*******************************************************************************
 * Function Name  :  uartWriteWiFi
 * Description    :  send data to ESP8266 via UART6
 * 描述    ：   向 WiFi 模组发送数据
 * Input          :  char * data          data to send   要发送的数据的首地址
 *                   uint16_t num         number of data 数据长度
 * Return         :  RESET                UART6 busy,failed to send  发送失败
 *                   SET                  send success               发送成功
 *******************************************************************************/
FlagStatus uartWriteWiFi(char * data, uint16_t num) {
    //如上次发送未完成，返回
    if (DMA_GetCurrDataCounter(DMA2_Channel6) != 0) {
        return RESET;
    }

    DMA_ClearFlag(DMA2_FLAG_TC8);
    DMA_Cmd(DMA2_Channel6, DISABLE);           // 关 DMA 后操作
    DMA2_Channel6->MADDR = (uint32_t) data;      // 发送缓冲区为 data
    DMA_SetCurrDataCounter(DMA2_Channel6, num);  // 设置缓冲区长度
    DMA_Cmd(DMA2_Channel6, ENABLE);             // 开 DMA
    return SET;
}

/*******************************************************************************
 * Function Name  :  uartWriteWiFiStr
 * Description    :  send string to ESP8266 via UART6    向 WiFi 模组发送字符串
 * Input          :  char * str          string to send
 * Return         :  RESET                UART busy,failed to send   发送失败
 *                   SET                  send success               发送成功
 *******************************************************************************/
FlagStatus uartWriteWiFiStr(char * str) {
    uint16_t num = 0;
    while(str[num])num++;
    // 计算字符串长度
    return uartWriteWiFi(str, num);
}

/*******************************************************************************
 * Function Name  :  uartReadWiFireceive
 * Description    :  read some bytes from receive buffer 从接收缓冲区读出一组数据
 * Input          :  char * buffer        buffer to storage the data 用来存放读出数据的地址
 *                   uint16_t num         number of data to read     要读的字节数
 * Return         :  int                  number of bytes read       返回实际读出的字节数
 *******************************************************************************/
uint16_t rxBufferReadPos = 0;       //接收缓冲区读指针
uint32_t uartReadWiFi(char * buffer, uint16_t num) {
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7); //计算 DMA 数据尾的位置
    uint16_t i = 0;
    if (rxBufferReadPos == rxBufferEnd) {
        // 无数据，返回
        return 0;
    }

    while (rxBufferReadPos!=rxBufferEnd && i < num) {
        buffer[i] = RxBuffer[rxBufferReadPos];
        i++;
        rxBufferReadPos++;
        if(rxBufferReadPos >= RXBUF_SIZE) {
            // 超出缓冲区，回零
            rxBufferReadPos = 0;
        }
    }
    return i;
}

/*******************************************************************************
 * Function Name  :  uartReadByteWiFi
 * Description    :  read one byte from UART buffer  从接收缓冲区读出 1 字节数据
 * Input          :  None
 * Return         :  char    read data               返回读出的数据(无数据也返回0)
 *******************************************************************************/
char uartReadByteWiFi() {
    char ret;
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7);
    if (rxBufferReadPos == rxBufferEnd) {
        // 无数据，返回
        return 0;
    }
    ret = RxBuffer[rxBufferReadPos];
    rxBufferReadPos++;
    if (rxBufferReadPos >= RXBUF_SIZE) {
        // 超出缓冲区，回零
        rxBufferReadPos = 0;
    }
    return ret;
}
/*******************************************************************************
 * Function Name  :  uartAvailableWiFi
 * Description    :  get number of bytes Available to read from the UART buffer  获取缓冲区中可读数据的数量
 * Input          :  None
 * Return         :  uint16_t    number of bytes Available to read               返回可读数据数量
 *******************************************************************************/
uint16_t uartAvailableWiFi() {
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel7); //计算 DMA 数据尾的位置
    // 计算可读字节
    if (rxBufferReadPos <= rxBufferEnd) {
        return rxBufferEnd - rxBufferReadPos;
    } else {
        return rxBufferEnd + RXBUF_SIZE - rxBufferReadPos;
    }
}

/********************************************************************
 * 函 数 名       : TIM6_Init
 * 函数功能    : 初始化 定时器 TIM6
 * 输    入          : arr：计数值，psc 预分频系数
 * 输    出          : 无
 ********************************************************************/

void send_AT_command(void) {
    while(uartWriteWiFiStr("AT+RST\r\n")==RESET);
    Delay_Ms(2000);
//    uartWriteWiFi("AT\r\n",4);
//       Delay_Ms(100);
    // 查询 打开AT回显
    while(uartWriteWiFi("ATE0\r\n",6)==RESET);
    Delay_Ms(100);
    // 设为 Station 模式
    while(uartWriteWiFiStr("AT+CWMODE=1\r\n")==RESET);
    Delay_Ms(100);
    // 复位WiFi模块

    // 设为单连接模式
    //    while(uartWriteWiFiStr("AT+CIPMUX=0\r\n")==RESET);
    //    Delay_Ms(100);
    // 连接一个名为 SSID、密码为 PASSWORD 的 WiFi 网络，

//    while(uartWriteWiFiStr("AT+CIPSTART=\"TCP\",\"192.168.137.1\",2345\r\n")==RESET);
//    Delay_Ms(500);
//    while(uartWriteWiFiStr("AT+CIPSEND=11111\r\n")==RESET);
//    Delay_Ms(500);

    while(uartWriteWiFiStr("AT+CWJAP=\"DAP\",\"12345678\"\r\n")==RESET);
    Delay_Ms(2000);


    while(uartWriteWiFiStr("AT+MQTTUSERCFG=0,1,\"NULL\",\"esp8266&hy3hFRQHfVP\",\"f2c7c793aaef628f1404d68453fbde446607e7160ea2c40be5c0959fddc60415\",0,0,\"\"\r\n")==RESET);
    Delay_Ms(2000);
    while(uartWriteWiFiStr("AT+MQTTCLIENTID=0,\"hy3hFRQHfVP.esp8266|securemode=2\\,signmethod=hmacsha256\\,timestamp=1719623253312|\"\r\n")==RESET);
    Delay_Ms(100);
    while(uartWriteWiFiStr("AT+MQTTCONN=0,\"iot-06z009wdg1cy5kg.mqtt.iothub.aliyuncs.com\",1883,1\r\n")==RESET);
    Delay_Ms(2000);
    while(uartWriteWiFiStr("AT+MQTTSUB=0,\"/sys/hy3hFRQHfVP/esp8266/thing/service/property/set\",1\r\n")==RESET);
    Delay_Ms(500);



//              Delay_Ms(3000);
//              int num = uartAvailableWiFi();
//                  if (num > 0 ){
//                      char buffer[1024]={"\0"};
//                      uartReadWiFi(buffer , num);                //读取数据
//                      printf("Revceived:\r\n%s",buffer);}
}

void send_sht31_data() {
    sprintf(atstr,
            "AT+MQTTPUB=0,\"/hy3hFRQHfVP/esp8266/user/esp8266\",\"{\\\"LightSwitch\\\":%.2f}\",1,0\r\n",
            sht31_data.temperature);

    while(uartWriteWiFiStr(atstr)==RESET);
    Delay_Ms(500);
    while(uartWriteWiFiStr("AT+MQTTPUB=0,\"/hy3hFRQHfVP/esp8266/user/esp8266\",\"{\\\"tempdata\\\":20}\",1,0\r\n")==RESET);
    // Delay_Ms(1000);
}
void send_sensor_data() {
    uint32_t output_MSB, output_LSB;
    combine_output_to_msb_lsb(&output, &output_MSB, &output_LSB);
    static SHT31_Data sht31_data_send;
    if (sht31_data.temperature != 0 & sht31_data.humidity != 0) {
        sht31_data_send = sht31_data;
    }

    //sprintf(atstr,"AT+MQTTPUB=0,\"/hy3hFRQHfVP/esp8266/user/esp8266\",\"{\\\"params\\\":{\\\"temperature\\\":%.2f\\,\\\"humi\\\":%.2f}}\",1,0\r\n",sht31_data.temperature,sht31_data.humidity);
    sprintf(atstr,
            "AT+MQTTPUB=0,\"/hy3hFRQHfVP/esp8266/user/esp8266\",\"{\\\"params\\\":{\\\"temperature\\\":%.2f\\,\\\"humi\\\":%.2f\\,\\\"soil_temp\\\":%.2f\\,\\\"soil_water\\\":%.2f\\,\\\"soil_ph\\\":%.2f\\,\\\"soil_elec\\\":%.2f\\,\\\"Switch\\\":%d}}\",1,0\r\n",
            sht31_data_send.temperature, sht31_data_send.humidity,
            soil_data1.temperature, soil_data1.water, soil_data1.ph,
            soil_data1.elec, output_MSB, output_LSB, output.pump3, output.fan1,
            output.fan2, output.fan3, output.fan4, output.heat1, output.heat2,
            output.heat3);
    while(uartWriteWiFiStr(atstr)==RESET);
    Delay_Ms(50);
//              while(uartWriteWiFiStr("AT+MQTTPUB=0,\"/hy3hFRQHfVP/esp8266/user/esp8266\",\"{\\\"params\\\":{\\\"temperature\\\":62.5\\,\\\"humi\\\":66}}\",1,0\r\n")==RESET);
//                      Delay_Ms(200);

//              while(uartWriteWiFiStr("AT+MQTTPUB=0,\"/hy3hFRQHfVP/esp8266/user/esp8266\",\"{\\\"tempdata\\\":22}\",1,0\r\n")==RESET);
//              Delay_Ms(200);
//              while(uartWriteWiFiStr("AT+MQTTPUB=0,\"/hy3hFRQHfVP/esp8266/user/esp8266\",\"{\\\"tempdata\\\":33}\",1,0\r\n")==RESET);
    //Delay_Ms(50);
    // Delay_Ms(1000);   //延时不得大于定时器
}

//uint16_t crc16(uint8_t *data, size_t length) {
//    uint16_t crc = 0xFFFF; // 置16位寄存器为全1
//
//    for (size_t i = 0; i < length; i++) {
//        crc ^= data[i]; // 把一个8位数据与16位CRC寄存器的低字节相异或
//
//        for (int j = 0; j < 8; j++) {
//            if (crc & 0x0001) { // 检查最低位
//                crc = (crc >> 1) ^ 0xA001; // 如果最低位为1，CRC寄存器与多项式A001H进行异或
//            } else {
//                crc >>= 1; // 如果最低位为0，右移一位
//            }
//        }
//    }
//
//    // 交换CRC-16寄存器的低字节和高字节
//    crc = (crc >> 8) | (crc << 8);
//
//    return crc;
//}

void print_output(void) {
    printf("pump1: %d\n", output.pump1);
    printf("pump2: %d\n", output.pump2);
    printf("pump3: %d\n", output.pump3);
    printf("fan1: %d\n", output.fan1);
    printf("fan2: %d\n", output.fan2);
    printf("fan3: %d\n", output.fan3);
    printf("fan4: %d\n", output.fan4);
    printf("heat1: %d\n", output.heat1);
    printf("heat2: %d\n", output.heat2);
    printf("heat3: %d\n", output.heat3);
}
void get_esp_data(OUTPUT *output) {
    char *begin1;
    char output_Str[20] = { "\0" };
    unsigned char num_buffer[21] = { 0 };
    uint32_t MSB, LSB;
    char msb_str[6] = { 0 };  // 5 digits + null terminator
    char lsb_str[6] = { 0 };  // 5 digits + null terminator
    int num = uartAvailableWiFi();
    //printf("devices:%d \r\n", num_buffer);
    if (num > 0) {
        char buffer[1024] = { "\0" };
        uartReadWiFi(buffer, num);
        printf("Revceived:\r\n%s", buffer);
        if (num > 7 & strstr(buffer, "Switch") != NULL) {
            begin1 = strstr(buffer, "Switch");

            strncpy(msb_str, begin1 + 9, 5);
            printf(begin1 + 8);
            msb_str[6] = '\0';
            MSB = atoi(msb_str);

            strncpy(lsb_str, begin1 + 14, 5);
            lsb_str[5] = '\0';
            LSB = atoi(lsb_str);

            printf("MSB: %d\n", MSB);
            printf("LSB: %d\n", LSB);

            snprintf(num_buffer, sizeof(num_buffer), "%05u%05u", MSB, LSB);
            printf("num buffer:\r\n");
            for (int i = 0; i < 11; i++)
                printf("%0d ", num_buffer[i]);
            printf("\r\n");

            output->pump1 = num_buffer[0] - '0';
            output->pump2 = num_buffer[1] - '0';
            output->pump3 = num_buffer[2] - '0';
            output->fan1 = num_buffer[3] - '0';
            output->fan2 = num_buffer[4] - '0';
            output->fan3 = num_buffer[5] - '0';
            output->fan4 = num_buffer[6] - '0';
            output->heat1 = num_buffer[7] - '0';
            output->heat2 = num_buffer[8] - '0';
            output->heat3 = num_buffer[9] - '0';
            output_push();
            print_output();
        }
        printf("devices:");
        for (int i = 0; i < num; i++) {
            printf("%d ", num_buffer[i]);
        }
        printf("\r\n");

    }
}

char get_esp_data1(void) {
    char *begin1;
    //char output_Str[20] = { "\0" };
    unsigned char num_buffer[21] = { 0 };
    int num = uartAvailableWiFi();
    //printf("devices:%d \r\n", num_buffer);
    if (num > 0) {
        char buffer[1024] = { "\0" };
        uartReadWiFi(buffer, num);
        //printf("Revceived:\r\n%s", buffer);
        if (num > 7 & strstr(buffer, "Switch") != NULL) {
            begin1 = strstr(buffer, "Switch");
            char switch_str[11] = { 0 };  // 10 digits + null terminator

            // Extract the digits after "Switch"
            snprintf(switch_str, sizeof(switch_str), "%s", begin1 + 10); // +10 to skip "Switch":""
            //printf(switch_str);
            // Assign extracted digits to output struct members
            output.pump1 = switch_str[0] - '0';
            output.pump2 = switch_str[1] - '0';
            output.pump3 = switch_str[2] - '0';
            output.fan1 = switch_str[3] - '0';
            output.fan2 = switch_str[4] - '0';
            output.fan3 = switch_str[5] - '0';
            output.fan4 = switch_str[6] - '0';
            output.heat1 = switch_str[7] - '0';
            output.heat2 = switch_str[8] - '0';
            output.heat3 = switch_str[9] - '0';
            print_output();
            return switch_str;
        }
    }
}

uint16_t crc16(uint8_t *addr, uint8_t num) {
    int i, j, temp;
    u16 crc = 0xFFFF;
    for (i = 0; i < num; i++) {
        crc = crc ^ (*addr);
        for (j = 0; j < 8; j++) {
            temp = crc & 0x0001;
            crc = crc >> 1;
            if (temp) {
                crc = crc ^ 0xA001;
            }
        }
        addr++;
    }
    crc = (crc >> 8) | (crc << 8);
    return crc;
}
uint8_t crc_veri(void) {
    u16 crc_2byte;
    u8 rx_no_crc[20] = { 0 };
    for (int loop = 0; loop < 11; loop++) {
        rx_no_crc[loop] = Serial_RxPacket[loop];
    }
    crc_2byte = (u16) (Serial_RxPacket[11] << 8) | Serial_RxPacket[12];

//    printf("crc_2byte:%2X\r\n",crc_2byte);
//    printf("last_2byte:%2X %2X\r\n",Serial_RxPacket[11],Serial_RxPacket[12]);
//
//                for (int i = 0; i < 17; i++)
//                                             printf("%02X ", Serial_RxPacket[i]);
//                printf("\r\n");
//                for (int i = 0; i < 17; i++)
//                                                 printf("%02X ", rx_no_crc[i]);
//
//                printf("\r\n");

    // printf("crc16:%2X\r\n",crc16(rx_no_crc, 11));
    if (crc_2byte == crc16(rx_no_crc, 11))
        //printf("crc16:%2X\r\n",crc16(data, 11));
        return 1;
    else
        return 0;
}
void get_soil_data() {
    if (crc_veri()) {
        //printf("rxstate:%d\r\n", RxState);
        if (RxState == 0) {
            soil_data1.temperature = get_soil_temp();
            soil_data1.elec = get_soil_elec();
            soil_data1.ph = get_soil_ph();
            soil_data1.water = get_soil_water();
        }
        if (RxState == 1) {
            soil_data2.temperature = get_soil_temp();
            soil_data2.elec = get_soil_elec();
            soil_data2.ph = get_soil_ph();
            soil_data2.water = get_soil_water();
        }
        if (RxState == 2) {
            soil_data3.temperature = get_soil_temp();
            soil_data3.elec = get_soil_elec();
            soil_data3.ph = get_soil_ph();
            soil_data3.water = get_soil_water();
        }
    }
}
int16_t get_tempx10(void) {

//    u16 crc_2byte;
//    u16 rx_no_crc[20];
//    for(int loop = 0; loop < 10; loop++) {
//        rx_no_crc[loop] = Serial_RxPacket[loop];
//       }
//    crc_2byte=(u16)(Serial_RxPacket[10]<<8)|Serial_RxPacket[11];
//    printf("crc_2byte:%2X\r\n",crc_2byte);
//
//    for (int i = 0; i < 17; i++)
//                                 printf(" %02X ", rx_no_crc[i]);
//                             printf("\r\n");
//                             printf("crc16:%4X\r\n",crc16(rx_no_crc, 10));
//    if(crc16(rx_no_crc, 10)==crc_2byte)
//    {

    if (Serial_RxPacket[5] == 0xFF)
        return ((Serial_RxPacket[5] << 8) | Serial_RxPacket[6]) - 0xFF;
    else {
        return (Serial_RxPacket[5] << 8) | Serial_RxPacket[6];
    }
//    }
//    else {
//
//        return crc_2byte;
//    }
}

double get_soil_temp(void) {

    double temp = 0.00;
    if (Serial_RxPacket[5] == 0xFF) {
        temp = (((Serial_RxPacket[5] << 8) | Serial_RxPacket[6]) - 0xFF) / 10.0;
        return temp;
    } else {
        temp = ((Serial_RxPacket[5] << 8) | Serial_RxPacket[6]) / 10.0;
        return temp;
    }

}
double get_soil_water(void) {
    double water = 0.00;
    water = ((Serial_RxPacket[3] << 8) | Serial_RxPacket[4]) / 10.0;
    return water;
}

double get_soil_elec(void) {
    double elec = 0.00;
    elec = ((Serial_RxPacket[7] << 8) | Serial_RxPacket[8]) / 10.0;
    return elec;
}

double get_soil_ph(void) {
    double ph = 0.00;
    ph = ((Serial_RxPacket[9] << 8) | Serial_RxPacket[10]) / 10.0;
    return ph;
}

//soil_Data get_soil_data(void)
//{
//    soil_Data soli_data={0.0,0.0,0.0,0.0};
//    soli_data.temperature=get_soil_temp();
//    soli_data.elec=get_soil_elec();
//    soli_data.ph=get_soil_ph();
//    soli_data.water=get_soil_water();
//    return soli_data;
//}

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void UART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) {

    //uint8_t len_intra=0;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
        Serial_RxData = USART_ReceiveData(USART2);
        USART_SendData(USART2, USART_ReceiveData(USART2));
        Serial_RxPacket[buf_size++] = Serial_RxData;
        if (Serial_RxPacket[buf_size - 2] == 0x01
                && Serial_RxPacket[buf_size - 1] == 0x03) {
            RxState = 0;
            Serial_RxPacket[buf_size++] = Serial_RxData;
            buf_size = 2;

            Serial_RxPacket[0] = 0x01;
            Serial_RxPacket[1] = 0x03;

        } else if (Serial_RxPacket[buf_size - 2] == 0x02
                && Serial_RxPacket[buf_size - 1] == 0x03) {
            RxState = 1;
            Serial_RxPacket[buf_size++] = Serial_RxData;
            buf_size = 2;

            Serial_RxPacket[0] = 0x02;
            Serial_RxPacket[1] = 0x03;
        }
        if (Serial_RxPacket[buf_size - 2] == 0x03
                && Serial_RxPacket[buf_size - 1] == 0x03) {
            RxState = 2;
            Serial_RxPacket[buf_size++] = Serial_RxData;
            buf_size = 2;

            Serial_RxPacket[0] = 0x03;
            Serial_RxPacket[1] = 0x03;
        }
//        if(buf_size>12)
//        {
//            buf_size=0;
//        }

        //USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void UART4_IRQHandler(void) {

    //uint8_t len_intra=0;

    if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET) {
        USART_SendData(UART4, USART_ReceiveData(UART4));
        USART_SendData(USART2, USART_ReceiveData(UART4));
        Serial_RxPacket[buf_size++] = Serial_RxData;
        //USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}


