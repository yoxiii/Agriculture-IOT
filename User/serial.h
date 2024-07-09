/*
 * serial.h
 *
 *  Created on: 2024Äê7ÔÂ2ÈÕ
 *      Author: YOXI
 */

#ifndef __SERIAL_H_
#define __SERIAL_H_

#include "sht31_reg.h"
#include "output.h"
extern uint16_t Serial_RxPacket[17];
extern u8 rxbuf_no_crc[16];
extern SHT31_Data sht31_data;

typedef struct {
    double temperature;
    double ph;
    double water;
    double elec;
} soil_Data;
extern soil_Data soil_data1 ;
extern soil_Data soil_data2 ;
extern soil_Data soil_data3 ;

void Serial_SendArray(uint8_t *Array, uint16_t Length);
uint16_t crc16(uint8_t *addr, uint8_t num);
void USARTy_CFG(void);
int16_t get_tempx10(void);
void USART6_CFG(void);
void USART4_CFG(void);
void DMA_INIT(void);
FlagStatus uartWriteWiFiStr(char * str);
uint32_t uartReadWiFi(char * buffer, uint16_t num);
char uartReadByteWiFi();
uint16_t uartAvailableWiFi();
void send_AT_command(void);
void send_sensor_data();
double get_soil_temp(void);
double get_soil_water(void);
double get_soil_elec(void);
double get_soil_ph(void);
void get_soil_data();

uint8_t crc_veri(void) ;
void print_output(void) ;
char get_esp_data1();

#endif /* USER_SERIAL_H_ */
