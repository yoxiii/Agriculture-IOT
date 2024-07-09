#ifndef __SHT31_SOFT_H__
#define __SHT31_SOFT_H__
#include "sht31_reg.h"
#include <stdint.h>

void sht31_soft_Init();
void sht31_Init();
void sht31soft_WriteReg(uint16_t Data);
void sht31_WriteReg(uint16_t Data);
//uint16_t sht31soft_ReadReg(void);
SHT31_Data sht31soft_ReadTempHumi(void);
SHT31_Data sht31_ReadTempHumi(void);
void TIM6_Init( uint16_t arr, uint16_t psc);
void Interrupt_Init(void);


#endif
