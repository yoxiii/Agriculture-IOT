/*
 * output.h
 *
 *  Created on: 2024Äê7ÔÂ6ÈÕ
 *      Author: YOXI
 */


#ifndef USER_OUTPUT_C_
#define USER_OUTPUT_C_

void GPIO_Toggle_INIT(void);

typedef struct{
    u8 pump1;
    u8 pump2;
    u8 pump3;
    u8 fan1;
    u8 fan2;
    u8 fan3;
    u8 fan4;
    u8 heat1;
    u8 heat2;
    u8 heat3;
}OUTPUT;
extern OUTPUT output;
void output_push(void);
OUTPUT output_pull(void);
uint32_t combinef5(OUTPUT output) ;
uint32_t combineFront5(OUTPUT output) ;
uint32_t combineBack5(OUTPUT output);
void TIM1drv_PWMOut_Init( u16 arr, u16 psc, u16 ccp );
void TIM9drv_PWMOut_Init( u16 arr, u16 psc );
void TIM10drv_PWMOut_Init( u16 arr, u16 psc );
void PWM_Init(void);

#endif /* USER_OUTPUT_C_ */
