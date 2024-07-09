/*
 * output.c
 *
 *  Created on: 2024年7月6日
 *      Author: YOXI
 */
#include "debug.h"
#include "output.h"
#include <string.h>
OUTPUT output={0,0,0,0,0,0,0,0,0,0};

void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_15;   //DRV8313 pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_3;   //FAN and PUMP
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}
void PWM_setcompare(OUTPUT output)
{
    TIM_SetCompare2(TIM1, output.pump1*10);  //pe11
    TIM_SetCompare3(TIM1, output.pump2*10);  //pe13
    TIM_SetCompare4(TIM1, output.pump3*10);  //pe14

    TIM_SetCompare2(TIM10, output.fan1*10);  //pd3
    TIM_SetCompare1(TIM9, output.fan2*10);  //pd9
    TIM_SetCompare2(TIM9, output.fan3*10);  //pd11
    TIM_SetCompare3(TIM9, output.fan4*10);  //pd13
}

void output_push(void)
{
        GPIO_WriteBit(GPIOE, GPIO_Pin_15, 1) ; //drv8313 en
        PWM_setcompare(output);


//        GPIO_WriteBit(GPIOD, GPIO_Pin_3, output.fan1) ;  //(i == 0) ? (i = Bit_SET) : (i = Bit_RESET)
//        GPIO_WriteBit(GPIOD, GPIO_Pin_9, output.fan2) ;
//        GPIO_WriteBit(GPIOD, GPIO_Pin_11, output.fan3) ;
//        GPIO_WriteBit(GPIOD, GPIO_Pin_13, output.fan4) ;


        GPIO_WriteBit(GPIOE, GPIO_Pin_2, output.heat1);
        GPIO_WriteBit(GPIOE, GPIO_Pin_3, output.heat2);
        GPIO_WriteBit(GPIOE, GPIO_Pin_4, output.heat3);
}
uint32_t combineFront5(OUTPUT output) {
    uint32_t combined = 0;
    combined |= (output.pump1 & 0xF) << 16;
    combined |= (output.pump2 & 0xF) << 12;
    combined |= (output.pump3 & 0xF) << 8;
    combined |= (output.fan1 & 0xF) << 4;
    combined |= (output.fan2 & 0xF);
    return combined;
}
uint32_t combineBack5(OUTPUT output) {
    uint32_t combined = 0;
    combined |= (output.fan3 & 0xF) << 16;
    combined |= (output.fan4 & 0xF) << 12;
    combined |= (output.heat1 & 0xF) << 8;
    combined |= (output.heat2 & 0xF) << 4;
    combined |= (output.heat3 & 0xF);
    return combined;
}

void combine_output_to_msb_lsb(const OUTPUT *output, unsigned int *msb, unsigned int *lsb) {
    char num_buffer[11]; // 10个数字字符 + 1个终止符

    // 将结构体成员值转换为字符并存入字符串
    snprintf(num_buffer, sizeof(num_buffer), "1%d%d%d%d%d%d%d%d%d%d",
             output->pump1, output->pump2, output->pump3, output->fan1,
             output->fan2, output->fan3, output->fan4, output->heat1,
             output->heat2, output->heat3);

    // 将字符串拆分成两个部分
    char msb_str[6]; // 5个字符 + 1个终止符
    char lsb_str[6]; // 5个字符 + 1个终止符

    strncpy(msb_str, num_buffer, 5);
    msb_str[5] = '\0';
    strncpy(lsb_str, num_buffer + 5, 5);
    lsb_str[5] = '\0';

    // 将这两个部分转换为整型数
    *msb = atoi(msb_str);
    *lsb = atoi(lsb_str);
}
OUTPUT output_pull(void)
{
    OUTPUT output1;
    output1.heat1=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
    output1.heat2=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
    output1.heat3=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
    return output1;
}
void TIM1drv_PWMOut_Init( u16 arr, u16 psc, u16 ccp )
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE | RCC_APB2Periph_TIM1|RCC_APB2Periph_AFIO, ENABLE );

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
//    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
//    GPIO_PinRemapConfig(GPIO_FullRemap_TIM9, ENABLE);
//    GPIO_PinRemapConfig(GPIO_FullRemap_TIM10, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOE, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init( GPIOE, &GPIO_InitStructure );

//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_13;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_Init( GPIOD, &GPIO_InitStructure );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

//    TIM_TimeBaseInit( TIM9, &TIM_TimeBaseInitStructure);
//    TIM_TimeBaseInit( TIM10, &TIM_TimeBaseInitStructure);
//    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init( TIM1, &TIM_OCInitStructure );
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

//    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
//    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
//    TIM_OC2Init(TIM10, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
       TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
       TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;
       TIM_OCInitStructure.TIM_Pulse = ccp;
       TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
       TIM_OC2Init(TIM9, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE );

//    TIM_CtrlPWMOutputs(TIM9, ENABLE );
//    TIM_CtrlPWMOutputs(TIM10, ENABLE );

    TIM_OC2PreloadConfig( TIM1, TIM_OCPreload_Disable );

//    TIM_OC2PreloadConfig( TIM9, TIM_OCPreload_Disable );
//    TIM_OC1PreloadConfig( TIM4, TIM_OCPreload_Disable );
//    TIM_OC2PreloadConfig( TIM4, TIM_OCPreload_Disable );

    TIM_ARRPreloadConfig( TIM1, ENABLE );
//    TIM_ARRPreloadConfig( TIM4, ENABLE );
//    TIM_ARRPreloadConfig( TIM9, ENABLE );
//    TIM_ARRPreloadConfig( TIM10, ENABLE );
    TIM_Cmd( TIM1, ENABLE );
//    TIM_Cmd( TIM4, ENABLE );
//    TIM_Cmd( TIM9, ENABLE );
//    TIM_Cmd( TIM10, ENABLE );

}

void TIM9drv_PWMOut_Init( u16 arr, u16 psc )
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD|RCC_APB2Periph_TIM9|RCC_APB2Periph_AFIO, ENABLE );

   GPIO_PinRemapConfig(GPIO_FullRemap_TIM9, ENABLE);
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
//    GPIO_PinRemapConfig(GPIO_FullRemap_TIM9, ENABLE);
//    GPIO_PinRemapConfig(GPIO_FullRemap_TIM10, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_9|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );


    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM9, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init( TIM9, &TIM_OCInitStructure );
    TIM_OC3Init(TIM9, &TIM_OCInitStructure);
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
       TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
       TIM_OCInitStructure.TIM_Pulse = 0;
       TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_CtrlPWMOutputs(TIM9, ENABLE );

    TIM_OC2PreloadConfig( TIM9, TIM_OCPreload_Disable );


    TIM_ARRPreloadConfig( TIM9, ENABLE );

    TIM_Cmd( TIM9, ENABLE );


}

void TIM10drv_PWMOut_Init( u16 arr, u16 psc )
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD| RCC_APB2Periph_TIM10|RCC_APB2Periph_AFIO, ENABLE );

   GPIO_PinRemapConfig(GPIO_FullRemap_TIM10, ENABLE);
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
//    GPIO_PinRemapConfig(GPIO_FullRemap_TIM9, ENABLE);
//    GPIO_PinRemapConfig(GPIO_FullRemap_TIM10, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );


    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM10, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init( TIM10, &TIM_OCInitStructure );


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
       TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
       TIM_OCInitStructure.TIM_Pulse = 0;
       TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_CtrlPWMOutputs(TIM10, ENABLE );

    TIM_OC2PreloadConfig( TIM10, TIM_OCPreload_Disable );


    TIM_ARRPreloadConfig( TIM10, ENABLE );

    TIM_Cmd( TIM10, ENABLE );


}
void PWM_Init(void)
{
    TIM1drv_PWMOut_Init(100, 48000 - 1,50);
    TIM9drv_PWMOut_Init(100, 48000 - 1);
    TIM10drv_PWMOut_Init(100, 48000 - 1);
}



