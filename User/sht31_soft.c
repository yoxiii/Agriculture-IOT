#include "sht31_soft.h"
#include "debug.h"
#include "MyI2C.h"
#include "IIC.h"
#define SHT31_ADDRESS 0x44


void sht31_soft_Init()
{
    MyI2C_Init();
    sht31soft_WriteReg(0x2737);
}



void sht31soft_WriteReg(uint16_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(SHT31_ADDRESS<<1);
	MyI2C_ReceiveAck();
	MyI2C_SendByte((uint8_t)(Data>>8));
	MyI2C_ReceiveAck();
	MyI2C_SendByte((uint8_t)(Data &= 0xFF));
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

uint8_t sht31_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
    uint32_t Timeout;
    Timeout = 2000;
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != READY)
    {
        Timeout --;
        if (Timeout == 0)
        {
            return 0;
        }
    }
    return 1;
}

void sht31_WriteReg(uint16_t Data)
{
//    MyI2C_Start();
//    MyI2C_SendByte(SHT31_ADDRESS<<1);
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte((uint8_t)(Data>>8));
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte((uint8_t)(Data &= 0xFF));
//    MyI2C_ReceiveAck();
//    MyI2C_Stop();
        I2C_AcknowledgeConfig( I2C2, ENABLE );
        I2C_GenerateSTART( I2C2, ENABLE );
        sht31_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
        I2C_Send7bitAddress(I2C2,((SHT31_ADDRESS <<1)|0),I2C_Direction_Transmitter);
        sht31_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
        I2C_SendData(I2C2,(uint8_t)(Data>>8));
        sht31_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        I2C_SendData(I2C2,(uint8_t)(Data &= 0xFF));
        sht31_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        I2C_GenerateSTOP( I2C2, ENABLE );
}

void sht31_Init()
{
    IIC_Init(20000, 0x02);
    sht31_WriteReg(0x2737);
}

SHT31_Data sht31soft_ReadTempHumi(void)
{
	SHT31_Data sht31_data;
	uint8_t Data[6];
	uint8_t i;
	MyI2C_Start();
		MyI2C_SendByte(SHT31_ADDRESS<<1);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(0xE0);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(0x00);
	MyI2C_ReceiveAck();
		MyI2C_Start();
	MyI2C_SendByte((SHT31_ADDRESS<<1)+1);
	MyI2C_ReceiveAck();
	for(i=0;i<6;i++)
    {
			Data[i]=MyI2C_ReceiveByte();
			if(i==0&Data[i]==0xFF)
				break;
			if(i==5)
        {
             MyI2C_SendAck(1); //发送非应答信号
            break;
        }
				MyI2C_SendAck(0);
		}

	MyI2C_Stop();
	// 解析温度数据
    uint16_t temp_raw = (Data[0] << 8) | Data[1];
    sht31_data.temperature = -45 + 175 * (temp_raw / 65535.0);

    // 解析湿度数据
    uint16_t hum_raw = (Data[3] << 8) | Data[4];
    sht31_data.humidity = 100 * (hum_raw / 65535.0);
	return sht31_data;
}


SHT31_Data sht31_ReadTempHumi(void)
{
    SHT31_Data sht31_data;
    uint8_t Data[6];
    uint8_t i;
//    MyI2C_Start();
//        MyI2C_SendByte(SHT31_ADDRESS<<1);
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte(0xE0);
//    MyI2C_ReceiveAck();
//    MyI2C_SendByte(0x00);
//    MyI2C_ReceiveAck();
//        MyI2C_Start();
//    MyI2C_SendByte((SHT31_ADDRESS<<1)+1);
//    MyI2C_ReceiveAck();
//    for(i=0;i<6;i++)
//    {
//            Data[i]=MyI2C_ReceiveByte();
//            if(i==0&Data[i]==0xFF)
//                break;
//            if(i==5)
//        {
//             MyI2C_SendAck(1); //发送非应答信号
//            break;
//        }
//                MyI2C_SendAck(0);
//        }
//
//    MyI2C_Stop();
            u8 rx_ready_flag=1;
            I2C_AcknowledgeConfig( I2C2, ENABLE );
            I2C_GenerateSTART( I2C2, ENABLE );
            sht31_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

            I2C_Send7bitAddress(I2C2,((SHT31_ADDRESS <<1)),I2C_Direction_Transmitter);
            sht31_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

            I2C_SendData(I2C2, 0xE0);
            sht31_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);

            I2C_SendData(I2C2, 0x00);
            sht31_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);

            I2C_GenerateSTART( I2C2, ENABLE );
            sht31_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

            I2C_Send7bitAddress(I2C2,((SHT31_ADDRESS <<1)|0x01),I2C_Direction_Receiver);
            if(sht31_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!=1)
//            if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != READY)
                {
                    I2C_GenerateSTOP( I2C2, ENABLE );
                     return sht31_data;
                }


            for(i=0;i<6;i++)
                {
                if(i==5)
                {
                    I2C_AcknowledgeConfig( I2C2, DISABLE );//发送非应答信号
                    break;
                }
                    I2C_AcknowledgeConfig( I2C2, ENABLE );
                    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_RXNE ) ==  RESET );
                    Data[i]=I2C_ReceiveData(I2C2);
                    if(Data[0]==0xFF)
                        break;


                    }


            I2C_GenerateSTOP( I2C2, ENABLE );

    // 解析温度数据
    uint16_t temp_raw = (Data[0] << 8) | Data[1];
    sht31_data.temperature = -45 + 175 * (temp_raw / 65535.0);

    // 解析湿度数据
    uint16_t hum_raw = (Data[3] << 8) | Data[4];
    sht31_data.humidity = 100 * (hum_raw / 65535.0);
    return sht31_data;
}

/********************************************************************
* 函 数 名       : TIM6_Init
* 函数功能    : 初始化 定时器 TIM6
* 输    入          : arr：计数值，psc 预分频系数
* 输    出          : 无
********************************************************************/
void TIM6_Init( uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM6, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit( TIM6, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_ARRPreloadConfig( TIM6, ENABLE );
    TIM_Cmd( TIM6, ENABLE );
}

/********************************************************************
* 函 数 名       : Interrupt_Init
* 函数功能    : 初始化定时器中断
* 输    入          : 无
* 输    出          : 无
********************************************************************/
void Interrupt_Init(void)
{
   NVIC_InitTypeDef NVIC_InitStructure={0};
   NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //子优先级
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

