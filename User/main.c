#include "debug.h"
#include "lcd.h"
#include "IIC.h"
#include "serial.h"
#include "sht31_soft.h"
#include "AP3216C.h"
#include "output.h"

#define REPORT_TIME_MS  100

uint8_t soil1_detect_command[9] =
        { 0x01, 0x03, 0x0, 0x0, 0x0, 0x04, 0x44, 0x09 }; //44 09      44 3A       45 EB
uint8_t soil2_detect_command[9] =
        { 0x02, 0x03, 0x0, 0x0, 0x0, 0x04, 0x44, 0x03A };
uint8_t soil3_detect_command[9] =
        { 0x03, 0x03, 0x0, 0x0, 0x0, 0x04, 0x45, 0xEB };
u8 soil_rx_flag = 10;

/*******************************************************************************
 * Function Name  : main
 * Description    : Main program.
 * Input          : None
 * Return         : None
 *******************************************************************************/
int main(void) {
    u16 ir, als, ps;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    //SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(9600);

    GPIO_Toggle_INIT();
    printf("SystemClk:%d\r\n", SystemCoreClock);

    PWM_Init();

    //USART6_CFG();
//    USARTy_CFG();
    lcd_init();
    DMA_INIT();
    USART6_CFG(); /* USART INIT */
    USART_DMACmd(UART6, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

    lcd_set_color(BLACK, WHITE);
    lcd_show_string(50, 0, 32, "ZhongYL");
    lcd_set_color(BLACK, WHITE);
    lcd_show_string(0, 48, 16, "AP3216C");

    uint8_t data[] = { 0x1, 0x3, 0x8, 0x0, 0x0, 0x1, 0xF, 0x0, 0x0, 0x0, 0x57 }; // 示例数据
    size_t length = sizeof(data) / sizeof(data[0]);

    uint16_t crc = crc16(data, 11);
    printf("CRC-16: %02X\n", crc);

    while (AP3216C_Init()) // 初始化AP3216C
    {
        delay_ms(REPORT_TIME_MS);
        lcd_set_color(BLACK, RED);
        lcd_show_string(180, 48, 16, "Error");
        Delay_Ms(200);
        lcd_show_string(180, 48, 16, "     ");
        Delay_Ms(200);
    }
    sht31_Init();

    lcd_set_color(BLACK, BLUE);
    lcd_show_string(180, 48, 16, "OK");
    send_AT_command();
    Delay_Ms(3000);
    int num = uartAvailableWiFi();
    if (num > 0) {
        char buffer[1024] = { "\0" };
        uartReadWiFi(buffer, num);
        printf("Revceived:\r\n%s", buffer);
    }
    USARTy_CFG();
    USART4_CFG();
    TIM6_Init(100 * REPORT_TIME_MS - 1, 9600 - 1); // 初始化定时器   f=1/t=(10^2 )*t
    Interrupt_Init(); //初始化定时器中断

    while (1)
    {

        // crc_verify();

        //get_tempx10();
        output_push();//输出io口
        if(soil_rx_flag!=0)
        {

            { // AP3216C
              // printf("read AP3216\r\n");
                AP3216C_ReadData(&ir, &ps, &als);// 读取数据
                lcd_set_color(BLACK, WHITE);
                lcd_show_string(0, 48, 16, "AP3216C");
                lcd_set_color(BLACK, CYAN);
                lcd_show_string(30, 64, 16, "IR : %5d", ir);// IR
                lcd_show_string(30, 80, 16, "PS : %5d", ps);// PS
                lcd_show_string(30, 96, 16, "ALS: %5d", als);// ALS
            }
            sht31_data=sht31_ReadTempHumi();

            if(sht31_data.temperature>10&sht31_data.temperature<45)
            {

                lcd_set_color(BLACK, GRED);
                lcd_show_num(10, 150, sht31_data.temperature, 2, 24);
                lcd_show_string(35,150,24,".");
                lcd_show_num(45, 150, (uint16_t)(sht31_data.temperature*100)%100, 2, 24);
            }
            if(sht31_data.humidity>0&sht31_data.humidity<100)
            {
                lcd_show_num(10, 200, sht31_data.humidity, 2, 24);
                lcd_show_string(35,200,24,".");
                lcd_show_num(45, 200, (uint16_t)(sht31_data.humidity*100)%100, 2, 24);
                // printf("sht31_data.temperature:%.2f\r\n",sht31_data.temperature);
            }
            lcd_show_string(30,112,16, "rx: %s",get_esp_data1());

             //printf("soil.temperature:%.1f soil.water:%.1f soil.ph:%.1f soil.elec:%.1f\r\n",soil_data3.temperature,soil_data3.water,soil_data3.ph,soil_data2.elec);

//            if(soil_data1.temperature>10&& soil_data1.temperature<45)
//            {
            lcd_set_color(BLACK, GRED);
            lcd_show_num(90, 100, soil_data1.temperature, 2, 24);
            lcd_show_string(115,100,24,".");
            lcd_show_num(135, 100, (uint16_t)(soil_data1.temperature*100)%100, 2, 24);

            lcd_set_color(BLACK, GRED);
            lcd_show_num(90, 130, soil_data2.temperature, 2, 24);
            lcd_show_string(115,130,24,".");
            lcd_show_num(135, 130, (uint16_t)(soil_data2.temperature*100)%100, 2, 24);

            lcd_set_color(BLACK, GRED);
            lcd_show_num(90, 160, soil_data3.temperature, 2, 24);
            lcd_show_string(115,160,24,".");
            lcd_show_num(135, 160, (uint16_t)(soil_data3.temperature*100)%100, 2, 24);
//            }

            get_soil_data();
            int num = uartAvailableWiFi();
//            printf("Serial_RxPacket: ");
//            for (int i = 0; i < 17; i++)
//            printf("%02X ", Serial_RxPacket[i]);
//            printf("\r\n");
            get_esp_data1();
            //print_output(&output);

            soil_rx_flag--;
        }
        else {
            Delay_Ms(100);
            Serial_SendArray(soil1_detect_command, 8);
            Delay_Ms(100);
//            Serial_SendArray(soil2_detect_command, 8);
//            Delay_Ms(100);
//            Serial_SendArray(soil3_detect_command, 8);
//            Delay_Ms(100);
            soil_rx_flag=3;

        }
    }
}

void TIM6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM6_IRQHandler(void) {
    TIM_ClearFlag(TIM6, TIM_FLAG_Update); //清除标志位
   send_sensor_data();

}
