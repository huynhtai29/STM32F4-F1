#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rtc.h"
#include "sensor.h"
extern void turnOnLED(uint32_t, int);
extern void turnOffLED(uint32_t, int);
uint16_t a = 7;
USART_TypeDef *huart2 = USART2;
uint8_t RxBuffer[31] = "nguye huynh tai";
char A[12];
//  Delays number of Systicks (happens every 1 ms)
uint8_t b = 0, c;
RTC_TimeTypeDef GET_TIME;
uint16_t distance;

// Delay ms
void delay(unsigned int time_out)
{
    unsigned int t1, t2;
    for (t1 = 0; t1 < time_out; t1++)
    {
        t2 = time_out;
        while (t2 > 0)
        {
            t2--;
            asm("nop");
        }
    }
}
void init_GPIO()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // ---------- GPIO  for LEDS -------- //
    // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Configure PD12, PD13, PD14 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // ---------- GPIO  for Push Button -------- //
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    //    /* Configure PA0 pin as input floating */
    //    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    //    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		// PA0 is connected to high, so use pulldown resistor
    //    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA2 is connected to high, so use pulldown resistor
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // PA3 is connected to high, so use pulldown resistor
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    // uart
    USART_DeInit(USART2);
    USART_InitStructure.USART_BaudRate = 115200; // Saniye içinde hat üzerinden kaç tane bit gönderilmesi gerektigi
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
    USART_LINBreakDetectLengthConfig(USART2, USART_LINBreakDetectLength_11b);
    USART_LINCmd(USART2, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    //	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_FE, ENABLE);
    // NVIC_UART
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;            // Function name for EXTI_Line0 interrupt handler
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // Set priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;        // Set sub priority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;              // Enable the interrupt
    NVIC_Init(&NVIC_InitStructure);

    /* Connect EXTI Line0 to PA0 pin (i.e. EXTI0CR[0])*/
    //    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    //    // SYSCFG->EXTICR[0] &= SYSCFG_EXTICR1_EXTI0_PA;		// Same as above, but with direct register access
    //
    //    /* Configure EXTI Line0 */
    //	EXTI_InitStructure.EXTI_Line = EXTI_Line0;			   // PA0 is connected to EXTI0
    //	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	// Interrupt mode
    //	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Trigger on Rising edge (Just as user presses btn)
    //	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			   // Enable the interrupt
    //	EXTI_Init(&EXTI_InitStructure);						   // Initialize EXTI
    //
    //	/* Enable and set priorities for the EXTI0 in NVIC */
    //	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			 // Function name for EXTI_Line0 interrupt handler
    //	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // Set priority
    //	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		 // Set sub priority
    //	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // Enable the interrupt
    //	NVIC_Init(&NVIC_InitStructure);
    /*-----------RTC---------------------*/

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    //PWR_BackupAccessCmd(ENABLE);
    PWR_DeInit();
    RTC_DeInit();

    PWR_BackupAccessCmd(ENABLE);
    RCC_LSICmd(ENABLE);

    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
        asm("nop");
    }
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WriteProtectionCmd(DISABLE);
    RTC_WaitForSynchro();

    RTC_InitTypeDef RCC_InitStructure;
    RTC_StructInit(&RCC_InitStructure);
    RTC_Init(&RCC_InitStructure);
    //	RTC_WriteProtectionCmd(ENABLE);
    RTC_EnterInitMode();
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_TimeStructInit(&RTC_TimeStruct);
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);

    RTC_DateTypeDef RTC_DateStruct;
    RTC_DateStructInit(&RTC_DateStruct);
    RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct);
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
    RTC_AlarmCmd(RTC_Alarm_B, DISABLE);
    // Alarm A;
    RTC_AlarmTypeDef RTC_AlarmStruct;
    RTC_AlarmStructInit(&RTC_AlarmStruct);
    //	RTC_AlarmStruct.RTC_AlarmTime.RTC_H12 = RTC_H12_AM;
    //  	RTC_AlarmStruct.RTC_AlarmTime.RTC_Hours = 3;
    //  	RTC_AlarmStruct.RTC_AlarmTime.RTC_Minutes = 20;
    //  	RTC_AlarmStruct.RTC_AlarmTime.RTC_Seconds = 20;
    // 	RTC_AlarmStruct.RTC_AlarmDateWeekDay = 2;
    //
    //  /* Alarm Date Settings : Date = 1st day of the month */
    //  	RTC_AlarmStruct.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
    //
    //  /* Alarm Masks Settings : Mask =  all fields are not masked */
    //  	RTC_AlarmStruct.RTC_AlarmMask = RTC_AlarmMask_None;
    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_B, &RTC_AlarmStruct);
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ALRMAR = 0x2032028;
    RTC->WPR = 0xFF;
    //RTC_GetDate(RTC_Format_BCD,&RTC_DateStruct);
    //EXTI->PR = 0x00400000;
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
    RTC_AlarmCmd(RTC_Alarm_B, ENABLE);
    EXTI_ClearITPendingBit(EXTI_Line22);
    EXTI_InitStructure.EXTI_Line = EXTI_Line22;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    RTC_ITConfig(RTC_IT_TS, ENABLE);
    RTC_ITConfig(RTC_IT_TAMP, ENABLE);
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);
    RTC_ITConfig(RTC_IT_ALRB, ENABLE);

    RTC_WakeUpCmd(DISABLE);
    RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div8);
    RTC_SetWakeUpCounter(0x4FFF);
    RTC_WakeUpCmd(ENABLE);
    RTC_ITConfig(RTC_IT_WUT, ENABLE);
    RTC_ClearFlag(RTC_FLAG_WUTF);
    //---------------------Configure Timer---------------------//
    //	TIM_DeInit(TIM1);
    //	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
    //	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
    //	TIM_TimeBaseInitStruct.TIM_Period = 9;
    //  	TIM_TimeBaseInitStruct.TIM_Prescaler = 99;
    //  	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    //  	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    // 	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x0000;
    //	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
    //	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
    //	TIM_Cmd(TIM1,DISABLE);
    //	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    //  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //	NVIC_Init(&NVIC_InitStructure);
}

int main(void)
{
    //	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
    SystemInit();
    init_GPIO();
    huart2->DR = 'a';
    sensorInit();
    //	PWR_DeInit();
    //	PWR_EnterSTANDBYMode();
    //	PWR_WakeUpPinCmd(ENABLE);
    //	PWR_BackupAccessCmd(ENABLE);
    //	PWR_EnterSTANDBYMode();
    //huart2->CR1|=0x202D;

    while (1)
    {

        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
        GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);
        delay(2000);
        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
        GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_SET);
//        distance = GET_Sensor();
//        GET_DISTANCE_USART(USART2, distance);

        delay(2000);
//        GET_TIME_UART(USART2);        
      USART_SendString(USART2,"nguyen huynh tai \n\r");
    }

    return 0;
}
void GET_TIME_UART(USART_TypeDef *USARTx)
{
    char str[9];
    RTC_TimeTypeDef GET_TIME;
    RTC_GetTime(RTC_Format_BIN, &GET_TIME);
    str[0] = (GET_TIME.RTC_Hours / 10) | 0x30;
    str[1] = (GET_TIME.RTC_Hours % 10) | 0x30;
    str[2] = ':';
    str[3] = (GET_TIME.RTC_Minutes / 10) | 0x30;
    str[4] = (GET_TIME.RTC_Minutes % 10) | 0x30;
    str[5] = ':';
    str[6] = (GET_TIME.RTC_Seconds / 10) | 0x30;
    str[7] = (GET_TIME.RTC_Seconds % 10) | 0x30;
    str[8] = ':';
    USART_SendString(USARTx, &str);
    USART_SendString(USARTx, "\n\r");
}

void GET_DISTANCE_USART(USART_TypeDef *USARTx, uint16_t dis)
{
    char str[3];
    str[0] = dis / 100 | 0x30;
    USART_SendData(USART2, str[0]);
    str[1] = dis / 10 % 10 | 0x30;
    USART_SendData(USART2, str[1]);
    str[2] = dis % 10 | 0x30;
    USART_SendData(USART2, str[2]);
    USART_SendString(USARTx, "\n\r");
}

void EXTI0_IRQHandler(void)
{
    /*
    // Make sure the interrupt flag is set for EXTI0
    if(EXTI_GetITStatus(EXTI_Line0) != RESET){
      static uint8_t i=0;
      
      if(i==0)
      {
        //USART_SendData(USART2,'0');
        TIM_SetCounter(TIM1,0);
        TIM1->ARR = 65535;
        TIM_Cmd(TIM1,ENABLE);
          i++;
      }
      else
      {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
      //  USART_SendData(USART2,'1');  
        TIM_Cmd(TIM1,DISABLE);
        distance = TIM1->CNT/58;
        i=0;
      }
      
        // Clear the interrupt flag
           EXTI_ClearITPendingBit(EXTI_Line0);
    }
	*/
    EXTI_Handler();
}
void TIM1_UP_TIM10_IRQHandler(void)
{
    /*
  if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
  {
    TIM_Cmd(TIM1,DISABLE);
    GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
    TIM1->ARR = 65535;
  //  TIM_SetCounter(TIM1,0);
    TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
  }
  */
    TIM_Handler();
}
