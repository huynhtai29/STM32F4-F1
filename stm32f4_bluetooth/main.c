/*
Barebones Blinky example MacOSX using arm-none-eabi and ST_Link

This is sample code to blink the user LEDs using three different methods:
1. Use the STD_Periph driver functions
2. Use the STD_Periph driver structures, NOT functions
3. Use assembly level functions defined in CortexM4asmOps.asm
*/
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rtc.h"
//#include "sensor.h"
volatile uint32_t count = 0;
volatile uint8_t state = 0;
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
void RunMotor(BitAction _state, uint32_t _value)
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_9,_state);
    count = _value;
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4,ENABLE);
}

void Enable_Motor(BitAction _state)
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_9,_state);
    TIM_Cmd(TIM4,ENABLE);

}

void Disable_Motor()
{
  TIM_Cmd(TIM4,DISABLE);
}

void init_GPIO()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_OCInitTypeDef OC_TIM_InitStructure;

    // ---------- Enable Clock Peripheral -------- //
    /* Enable GPIO Clock */    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /*Enable USART2 Clock*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /*Enable TIM4 Clock*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    // Configure PD12, PD13, PD14 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    //GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
    // ---------- GPIO  for Push Button -------- //
    //-----Config GPIO_PB8-TIM4-CH3------//
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    /* Configure PA0 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		// PA0 is connected to high, so use pulldown resistor
    GPIO_Init(GPIOA, &GPIO_InitStructure);

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
    USART_InitStructure.USART_BaudRate = 9600; // Saniye içinde hat üzerinden kaç tane bit gönderilmesi gerektigi
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

    //----Config TIM------///
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Period = 999;
    TIM_InitStructure.TIM_Prescaler = 25;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4,&TIM_InitStructure);
    TIM_Cmd(TIM4,DISABLE);
    TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
    OC_TIM_InitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    OC_TIM_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
    OC_TIM_InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    OC_TIM_InitStructure.TIM_Pulse = 499 ;
    TIM_OC3Init(TIM4,&OC_TIM_InitStructure);
    TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /*-------------Config EXTI0 --- PA0----------------*/
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

int main(void)
{
    //	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
    SystemInit();
    init_GPIO();
    GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
    RunMotor(1,3000);
    while (1)
    {

        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
        delay(2000);
        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
        delay(2000);
    }

    return 0;
}
void EXTI0_IRQHandler(void)
{
  GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
  state++;
  if(state == 1)
  {
    TIM_Cmd(TIM4,ENABLE);
  }
  else
  {
    TIM_Cmd(TIM4,DISABLE);
    state  = 0;
  }
  EXTI_ClearITPendingBit(EXTI_Line0);
}

void TIM4_IRQHandler(void)
{
  if(TIM_GetFlagStatus(TIM4,TIM_IT_Update)== SET)
  {
    count--;
    if(count == 0)
    {    
    TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
    TIM_Cmd(TIM4,DISABLE);
    GPIO_WriteBit(GPIOD,GPIO_Pin_15,RESET);
    GPIO_WriteBit(GPIOD,GPIO_Pin_14,RESET);
    }
//    GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
  }
}
void USART2_IRQHandler(void) 
{

  if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)!=RESET)
  {
      char tmp = USART_ReceiveData(USART2);
      if(tmp == 49) 
      {
        GPIO_WriteBit(GPIOD,GPIO_Pin_15,ENABLE);
        GPIO_WriteBit(GPIOB,GPIO_Pin_9,ENABLE);
        TIM_Cmd(TIM4,ENABLE);
        TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
        count = 3000;
      }
      else if(tmp == 50)
      {
        
        GPIO_WriteBit(GPIOD,GPIO_Pin_14,ENABLE);
        GPIO_WriteBit(GPIOB,GPIO_Pin_9,DISABLE);
        TIM_Cmd(TIM4,ENABLE);
        TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
        count = 3000;

      }
      else if(tmp == 51)
      {
        GPIO_WriteBit(GPIOD,GPIO_Pin_15,DISABLE);
        GPIO_WriteBit(GPIOD,GPIO_Pin_14,DISABLE);
        TIM_Cmd(TIM4,DISABLE);
      }
      
      USART_ClearFlag(USART2,USART_FLAG_RXNE);
  }
}