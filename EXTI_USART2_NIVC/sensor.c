#include "sensor.h"


NVIC_InitTypeDef NVIC_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
GPIO_InitTypeDef  GPIO_InitStructure;
volatile uint8_t state;
uint32_t dis;
void sensorInit()
{
    /*-----------Clock-------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /*-------------Config TIMx------------*/
    TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_TimeBaseInitStruct.TIM_Period = 9;
  	TIM_TimeBaseInitStruct.TIM_Prescaler = 99;
  	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
 	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM1,DISABLE);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
    /*-------------Config EXTI------------------*/
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;			   // PA0 is connected to EXTI0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	// Interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Trigger on Rising edge (Just as user presses btn)
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			   // Enable the interrupt
	EXTI_Init(&EXTI_InitStructure);						   // Initialize EXTI
	/* Enable and set priorities for the EXTI0 in NVIC */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			 // Function name for EXTI_Line0 interrupt handler
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // Set priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		 // Set sub priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // Enable the interrupt
	NVIC_Init(&NVIC_InitStructure);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    /*--------------Config GPIO------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		// PA0 is connected to high, so use pulldown resistor
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
int16_t GET_Sensor()
{   
    state = 0;
    GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET);
    TIM1->ARR = 9;
    TIM_Cmd(TIM1,ENABLE);
    while (state==0)
    {
        asm("nop");
    }

    return (uint16_t)dis;
}

void EXTI_Handler(void) 
{
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
        dis = TIM1->CNT/58;
        i=0;
        state = 1;
      }
        // Clear the interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void TIM_Handler(void)
{
  if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
  {
    TIM_Cmd(TIM1,DISABLE);
    GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
    TIM1->ARR = 65535;
    TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
  }
}
