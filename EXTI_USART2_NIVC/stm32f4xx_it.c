#include "stm32f4xx_it.h"
#include "stm32f4xx_conf.h"

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SysTick_Handler(void)
{

}

void USART2_IRQHandler(void) 
{
//  if(USART_GetFlagStatus(USART2,USART_FLAG_FE)!=RESET)
//  {
//    	GPIO_ToggleBits(GPIOD,GPIO_Pin_12);	
//      USART_SendData(USART2,'a');
//      USART_ClearITPendingBit(USART2,USART_IT_FE);
//
//
// }
  if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)!=RESET)
  {
      char tmp = USART_ReceiveData(USART2); 
      USART_SendData(USART2, tmp);
  }
}
//USART_SendData(USART2,'a');
void RTC_WKUP_IRQHandler(void) 
{
if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {
		//    PWR_BackupAccessCmd(ENABLE);
//  	GET_TIME_UART(USART2);
    RTC_ClearFlag(RTC_FLAG_WUTF);
  //  PWR_BackupAccessCmd(DISABLE);
  //PWR_ClearFlag(PWR_FLAG_WU);
	}
 	EXTI->PR=0x00400000;


}

void RTC_Alarm_IRQHandler(void)
{
    if(RTC_GetITStatus(RTC_IT_ALRA)==SET)
    {
      USART_SendData(USART2,'1');
      RTC_ClearITPendingBit(RTC_IT_ALRA);
    }
    if(RTC_GetITStatus(RTC_IT_ALRB)==SET)
    {
      USART_SendData(USART2,'2');
      RTC_ClearITPendingBit(RTC_IT_ALRB);
    }
  
    EXTI_ClearITPendingBit(EXTI_Line17);
}