#include <stdio.h>
#include "stm32f4xx.h"
#include "EEPROM.h"

#define ARRAYSIZE 50;
#define address 0x08060000
uint16_t a=7;
uint8_t dem=0;
uint32_t Address=0x08060000;
USART_TypeDef *huart2= USART2;
uint8_t RxBuffer[31];
uint8_t RxBuffer1[31]="ngay 26-2-1999 gia tri cam bien";
uint8_t i=0;
DMA_InitTypeDef DMA_InitStructure1;
// uint8_t Rx1Buffer[16]="nguyen huynh tai";

//  Delays number of Systicks (happens every 1 ms)

// Delay ms 
void delay(unsigned int time_out)
{
	unsigned int t1,t2;
	for (t1= 0; t1 < time_out; t1++)
	{
		t2=time_out;
		while (t2>0)
		{
			t2--;
			asm("nop");
		}
		
		
	}
	
	
}
void init_GPIO() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// ---------- GPIO  for LEDS -------- //
	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15;
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Configure PA0 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		// PA0 is connected to high, so use pulldown resistor
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;// PA2 is connected to high, so use pulldown resistor
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;// PA3 is connected to high, so use pulldown resistor
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);	
	// uart
	USART_InitStructure.USART_BaudRate=115200; 
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);
	
	USART_Cmd(USART2,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,DISABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
//	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	//USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	// NVIC_UART
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                // Function name for EXTI_Line0 interrupt handler
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;    // Set priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;           // Set sub priority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 // Enable the interrupt
    NVIC_Init(&NVIC_InitStructure);  

	
    /* Connect EXTI Line0 to PA0 pin (i.e. EXTI0CR[0])*/
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    // SYSCFG->EXTICR[0] &= SYSCFG_EXTICR1_EXTI0_PA;		// Same as above, but with direct register access
    
    /* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;			   // PA0 is connected to EXTI0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	// Interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Trigger on Rising edge (Just as user presses btn)
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			   // Enable the interrupt
	EXTI_Init(&EXTI_InitStructure);						   // Initialize EXTI

	/* Enable and set priorities for the EXTI0 in NVIC */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			 // Function name for EXTI_Line0 interrupt handler
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // Set priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		 // Set sub priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // Enable the interrupt
	NVIC_Init(&NVIC_InitStructure);
	// UART 2

	/*-------------------------- WATCH DOG ------------------------ */
	// INDEPENDENT WATCH DOG
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload(0xFFF);
	//DMA1
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel=DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 31;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//	DMA1_Stream5->M1AR=(uint32_t)&Rx1Buffer;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream5,ENABLE);
	//send values to DMA registers

	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure1.DMA_Channel=DMA_Channel_4;
	DMA_InitStructure1.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	DMA_InitStructure1.DMA_Memory0BaseAddr = (uint32_t)RxBuffer1;
	DMA_InitStructure1.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure1.DMA_BufferSize =1;
	DMA_InitStructure1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure1.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure1.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure1.DMA_MemoryBurst=DMA_MemoryBurst_Single;
	DMA_InitStructure1.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
	DMA_InitStructure1.DMA_FIFOMode=DMA_FIFOMode_Disable;
	DMA_InitStructure1.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_Init(DMA1_Stream6, &DMA_InitStructure1);
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
//	DMA_ITConfig(DMA1_Stream6,DMA_IT_FE,ENABLE);
	DMA_Cmd(DMA1_Stream6,DISABLE);
/*	Enable DMA1 channel IRQ Channel	*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


int main(void) {
	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
	SystemInit();
	init_GPIO();
//	EEPROM_Init();	
//	EEPROM_DeInit();
//	FLASH_Unlock();
//	FLASH_EraseSector(FLASH_Sector_7,VoltageRange_1);
//	FLASH->CR &= CR_PSIZE_MASK;
//  FLASH->CR |= FLASH_PSIZE_BYTE;
//  FLASH->CR |= FLASH_CR_PG;
//	*((volatile uint8_t*)0x08060000) = 0x01;
//	FLASH->CR &= (~FLASH_CR_PG);
//	FLASH_Lock();
//	EEPROM_Init();
	huart2->DR = 'a';

while(1){
		//if(GPIOA->IDR & 0x01)
		//DMA_Cmd(DMA1_Stream5,ENABLE);		
		//IWDG_Enable();
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);						
		delay(2000);
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);	
		delay(2000);
		//huart2->DR='a';
		//USART_SendString(USART2,"nguyen huynh tai gia tri nhiet do\r\n");
		//str++;
		//huart2->DR = huart2->DR;				
		//IWDG_ReloadCounter();
	//	DMA_Cmd(DMA1_Stream6,ENABLE);
	}
	return 0;
}

void DMA1_Stream5_IRQHandler(void) 
{
  if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5)==SET)
  {
    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
//	DMA_Cmd(DMA1_Stream6,ENABLE);
//	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
//	DMA_InitStructure1.DMA_Memory0BaseAddr=DMA1_Stream5->M0AR;
//	DMA1_Stream6->M0AR=DMA_InitStructure1.DMA_Memory0BaseAddr;
	DMA_Cmd(DMA1_Stream6,ENABLE);
    DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);
  }
  
}
void DMA1_Stream6_IRQHandler(void)
{	

	 if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6)==SET)
	{	
		while (USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET)
		{
			asm("nop");
			
		}		
		if(DMA_InitStructure1.DMA_Memory0BaseAddr==((uint32_t)RxBuffer1+31))
		{
			DMA_InitStructure1.DMA_Memory0BaseAddr=(uint32_t)RxBuffer1-1;
			DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
			DMA_Cmd(DMA1_Stream6,DISABLE);
		}
		else
		{		
			DMA_InitStructure1.DMA_Memory0BaseAddr+=1;
	    	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
			DMA1_Stream6->M0AR=DMA_InitStructure1.DMA_Memory0BaseAddr;
			DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
			DMA_Cmd(DMA1_Stream6,ENABLE);
		}
		
	
  	}	
//	if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_FE)==SET)
//	{
//		GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
//		DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_FE);
//	}
}

void EXTI0_IRQHandler(void) 
{
    // Make sure the interrupt flag is set for EXTI0
    if(EXTI_GetITStatus(EXTI_Line0) != RESET){
        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
        DMA_Cmd(DMA1_Stream6,ENABLE);
		USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
        // Clear the interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void USART2_IRQHandler(void) 
{
 
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==SET)
  	{
	
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
		USART_DMACmd(USART2,USART_DMAReq_Rx,DISABLE);	

//------------------------Flash Memory------------------------
/*
    	//	FLASH->CR &= CR_PSIZE_MASK;
    	//    FLASH->CR |= FLASH_PSIZE_BYTE;
    	//    FLASH->CR |= FLASH_CR_PG;
    	//	*((volatile uint8_t*)0x08060000+dem) = temp;
    	//	FLASH->CR &= (~FLASH_CR_PG);
    	//	dem++;
    	//	EEPROM_Init();
    	//	EEPROM_WriteByte(Address,(uint8_t)temp);
    	//	FLASH_Unlock();
    	//	FLASH_ProgramByte(Address,(uint8_t)temp);
*/
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);	
  	}
   if(USART_GetITStatus(USART2,USART_IT_TC)==SET)
  	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		USART_ClearITPendingBit(USART2,USART_IT_TC);
  	}
//USART_SendData(USART2,'a');
}