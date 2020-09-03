#include <stdio.h>
#include "stm32f4xx.h"
#include "EEPROM.h"
#include "USART_DMA.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "ff.h"        
uint16_t a=7;
uint8_t dem=0;
uint32_t Address=0x08060000;
USART_TypeDef *huart2= USART2;
uint8_t i=0;
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

SD_Error Status = SD_OK;

FATFS filesystem;		/* volume lable */
FILINFO fileinfo;
FRESULT ret;			  /* Result code */

FIL file;				    /* File object */

DIR dir;				    /* Directory object */

FILINFO fno;			  /* File information object */

UINT bw, br;

uint8_t buff[128];

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

}


int main(void) {
	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
	SystemInit();
	init_GPIO();
	USART_DMA_Init();

//	SD_Init();
//	SD_PowerON();
//	SD_InitializeCards();
//	SDIO_Config();
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
//	huart2->DR = 'a';
  delay(10000);
USART_DMA_Sendstring(DMA1_Stream6,"nguyen huynh tai\n\r");
ret=f_mount(0, &filesystem);
  if (ret != FR_OK) {
    USART_SendString(USART2,"could not open filesystem \n\r");
  }
USART_SendString(USART2,"Create a new file (hello.txt)\n\r");
ret = f_open(&file, "HELLO.TXT", FA_CREATE_ALWAYS|FA_WRITE);



	delay(100);
//sret = f_write(&file, "Hello world!", 14, &bw);
//sret = f_close(&file);
  if (ret) {
    USART_SendString(USART2,"Create a new file error\n\r");
 //   fault_err(ret);
  }
//  else {
//   USART_SendString(USART2,"Write a text data. (hello.txt)\n\r");
//    ret = f_write(&file, "Hello world!", 14, &bw);
//    if (ret) {
//      USART_SendString(USART2,"Write a text data to file error\n\r");
//    } 
//    delay(50);
    USART_SendString(USART2,"Close the file\n\r");
    ret = f_close(&file);
//    if (ret) {
//      USART_SendString(USART2,"Close the hello.txt file error\n\r");			
//    }


while(1){
	

	//GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);						
	//delay(2000);
	//GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);	

	//USART_DMA_Sendstring(DMA1_Stream6,"nguyen huynh tai 29/1111-- \r\n");
	
	}
	return 0;
}
void DMA1_Stream5_IRQHandler(void) 
{
	DMA_Rx_Handler();
}
void DMA1_Stream6_IRQHandler(void)
{	
	DMA_Tx_Handler();
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
	
	USART_DMA_Sendbyte(DMA1_Stream6,USART_DMA_Received_1Byte());
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);	
  	}
   if(USART_GetITStatus(USART2,USART_IT_TC)==SET)
  	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		USART_ClearITPendingBit(USART2,USART_IT_TC);
  	}
//USART_SendData(USART2,'a');
}
void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}