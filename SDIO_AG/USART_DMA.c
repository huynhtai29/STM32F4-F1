#include "USART_DMA.h"
uint8_t size;
uint8_t TxBuffer[255];
uint8_t RxBuffer[255];
char *_address;
uint8_t state;
USART_InitTypeDef   USART_InitStructure;
NVIC_InitTypeDef    NVIC_InitStructure;
DMA_InitTypeDef     DMA_InitStructure;
DMA_InitTypeDef	    DMA_InitStructure1;



void USART_DMA_Sendbyte(DMA_Stream_TypeDef* DMAy_Streamx,char data)
{
    size =0;
    RxBuffer[size]=data;
    DMA_Cmd(DMAy_Streamx,ENABLE);
}
void USART_DMA_Sendstring(DMA_Stream_TypeDef* DMAy_Streamx,char str[50])
{
	state=0;
   uint8_t dem=0;
   size =0;
    while (str[dem++]!='\0')
    {
		asm("nop");
    }
    size = dem;
//   for( uint8_t i=0;i<=size;i++)
//    {
//        RxBuffer[i]=str[i];
//    }

	
    _address = str;
	DMA1_Stream6->M0AR=_address;
    DMA_Cmd(DMAy_Streamx,ENABLE);
	while(state==0)
	{
		asm("nop");
	}
}
uint8_t USART_DMA_Received_1Byte()
{
    static uint8_t dem1=0;
    return TxBuffer[dem1++]; 
}
void USART_DMA_Init()
{   

    USART_InitStructure.USART_BaudRate=9600; 
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);
	
	USART_Cmd(USART2,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,DISABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
//  NVIC_UART
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                // Function name for EXTI_Line0 interrupt handler
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;    // Set priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;           // Set sub priority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 // Enable the interrupt
    NVIC_Init(&NVIC_InitStructure);  

	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel=DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	//DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)*_address;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 255;
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
	DMA_InitStructure1.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
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
void DMA_Tx_Handler(void)
{	

	 if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6)==SET)
	{	
		while (USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET)
		{
			asm("nop");
			
		}		
		if(size==0)
		{
			
			state=1;
		//	GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
			DMA_Cmd(DMA1_Stream6,DISABLE);
			DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
		}
		else
		{
			size--;
			DMA_InitStructure1.DMA_Memory0BaseAddr+=1;
	    //	GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_SET);
			DMA1_Stream6->M0AR+=1;
			DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
			DMA_Cmd(DMA1_Stream6,ENABLE);
		}
  	}	

}
void DMA_Rx_Handler(void) 
{
  if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5)==SET)
  {
    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
	DMA_Cmd(DMA1_Stream6,ENABLE);
    DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);
  }
  
}