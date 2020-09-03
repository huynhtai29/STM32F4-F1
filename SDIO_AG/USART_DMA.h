/*
    USART_DMA


    nguyen huynh tai
    date: 14/8/2019
*/
#ifndef USART_DMA
#define USART_DMA

#endif //USART_DMA
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"


extern uint8_t size;
extern uint8_t TxBuffer[255];
extern uint8_t RxBuffer[255];
extern char *_address;

void USART_DMA_Sendstring(DMA_Stream_TypeDef* DMAy_Streamx,char str[]);
void USART_DMA_Sendbyte(DMA_Stream_TypeDef* DMAy_Streamx,char data);
uint8_t USART_DMA_Received_1Byte();
void USART_DMA_Init();
void DMA_Tx_Handler();
void DMA_Rx_Handler();
#ifdef __cplusplus
}
#endif