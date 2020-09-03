#include "stdio.h"
#include "stdarg.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

void my_printf_config(void);
void my_printf(USART_TypeDef* USARTx,char *buff, ...);
void print_int(USART_TypeDef* USARTx,int num);
