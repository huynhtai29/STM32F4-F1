/*
    EEPROM_moudle



    nguyen huynh tai
    date: 10/8/2019
*/
#ifndef __EEPROM_H
#define __EEPROM_H



#endif //__NAME_H
#ifdef __cplusplus
extern "C" {
#endif
 


#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"

void EEPROM_Init(void);
void EEPROM_DeInit(void);
uint8_t EEPROM_ReadByte(uint32_t address);
void EEPROM_WriteByte(uint32_t address, uint8_t dat);



#ifdef __cplusplus
}
#endif