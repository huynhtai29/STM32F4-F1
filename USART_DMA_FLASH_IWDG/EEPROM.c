#include "EEPROM.h"
#define RDM_PARAMETTER_OFFSET_ADDR 0x08060000

void EEPROM_Init(void)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP| FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR|FLASH_FLAG_OPERR|FLASH_FLAG_PGPERR|FLASH_FLAG_RDERR|FLASH_FLAG_RDERR);
    if (FLASH_COMPLETE != FLASH_EraseSector(FLASH_Sector_7,VoltageRange_1))
    {
        FLASH_Lock();
        return;// Cannot erase Page
    }
}

void EEPROM_DeInit(void)
{
    FLASH_Lock();
}

void EEPROM_WriteByte(uint32_t address, uint8_t dat)
{
    uint32_t _realAddress;
    _realAddress = (address - RDM_PARAMETTER_OFFSET_ADDR) * 2 + RDM_PARAMETTER_OFFSET_ADDR;
    if (FLASH_COMPLETE != FLASH_ProgramHalfWord(_realAddress, (uint16_t)dat))
    {
        FLASH_Lock();
        return; // Cannot write flash here
    }
}

uint8_t EEPROM_ReadByte(uint32_t address)
{
    uint32_t _realAddress;
    _realAddress = (address - RDM_PARAMETTER_OFFSET_ADDR) * 2 + RDM_PARAMETTER_OFFSET_ADDR;
    return (uint8_t)(*(__IO uint16_t*)_realAddress);
}